#!/usr/bin/env python3
"""Live Sassy dashboard: D6 lidar plus OAK-D Lite RGB and stereo depth."""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io, json, math, statistics, subprocess, threading, time
from pathlib import Path

import depthai as dai
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw
import serial

LIDAR_PORT = "/dev/ttyTHS1"; LIDAR_BAUD = 230400; UNO_PORT = "/dev/leia-uno"; UNO_BAUD = 115200; HTTP_PORT = 8080
SIZE = 800; RANGE_M = 4.0; LIDAR_FORWARD_DEG = 90.0
# Steering is reserved for a nearby wall in front of the robot.  The previous
# 1.56 m / +/-75 deg envelope treated distant room walls as active hazards.
OBSTRUCTION_DEPTH_M = 1.50; OBSTRUCTION_WIDTH_M = 1.217; CAMERA_FPS = 15
MIN_VALID_DISTANCE_M = .30; TRIAL_S = 5.0; FORWARD_SPEED = 255
FORWARD_ARC_DEG = 55.0; CLEARANCE_S = .5
TRIAL_VIDEO = Path("artifacts/navigation_trial_5s_right_turn_fix.mp4")
state = {"scans": deque(maxlen=3), "rgb": None, "depth": None, "depth_mm": None, "trial": "ready", "events": deque(maxlen=24), "lock": threading.Lock()}

def record_event(message):
    """Keep a compact operator-facing record of sensor geometry and action."""
    entry = f"{time.strftime('%H:%M:%S')}  {message}"
    with state["lock"]:
        if not state["events"] or state["events"][-1] != entry:
            state["events"].append(entry)

def angle_error(angle, reference): return (angle-reference+180.0) % 360.0 - 180.0

def pivot_command(turn_right):
    """Physical trial: slot 1 forward pivots right; slot 2 pivots left."""
    return f"D,{FORWARD_SPEED},0" if turn_right else f"D,0,{FORWARD_SPEED}"

def lidar_clusters(scan):
    ordered=sorted((a,d) for a,d in scan if MIN_VALID_DISTANCE_M <= d <= OBSTRUCTION_DEPTH_M)
    groups=[]; group=[]
    for item in ordered:
        if group and (item[0]-group[-1][0] > 3 or abs(item[1]-group[-1][1]) > .20): groups.append(group); group=[]
        group.append(item)
    if group: groups.append(group)
    return [(sum(a for a,_ in group)/len(group),sum(d for _,d in group)/len(group),len(group)) for group in groups if len(group) >= 3]

def lidar_obstacle(scan):
    candidates=[]
    for angle,distance,count in lidar_clusters(scan):
        offset=angle_error(angle,LIDAR_FORWARD_DEG)
        # Cover the full forward safety arc, including forward-left/right walls.
        if abs(offset) <= FORWARD_ARC_DEG: candidates.append((angle,distance,count))
    return min(candidates,key=lambda item:item[1]) if candidates else None

def camera_obstacle(depth_mm):
    """Classify an obstacle from calibrated stereo depth sectors, not a room-specific mask."""
    if depth_mm is None: return None
    height,width=depth_mm.shape
    # Use the middle of the image.  The lower portion views the floor directly
    # in front of Sassy and caused false "wall" reports after a real wall cleared.
    region=depth_mm[height//4:height*3//5,:]
    candidates=[]
    for side,start,end in (("left",0,width//3),("center",width//3,2*width//3),("right",2*width//3,width)):
        values=region[:,start:end]; valid=values[(values >= 300) & (values <= int(OBSTRUCTION_DEPTH_M*1000))]
        if valid.size >= 40: candidates.append((side,float(np.percentile(valid,20))/1000))
    return min(candidates,key=lambda item:item[1]) if candidates else None

def command_for_obstacles(scan,depth_mm, source_filter=None):
    """LiDAR is authoritative for steering; OAK is logged as corroboration only."""
    hit=lidar_obstacle(scan)
    camera_hit=camera_obstacle(depth_mm)
    if hit and source_filter in (None, "lidar"):
        offset=angle_error(hit[0],LIDAR_FORWARD_DEG); distance=hit[1]
        turn_right=offset <= 10
        source="lidar"
    else:
        # Stereo depth can see floor texture/background patches as close depth.
        # Without a LiDAR cluster we therefore keep travelling forward rather
        # than creating a turn from an unverified camera-only observation.
        if camera_hit and source_filter is None:
            side, distance = camera_hit
            return None, f"OAK-DEPTH: {distance * 1000:.0f} mm in {side}; unconfirmed -> FORWARD", None
        return None, "CLEAR -> FORWARD", None
    # The last navigation trial showed slot 2 physically pivoted left.
    action = "TURN RIGHT" if turn_right else "TURN LEFT"
    command = pivot_command(turn_right)
    oak_note = ""
    if camera_hit:
        oak_note = f"; OAK {camera_hit[0]} {camera_hit[1] * 1000:.0f} mm"
    return command, f"{source.upper()}: wall {distance * 1000:.0f} mm at {offset:+.0f} deg -> {action}{oak_note}", source

def run_trial():
    """Use the dashboard's already-live sensor frames for one guarded motion test."""
    video = None
    uno = None
    recording_stop = threading.Event()
    recording_thread = None
    with state["lock"]:
        if state.get("active", False): return
        state["active"] = True
        state["trial"] = "arming"
    try:
        if True:
            uno = serial.Serial(UNO_PORT,UNO_BAUD,timeout=.1,write_timeout=.2)
            uno.write(b"S\n"); uno.flush(); time.sleep(1.5) # Uno resets on serial open.
            deadline=time.monotonic()+5
            while time.monotonic() < deadline:
                with state["lock"]: ready=bool(state["scans"]) and state["depth_mm"] is not None
                if ready: break
                time.sleep(.05)
            if not ready: raise RuntimeError("LiDAR or OAK depth is unavailable")
            with state["lock"]: state["trial"]="running"
            started=time.monotonic(); latched_turn=None; latched_source=None; latched_description=None; clear_started=None; last_logged=None; last_log_time=0.0
            video = start_trial_video()
            def record_frames():
                while not recording_stop.is_set():
                    tick = time.monotonic()
                    with state["lock"]: label = state["trial"]
                    write_trial_frame(video, label)
                    recording_stop.wait(max(0, .1 - (time.monotonic()-tick)))
            recording_thread = threading.Thread(target=record_frames, daemon=True)
            recording_thread.start()
            started=time.monotonic()
            last_sent=started; max_gap=0
            while time.monotonic()-started < TRIAL_S:
                with state["lock"]:
                    scan=list(state["scans"][-1]) if state["scans"] else []
                    depth=None if state["depth_mm"] is None else state["depth_mm"].copy()
                    scan_age=time.monotonic()-state.get("scan_time",0)
                if scan_age > .5:
                    raise RuntimeError(f"LiDAR stale ({scan_age:.2f}s); stopping")
                command,description,source = command_for_obstacles(scan,depth,latched_source)
                if command is not None:
                    # Choose an escape direction once, then retain it until the
                    # sensor that found the wall reports a genuine clearance.
                    if latched_turn is None:
                        latched_turn=command; latched_source=source; latched_description=description
                    command=latched_turn
                    action="TURN RIGHT" if command == pivot_command(True) else "TURN LEFT"
                    description=description.split(" -> ")[0] + " -> " + action
                    clear_started=None
                elif latched_turn is not None:
                    if clear_started is None:
                        clear_started=time.monotonic()
                    if time.monotonic()-clear_started < CLEARANCE_S:
                        command=latched_turn
                        action="TURN RIGHT" if command == pivot_command(True) else "TURN LEFT"
                        description=f"CLEAR -> hold {action} for {CLEARANCE_S:.1f}s"
                    else:
                        latched_turn=None; latched_source=None; latched_description=None; command=f"D,{FORWARD_SPEED},{FORWARD_SPEED}"; description="CLEAR -> FORWARD"
                else:
                    command=f"D,{FORWARD_SPEED},{FORWARD_SPEED}"
                now=time.monotonic(); max_gap=max(max_gap,now-last_sent); last_sent=now
                uno.write((command+"\n").encode()); uno.flush()
                description += f" [sent {command}]"
                with state["lock"]: state["trial"]=description
                now=time.monotonic()
                if description != last_logged or now-last_log_time >= .5:
                    record_event(description)
                    last_logged=description; last_log_time=now
                time.sleep(.08)
            uno.write(b"S\n"); uno.flush()
            with state["lock"]: state["trial"]="complete: stop sent"
            record_event("STOP")
            record_event(f"Maximum command interval: {max_gap*1000:.0f} ms")
    except Exception as exc:
        with state["lock"]: state["trial"]=f"error: {exc}"
        record_event(f"ERROR: {exc}")
    finally:
        if uno is not None:
            try:
                uno.write(b"S\n"); uno.flush()
            finally:
                uno.close()
        recording_stop.set()
        if recording_thread is not None: recording_thread.join(5)
        finish_trial_video(video)
        with state["lock"]: state["active"]=False

def read_lidar():
    buf=bytearray(); scan=[]; previous=None
    with serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=.1) as uart:
        while True:
            buf.extend(uart.read(4096))
            while True:
                i=buf.find(b"\xaa\x55\x00\x19")
                if i < 0: buf[:]=buf[-3:]; break
                if len(buf) < i+85: del buf[:i]; break
                packet=bytes(buf[i:i+85]); del buf[:i+85]
                start=(int.from_bytes(packet[4:6], "little") >> 1)/64; end=(int.from_bytes(packet[6:8], "little") >> 1)/64
                if end < start: end += 360
                if previous is not None and start+5 < previous:
                    if scan:
                        with state["lock"]:
                            state["scans"].append(scan)
                            state["scan_time"]=time.monotonic()
                    scan=[]
                previous=start
                for n in range(25):
                    offset=10+n*3; distance=int.from_bytes(packet[offset+1:offset+3], "little")/1000
                    if .05 <= distance <= RANGE_M: scan.append(((start+(end-start)*n/24)%360, distance))

def encode(image, quality=85, bgr=False):
    """Encode a NumPy image without depending on OpenCV's binary bindings."""
    if bgr:
        image=image[:, :, ::-1]
    output=io.BytesIO(); Image.fromarray(image).save(output, format="JPEG", quality=quality)
    return output.getvalue()

def read_oak():
    """RGB and passive stereo depth run on the OAK-D Lite itself."""
    pipeline=dai.Pipeline()
    rgb=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    left=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    rgb_out=rgb.requestOutput((640,400), fps=CAMERA_FPS)
    left_out=left.requestOutput((640,400), type=dai.ImgFrame.Type.GRAY8, fps=CAMERA_FPS)
    right_out=right.requestOutput((640,400), type=dai.ImgFrame.Type.GRAY8, fps=CAMERA_FPS)
    stereo=pipeline.create(dai.node.StereoDepth); stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    left_out.link(stereo.left); right_out.link(stereo.right)
    rgb_queue=rgb_out.createOutputQueue(maxSize=1, blocking=False)
    depth_queue=stereo.depth.createOutputQueue(maxSize=1, blocking=False)
    pipeline.start()
    try:
        while pipeline.isRunning():
            frame=rgb_queue.tryGet()
            if frame is not None:
                image=frame.getCvFrame()
                # DepthAI's OpenCV frame is BGR; Pillow expects RGB.
                data=encode(image, bgr=True)
                if data:
                    with state["lock"]:
                        state["rgb"]=data
                        state["rgb_time"]=time.monotonic()
            frame=depth_queue.tryGet()
            if frame is not None:
                depth=frame.getFrame(); valid=depth>0
                normalized=np.clip((depth.astype(np.float32)-300)/4700,0,1)
                # Near = red/yellow, far = blue; invalid pixels are dark.
                image=np.empty((*depth.shape,3),dtype=np.uint8)
                image[...,0]=(255*np.clip(1.5-abs(4*normalized-3),0,1)).astype(np.uint8)
                image[...,1]=(255*np.clip(1.5-abs(4*normalized-2),0,1)).astype(np.uint8)
                image[...,2]=(255*np.clip(1.5-abs(4*normalized-1),0,1)).astype(np.uint8)
                image[~valid]=(20,24,32)
                data=encode(image)
                if data:
                    with state["lock"]:
                        state["depth"]=data
                        state["depth_mm"]=depth.copy()
                        state["depth_time"]=time.monotonic()
            time.sleep(.002)
    finally: pipeline.stop()

def make_lidar_image():
    with state["lock"]: scans=list(state["scans"])
    bins=[[] for _ in range(720)]
    for scan in scans:
        for angle,distance in scan: bins[int(angle*2)%720].append(distance)
    points=[(i/2,statistics.median(values)) for i,values in enumerate(bins) if values]
    image=Image.new("RGB", (SIZE,SIZE), "#101820"); draw=ImageDraw.Draw(image); c=SIZE//2; scale=SIZE/(2*RANGE_M)
    for meters in (1,2,3,4):
        radius=meters*scale; draw.ellipse((c-radius,c-radius,c+radius,c+radius), outline="#2e4352"); draw.text((c+5,c-radius+4),f"{meters} m",fill="#87a1b1")
    draw.line((c-15,c,c+15,c),fill="white",width=2); draw.line((c,c-15,c,c+15),fill="white",width=2)
    half=(OBSTRUCTION_WIDTH_M/2)*scale; top=c-OBSTRUCTION_DEPTH_M*scale
    draw.rectangle((c-half,top,c+half,c),outline="#ff4b4b",width=3); draw.text((c+half+6,top+4),"obstruction zone",fill="#ff7676")
    draw.text((15,15),"Sassy / WitMotion D6 live lidar",fill="white"); draw.text((15,40),"Top = robot forward; centre = lidar",fill="#b6cbd8")
    xy=[]
    for angle,distance in points:
        relative=math.radians(angle_error(angle,LIDAR_FORWARD_DEG)); xy.append((c+distance*math.sin(relative)*scale,c-distance*math.cos(relative)*scale,distance))
    for first,second in zip(xy,xy[1:]):
        if abs(first[2]-second[2]) < .12: draw.line((first[0],first[1],second[0],second[1]),fill="#48d7bd",width=3)
    for x,y,distance in xy:
        blocked=c-half <= x <= c+half and top <= y <= c
        color="#ff4b4b" if blocked else ("#52616b" if distance < .30 else "#f4c95d")
        draw.ellipse((x-3,y-3,x+3,y+3),fill=color)
    out=io.BytesIO(); image.save(out,format="JPEG",quality=88); return out.getvalue()

def placeholder(label):
    image=Image.new("RGB",(640,400),(16,24,32)); ImageDraw.Draw(image).text((35,200),label,fill=(220,220,220))
    output=io.BytesIO(); image.save(output,format="JPEG",quality=85); return output.getvalue()

def dashboard_image(data, size, label):
    if not data:
        return Image.new("RGB", size, "#101820")
    try:
        return Image.open(io.BytesIO(data)).convert("RGB").resize(size)
    except Exception:
        return Image.new("RGB", size, "#101820")

def start_trial_video():
    """Capture the actual control trial from the dashboard's live sensor frames."""
    TRIAL_VIDEO.parent.mkdir(exist_ok=True)
    return subprocess.Popen([
        imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "1280x720",
        "-framerate", "10", "-i", "-", "-an", "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p", str(TRIAL_VIDEO),
    ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def make_trial_frame(description, events=None):
    """The shared four-pane renderer for video frames and one-shot PNGs."""
    lidar_jpeg=make_lidar_image()
    with state["lock"]:
        rgb=state["rgb"]; depth=state["depth"]
        if events is None: events=list(state["events"])[-7:]
    canvas=Image.new("RGB", (1280,720), "#101820")
    canvas.paste(dashboard_image(lidar_jpeg, (640,360), "LiDAR"), (0,0))
    canvas.paste(dashboard_image(rgb, (640,360), "OAK RGB"), (640,0))
    canvas.paste(dashboard_image(depth, (640,360), "OAK depth"), (640,360))
    draw=ImageDraw.Draw(canvas)
    draw.rectangle((0,360,640,720), fill="#17242d")
    draw.text((18,380), "WALL POSITION / ROBOT RESPONSE", fill="white")
    draw.text((18,410), description, fill="#ffcf5a")
    y=450
    for event in events:
        draw.text((18,y),event,fill="#b9d8e7"); y += 34
    return canvas

def write_trial_frame(video, description):
    if video is None or video.stdin is None:
        return
    canvas=make_trial_frame(description)
    try:
        video.stdin.write(canvas.tobytes())
    except (BrokenPipeError, OSError):
        pass

def finish_trial_video(video):
    if video is None:
        return
    try:
        if video.stdin:
            video.stdin.close()
        video.wait(timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        video.kill()

def stream(handler, source):
    handler.send_response(200); handler.send_header("Cache-Control","no-cache"); handler.send_header("Content-Type","multipart/x-mixed-replace; boundary=frame"); handler.end_headers()
    try:
        while True:
            data=source(); handler.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+str(len(data)).encode()+b"\r\n\r\n"+data+b"\r\n"); handler.wfile.flush(); time.sleep(.08)
    except (BrokenPipeError,ConnectionResetError): pass

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/snapshot.png":
            with state["lock"]:
                now=time.monotonic()
                ready=(bool(state["scans"]) and state["rgb"] is not None and
                       state["depth_mm"] is not None and
                       all(now-state.get(key,0) < 1.0 for key in
                           ("scan_time", "rgb_time", "depth_time")))
                if ready:
                    scan=list(state["scans"][-1]); depth=state["depth_mm"].copy()
            if not ready:
                self.send_error(503, "Waiting for fresh LiDAR, RGB, and depth frames")
                return
            command, measurement, _=command_for_obstacles(scan,depth)
            if command is None:
                label=f"WOULD FORWARD [D,{FORWARD_SPEED},{FORWARD_SPEED}]"
            else:
                action="TURN RIGHT" if command == pivot_command(True) else "TURN LEFT"
                label=f"WOULD {action} [{command}]"
            canvas=make_trial_frame(label,[measurement,"Preview only; motors were not commanded"])
            output=io.BytesIO(); canvas.save(output,format="PNG")
            body=output.getvalue()
            self.send_response(200); self.send_header("Content-Type","image/png")
            self.send_header("Cache-Control","no-store")
            self.send_header("Content-Length",str(len(body))); self.end_headers()
            self.wfile.write(body)
            return
        downloads = {
            "/artifacts/fused_wall_capture_10s.mp4": (Path("artifacts/fused_wall_capture_10s.mp4"), "video/mp4"),
            "/artifacts/fused_wall_capture_10s.csv": (Path("artifacts/fused_wall_capture_10s.csv"), "text/csv"),
            "/artifacts/navigation_trial_5s_right_turn_fix.mp4": (TRIAL_VIDEO, "video/mp4"),
            "/artifacts/navigation_snapshot.png": (Path("artifacts/navigation_snapshot.png"), "image/png"),
        }
        if self.path in downloads:
            path, content_type = downloads[self.path]
            if not path.is_file(): self.send_error(404); return
            body = path.read_bytes()
            self.send_response(200); self.send_header("Content-Type", content_type); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
        if self.path=="/":
            body=b'''<!doctype html><title>Sassy sensors</title><style>body{background:#101820;color:#ddd;font:16px sans-serif;margin:20px}h1{text-align:center}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:16px;max-width:1800px;margin:auto}.panel{background:#17242d;padding:10px;border-radius:8px}h2{font-size:18px;margin:0 0 8px}img{width:100%;display:block}button{font-size:16px;padding:10px 16px}#status{margin:12px;text-align:center}#events{white-space:pre-wrap;min-height:110px;max-height:230px;overflow:auto;color:#b9d8e7;font:14px ui-monospace,monospace}</style><h1>Sassy live sensors</h1><p id="status">controller: loading</p><p style="text-align:center"><button onclick="fetch('/trial',{method:'POST'}).then(r=>r.text()).then(t=>status.textContent=t)">Run guarded 20-second test (85% power)</button></p><section class="panel" style="max-width:1100px;margin:0 auto 16px"><h2>Wall position and robot response</h2><div id="events">No trial events yet.</div></section><script>const status=document.querySelector('#status'),events=document.querySelector('#events');setInterval(()=>{fetch('/status').then(r=>r.text()).then(t=>status.textContent='controller: '+t);fetch('/events').then(r=>r.json()).then(x=>events.textContent=x.join('\\n')||'No trial events yet.')},500)</script><div class="grid"><section class="panel"><h2>WitMotion D6 LiDAR</h2><img src="/lidar.mjpg"></section><section class="panel"><h2>OAK-D Lite RGB</h2><img src="/camera.mjpg"></section><section class="panel"><h2>OAK-D Lite depth</h2><img src="/depth.mjpg"></section></div>'''
            body=body.replace(b"20-second test (85% power)", b"5-second test (100% power)")
            self.send_response(200); self.send_header("Content-Type","text/html"); self.send_header("Content-Length",str(len(body))); self.end_headers(); self.wfile.write(body); return
        if self.path in ("/lidar.mjpg","/stream.mjpg"): stream(self,make_lidar_image); return
        if self.path=="/camera.mjpg": stream(self,lambda: state["rgb"] or placeholder("Waiting for OAK-D Lite RGB...")); return
        if self.path=="/depth.mjpg": stream(self,lambda: state["depth"] or placeholder("Waiting for OAK-D Lite depth...")); return
        if self.path=="/status":
            with state["lock"]: body=state["trial"].encode()
            self.send_response(200); self.send_header("Content-Type","text/plain"); self.send_header("Content-Length",str(len(body))); self.end_headers(); self.wfile.write(body); return
        if self.path=="/events":
            with state["lock"]: body=json.dumps(list(state["events"])).encode()
            self.send_response(200); self.send_header("Content-Type","application/json"); self.send_header("Content-Length",str(len(body))); self.end_headers(); self.wfile.write(body); return
        self.send_error(404)
    def do_POST(self):
        if self.path != "/trial": self.send_error(404); return
        with state["lock"]:
            active=state.get("active",False)
        if active:
            body=b"controller already active"
        else:
            threading.Thread(target=run_trial,daemon=True).start(); body=f"{TRIAL_S:.0f}-second shared-sensor trial started".encode()
        self.send_response(202); self.send_header("Content-Type","text/plain"); self.send_header("Content-Length",str(len(body))); self.end_headers(); self.wfile.write(body)
    def log_message(self,*_): pass

def main():
    threading.Thread(target=read_lidar,daemon=True).start(); threading.Thread(target=read_oak,daemon=True).start()
    print(f"Open http://localhost:{HTTP_PORT}"); ThreadingHTTPServer(("0.0.0.0",HTTP_PORT),Handler).serve_forever()
if __name__=="__main__": main()
