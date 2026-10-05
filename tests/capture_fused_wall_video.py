#!/usr/bin/env python3
"""Record a 10-second LiDAR + OAK-D Lite wall-observation video; never moves motors."""
from collections import deque
import argparse, csv, io, math, statistics, subprocess, threading, time
from pathlib import Path

import depthai as dai
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw
import serial

LIDAR_PORT="/dev/ttyTHS1"; LIDAR_BAUD=230400; FORWARD_DEG=180.0
DURATION_S=10; FPS=10; MIN_M=.15; MAX_M=4.0; BACKUP_DISTANCE_M=.15
OUT=Path("artifacts/fused_wall_capture_10s.mp4")
LOG=Path("artifacts/fused_wall_capture_10s.csv")

def angle_error(angle, reference): return (angle-reference+180)%360-180

class LidarReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True); self.scan=[]; self.lock=threading.Lock(); self.stop_event=threading.Event()
    def run(self):
        buffer=bytearray(); scan=[]; previous=None
        with serial.Serial(LIDAR_PORT,LIDAR_BAUD,timeout=.05) as uart:
            while not self.stop_event.is_set():
                buffer.extend(uart.read(4096))
                while True:
                    index=buffer.find(b"\xaa\x55\x00\x19")
                    if index<0: buffer[:]=buffer[-3:]; break
                    if len(buffer)<index+85: del buffer[:index]; break
                    packet=bytes(buffer[index:index+85]); del buffer[:index+85]
                    start=(int.from_bytes(packet[4:6],"little")>>1)/64; end=(int.from_bytes(packet[6:8],"little")>>1)/64
                    if end<start: end+=360
                    if previous is not None and start+5<previous:
                        if scan:
                            with self.lock: self.scan=scan
                        scan=[]
                    previous=start
                    for n in range(25):
                        offset=10+n*3; distance=int.from_bytes(packet[offset+1:offset+3],"little")/1000
                        if MIN_M<=distance<=MAX_M: scan.append(((start+(end-start)*n/24)%360,distance))
    def snapshot(self):
        with self.lock: return list(self.scan)

def wall_from_lidar(scan):
    ordered=sorted(scan); groups=[]; group=[]
    for point in ordered:
        if group and (point[0]-group[-1][0]>3 or abs(point[1]-group[-1][1])>.25): groups.append(group); group=[]
        group.append(point)
    if group: groups.append(group)
    clusters=[]
    for group in groups:
        if len(group)>=3:
            clusters.append((statistics.mean(a for a,_ in group),statistics.median(d for _,d in group),len(group)))
    return min(clusters,key=lambda item:item[1]) if clusters else None

def oak_wall(depth):
    if depth is None: return None
    height,width=depth.shape; region=depth[height//4:height*9//10]
    candidates=[]
    for side,start,end,approx_deg in (("left",0,width//3,-25),("center",width//3,2*width//3,0),("right",2*width//3,width,25)):
        values=region[:,start:end]; valid=values[(values>=int(MIN_M*1000))&(values<=int(MAX_M*1000))]
        if valid.size>=40: candidates.append((side,float(np.percentile(valid,20))/1000,approx_deg))
    return min(candidates,key=lambda item:item[1]) if candidates else None

def recommended_action(bearing, distance):
    """Recommend a motion from measured wall geometry, not a room-specific rule."""
    if bearing is None or distance is None:
        return "FORWARD", "no front-wall measurement"
    if distance <= BACKUP_DISTANCE_M:
        return "BACKWARDS", "wall is within 150 mm; back up before turning"
    if abs(bearing) >= 90:
        return "FORWARD", "wall is behind or outside the forward half-plane"
    if bearing <= 0:
        return "TURN RIGHT", "wall is forward-left; turn away"
    return "TURN LEFT", "wall is forward-right; turn away"

def depth_image(depth):
    if depth is None: return Image.new("RGB",(640,400),"#101820")
    valid=depth>0; normalized=np.clip((depth.astype(float)-MIN_M*1000)/(MAX_M*1000-MIN_M*1000),0,1)
    image=np.empty((*depth.shape,3),dtype=np.uint8)
    image[...,0]=(255*np.clip(1.5-abs(4*normalized-3),0,1)).astype(np.uint8)
    image[...,1]=(255*np.clip(1.5-abs(4*normalized-2),0,1)).astype(np.uint8)
    image[...,2]=(255*np.clip(1.5-abs(4*normalized-1),0,1)).astype(np.uint8)
    image[~valid]=(16,24,32); return Image.fromarray(image).resize((640,360))

def lidar_image(scan, lidar_hit):
    size=640; center=size//2; scale=size/(2*MAX_M); image=Image.new("RGB",(size,size),"#101820"); draw=ImageDraw.Draw(image)
    for meters in (1,2,3,4):
        radius=meters*scale; draw.ellipse((center-radius,center-radius,center+radius,center+radius),outline="#2e4352")
        draw.text((center+5,center-radius+4),f"{meters}m",fill="#87a1b1")
    draw.line((center-15,center,center+15,center),fill="white",width=2); draw.line((center,center-15,center,center+15),fill="white",width=2)
    for angle,distance in scan:
        relative=math.radians(angle_error(angle,FORWARD_DEG)); x=center+distance*math.sin(relative)*scale; y=center-distance*math.cos(relative)*scale
        draw.ellipse((x-2,y-2,x+2,y+2),fill="#f4c95d")
    draw.text((14,14),"WitMotion D6 LiDAR",fill="white"); draw.text((14,36),"top = robot forward",fill="#b6cbd8")
    if lidar_hit:
        angle,distance,_=lidar_hit; relative=math.radians(angle_error(angle,FORWARD_DEG)); x=center+distance*math.sin(relative)*scale; y=center-distance*math.cos(relative)*scale
        draw.ellipse((x-8,y-8,x+8,y+8),outline="#ff4b4b",width=3)
    return image

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--run",action="store_true"); args=parser.parse_args()
    if not args.run:
        print("Ready only. Say go, then run with --run to capture 10 seconds."); return
    OUT.parent.mkdir(parents=True,exist_ok=True)
    lidar=LidarReader(); lidar.start()
    pipeline=dai.Pipeline(); rgb=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    left=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B); right=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    rgb_out=rgb.requestOutput((640,400),fps=15); left_out=left.requestOutput((640,400),type=dai.ImgFrame.Type.GRAY8,fps=15); right_out=right.requestOutput((640,400),type=dai.ImgFrame.Type.GRAY8,fps=15)
    stereo=pipeline.create(dai.node.StereoDepth); stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS); left_out.link(stereo.left); right_out.link(stereo.right)
    rgb_q=rgb_out.createOutputQueue(maxSize=1,blocking=False); depth_q=stereo.depth.createOutputQueue(maxSize=1,blocking=False); pipeline.start()
    encoder=subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),"-y","-f","rawvideo","-pix_fmt","rgb24","-s","1280x720","-r",str(FPS),"-i","-","-c:v","libx264","-pix_fmt","yuv420p",str(OUT)],stdin=subprocess.PIPE)
    latest_rgb=None; latest_depth=None
    try:
        deadline=time.monotonic()+5
        while (not lidar.snapshot() or latest_depth is None) and time.monotonic()<deadline:
            message=rgb_q.tryGet(); latest_rgb=message.getCvFrame() if message else latest_rgb
            message=depth_q.tryGet(); latest_depth=message.getFrame() if message else latest_depth
            time.sleep(.01)
        if not lidar.snapshot() or latest_depth is None: raise RuntimeError("LiDAR or OAK depth not ready")
        print("capturing now: move Sassy for 10 seconds")
        started=time.monotonic()
        with LOG.open("w",newline="") as log_file:
            writer=csv.writer(log_file); writer.writerow(("seconds","lidar_bearing_deg","lidar_distance_m","oak_sector","oak_distance_m"))
            for frame_number in range(DURATION_S*FPS):
                target=started+frame_number/FPS
                while time.monotonic()<target: time.sleep(.001)
                message=rgb_q.tryGet(); latest_rgb=message.getCvFrame() if message else latest_rgb
                message=depth_q.tryGet(); latest_depth=message.getFrame() if message else latest_depth
                hit=wall_from_lidar(lidar.snapshot()); oak=oak_wall(latest_depth)
                bearing=None if hit is None else angle_error(hit[0],FORWARD_DEG); distance=None if hit is None else hit[1]
                writer.writerow((f"{time.monotonic()-started:.2f}","" if bearing is None else f"{bearing:.1f}","" if distance is None else f"{distance:.3f}","" if oak is None else oak[0],"" if oak is None else f"{oak[1]:.3f}"))
                canvas=Image.new("RGB",(1280,720),"#101820")
                rgb_image=Image.fromarray(latest_rgb[:,:,::-1]).resize((640,360)) if latest_rgb is not None else Image.new("RGB",(640,360),"#101820")
                canvas.paste(rgb_image,(640,0)); canvas.paste(depth_image(latest_depth),(640,360)); draw=ImageDraw.Draw(canvas)
                action, reason=recommended_action(bearing,distance)
                # Left column: lidar map above, its real-time decision/log table below.
                lidar_panel=lidar_image(lidar.snapshot(),hit).resize((640,360))
                canvas.paste(lidar_panel,(0,0))
                draw.rectangle((0,360,640,720),fill="#17242d")
                draw.text((18,380),"WALL GEOMETRY / RECOMMENDED ACTION",fill="white")
                degree_text="no LiDAR wall cluster" if bearing is None else f"{bearing:+.1f} deg relative to robot-forward"
                distance_text="n/a" if distance is None else f"{distance*1000:.0f} mm ({distance:.2f} m)"
                oak_text="no near depth cluster" if oak is None else f"{oak[0]} sector, {oak[1]*1000:.0f} mm"
                rows=(("Wall bearing",degree_text),("Wall distance",distance_text),("OAK confirmation",oak_text),("Action",action),("Why",reason))
                y=420
                for label,value in rows:
                    draw.text((24,y),label+":",fill="#87a1b1"); draw.text((185,y),value,fill="#ff7676" if label=="Action" else "#f4c95d"); y+=52
                draw.text((652,12),"OAK-D Lite RGB",fill="white"); draw.text((652,372),"OAK-D Lite stereo depth",fill="white")
                encoder.stdin.write(canvas.tobytes())
    finally:
        if encoder.stdin: encoder.stdin.close()
        encoder.wait(); lidar.stop_event.set(); lidar.join(2)
        try: pipeline.stop()
        except Exception: pass
    print(f"wrote {OUT} and {LOG}")

if __name__=="__main__": main()
