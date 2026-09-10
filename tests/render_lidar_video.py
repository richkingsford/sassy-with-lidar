#!/usr/bin/env python3
import math, subprocess, time
from PIL import Image, ImageDraw
import serial
import imageio_ffmpeg

OUT = 'lidar_walls_objects_10s_test2.mp4'
W = H = 720
SCALE = 45.0  # pixels per metre

def frame(points):
    im = Image.new('RGB', (W, H), '#101820'); d = ImageDraw.Draw(im)
    cx = cy = W // 2
    for r in range(1, 7):
        q = int(r * SCALE); d.ellipse((cx-q, cy-q, cx+q, cy+q), outline='#263746')
        d.text((cx+4, cy-q+2), f'{r}m', fill='#6b8494')
    d.line((cx-12,cy,cx+12,cy), fill='#efefef'); d.line((cx,cy-12,cx,cy+12), fill='#efefef')
    for a, dist in points:
        rr = dist * SCALE
        x = int(cx + rr * math.cos(a)); y = int(cy - rr * math.sin(a))
        if 0 <= x < W and 0 <= y < H: d.ellipse((x-2,y-2,x+2,y+2), fill='#55d6be')
    d.text((16, 16), 'Sassy / WitMotion D6 live 2D scan', fill='white')
    return im

def main():
    exe = imageio_ffmpeg.get_ffmpeg_exe()
    proc = subprocess.Popen([exe, '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', '10', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', OUT], stdin=subprocess.PIPE)
    buf = bytearray(); batches=[]; end=time.time()+10
    with serial.Serial('/dev/ttyTHS1', 230400, timeout=.1) as s:
        while time.time() < end:
            buf.extend(s.read(4096))
            while True:
                i=buf.find(b'\xaa\x55\x00\x19')
                if i < 0: buf[:] = buf[-3:]; break
                if len(buf) < i+85: del buf[:i]; break
                p=bytes(buf[i:i+85]); del buf[:i+85]
                # D6/COIN packet angles are Q6 values with bit 0 reserved
                # as the sample/check flag; discard it before scaling.
                start=(int.from_bytes(p[4:6],'little') >> 1)/64.0
                stop=(int.from_bytes(p[6:8],'little') >> 1)/64.0
                if stop < start: stop += 360
                batches.append((start, stop, p[10:85]))
    points=[]
    for n in range(100):
        current=[]
        for start, stop, raw in batches[max(0, int(len(batches)*n/100)):int(len(batches)*(n+1)/100)]:
            for j in range(25):
                dist=int.from_bytes(raw[3*j:3*j+2],'little')/1000.0
                if .05 <= dist <= 12: current.append((math.radians(start+(stop-start)*j/24),dist))
        points = current
        proc.stdin.write(frame(points).tobytes())
    proc.stdin.flush()
    proc.stdin.close(); proc.wait(); print(OUT)
if __name__ == '__main__': main()
