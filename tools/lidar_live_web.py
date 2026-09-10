#!/usr/bin/env python3
"""Small dependency-light live D6 lidar web viewer."""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import math
import statistics
import threading

from PIL import Image, ImageDraw
import serial

PORT = "/dev/ttyTHS1"
BAUD = 230400
HTTP_PORT = 8080
SIZE = 800
RANGE_M = 4.0

state = {"scans": deque(maxlen=3), "lock": threading.Lock()}


def read_lidar():
    buf = bytearray(); scan = []; previous = None
    with serial.Serial(PORT, BAUD, timeout=.1) as uart:
        while True:
            buf.extend(uart.read(4096))
            while True:
                i = buf.find(b"\xaa\x55\x00\x19")
                if i < 0: buf[:] = buf[-3:]; break
                if len(buf) < i + 85: del buf[:i]; break
                p = bytes(buf[i:i+85]); del buf[:i+85]
                start = (int.from_bytes(p[4:6], 'little') >> 1) / 64
                end = (int.from_bytes(p[6:8], 'little') >> 1) / 64
                if end < start: end += 360
                if previous is not None and start + 5 < previous:
                    if scan:
                        with state['lock']: state['scans'].append(scan)
                    scan = []
                previous = start
                for n in range(25):
                    o = 10 + n * 3
                    distance = int.from_bytes(p[o+1:o+3], 'little') / 1000
                    if .05 <= distance <= RANGE_M:
                        scan.append(((start + (end-start)*n/24) % 360, distance))


def make_image():
    with state['lock']: scans = list(state['scans'])
    bins = [[] for _ in range(720)]
    for scan in scans:
        for angle, distance in scan: bins[int(angle*2) % 720].append(distance)
    points = [(math.radians(i/2), statistics.median(v)) for i,v in enumerate(bins) if v]
    image = Image.new('RGB', (SIZE, SIZE), '#101820'); draw = ImageDraw.Draw(image)
    c = SIZE // 2; scale = SIZE / (2 * RANGE_M)
    for m in (1, 2, 3, 4):
        r=m*scale; draw.ellipse((c-r,c-r,c+r,c+r), outline='#2e4352')
        draw.text((c+5,c-r+4), f'{m} m', fill='#87a1b1')
    draw.line((c-15,c,c+15,c), fill='white', width=2); draw.line((c,c-15,c,c+15), fill='white', width=2)
    draw.text((15,15), 'Sassy / WitMotion D6 live lidar', fill='white')
    draw.text((15,40), 'Top = lidar-forward reference; centre = sensor', fill='#b6cbd8')
    # Mirror left/right while preserving north/south orientation.
    xy=[(c-d*scale*math.cos(a),c-d*scale*math.sin(a),d) for a,d in points]
    for a,b in zip(xy,xy[1:]):
        if abs(a[2]-b[2]) < .12: draw.line((a[0],a[1],b[0],b[1]),fill='#48d7bd',width=3)
    for x,y,_ in xy: draw.ellipse((x-3,y-3,x+3,y+3),fill='#f4c95d')
    out=io.BytesIO(); image.save(out,format='JPEG',quality=88); return out.getvalue()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            body=b'''<!doctype html><title>Sassy lidar</title><style>body{background:#101820;color:#ddd;font:16px sans-serif;text-align:center}img{max-width:95vw;height:auto}</style><h1>Sassy live lidar</h1><img src="/stream.mjpg">'''
            self.send_response(200); self.send_header('Content-Type','text/html'); self.send_header('Content-Length',str(len(body))); self.end_headers(); self.wfile.write(body); return
        if self.path != '/stream.mjpg': self.send_error(404); return
        self.send_response(200); self.send_header('Cache-Control','no-cache'); self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame'); self.end_headers()
        try:
            while True:
                jpg=make_image(); self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpg)).encode()+b'\r\n\r\n'+jpg+b'\r\n')
                self.wfile.flush()
                import time; time.sleep(.1)
        except (BrokenPipeError, ConnectionResetError): pass
    def log_message(self, *_): pass


def main():
    threading.Thread(target=read_lidar, daemon=True).start()
    print(f'Open http://localhost:{HTTP_PORT}')
    ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler).serve_forever()

if __name__ == '__main__': main()
