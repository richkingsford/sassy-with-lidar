#!/usr/bin/env python3
"""Guarded forward-and-turn-left lidar trial for Sassy."""
import argparse
import threading
import time

import serial

LIDAR = '/dev/ttyTHS1'; UNO = '/dev/leia-uno'
LIDAR_BAUD = 230400; UNO_BAUD = 115200
TRIAL_S = 10.0; UNO_ARM_S = 1.5; INITIAL_CRAWL_S = 0.5
SPEED = 128; FRONT_DEG = 90.0; HIT_M = 1.0; ZONE_WIDTH_M = 0.78
MIN_CLUSTER_POINTS = 3; MIN_TURN_S = 0.8; COMMAND_PERIOD_S = 0.08

def angle_error(a, reference): return (a-reference+180.0) % 360.0 - 180.0

def clusters(scan):
    ordered = sorted((a,d) for a,d in scan if .05 <= d <= HIT_M)
    groups=[]; group=[]
    for item in ordered:
        if group and (item[0]-group[-1][0] > 3.0 or abs(item[1]-group[-1][1]) > .20):
            groups.append(group); group=[]
        group.append(item)
    if group: groups.append(group)
    return [(sum(a for a,_ in g)/len(g), sum(d for _,d in g)/len(g), len(g)) for g in groups if len(g)>=MIN_CLUSTER_POINTS]

def in_obstruction_zone(angle, distance):
    """Match the 1m x 78cm red box shown by the live lidar website."""
    radians = angle * 3.141592653589793 / 180.0
    forward = distance * __import__('math').sin(radians)
    lateral = -distance * __import__('math').cos(radians)
    return 0.0 <= forward <= HIT_M and abs(lateral) <= ZONE_WIDTH_M / 2

def front_cluster(scan):
    candidates=[c for c in clusters(scan) if in_obstruction_zone(c[0], c[1])]
    return min(candidates,key=lambda c:c[1]) if candidates else None

def tracked_at_side(scan, initial, turn_left):
    # A left turn moves the stationary obstacle to robot-right; right is inverse.
    target=(initial[0] + (-90.0 if turn_left else 90.0)) % 360.0
    return any(abs(angle_error(a,target)) <= 20 and abs(d-initial[1]) <= .5 for a,d,_ in clusters(scan))

class ScanReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True); self.latest=[]; self.lock=threading.Lock(); self.stop=threading.Event()
    def run(self):
        buf=bytearray(); scan=[]; previous=None
        with serial.Serial(LIDAR,LIDAR_BAUD,timeout=.05) as uart:
            while not self.stop.is_set():
                buf.extend(uart.read(4096))
                while True:
                    i=buf.find(b'\xaa\x55\x00\x19')
                    if i<0: buf[:]=buf[-3:]; break
                    if len(buf)<i+85: del buf[:i]; break
                    p=bytes(buf[i:i+85]); del buf[:i+85]
                    start=(int.from_bytes(p[4:6],'little')>>1)/64.0; end=(int.from_bytes(p[6:8],'little')>>1)/64.0
                    if end<start: end+=360.0
                    if previous is not None and start+5<previous:
                        if scan:
                            with self.lock: self.latest=scan
                        scan=[]
                    previous=start
                    for n in range(25):
                        o=10+3*n; d=int.from_bytes(p[o+1:o+3],'little')/1000.0
                        if .05<=d<=12: scan.append(((start+(end-start)*n/24)%360,d))
    def scan(self):
        with self.lock: return list(self.latest)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--run',action='store_true'); args=parser.parse_args()
    if not args.run:
        print('Dry run only. Add --run only in a clear, supervised area with an emergency stop ready.'); return
    reader=ScanReader(); reader.start()
    with serial.Serial(UNO,UNO_BAUD,timeout=.1) as uno:
        try:
            uno.write(b'S\n'); uno.flush(); print(f'arming Uno for {UNO_ARM_S:.1f}s'); time.sleep(UNO_ARM_S)
            scan_deadline=time.monotonic()+5.0
            while not reader.scan() and time.monotonic()<scan_deadline:
                time.sleep(.05)
            if not reader.scan():
                raise RuntimeError('No complete lidar scan received within 5 seconds')
            print('lidar scan ready; starting 10-second trial clock')
            started=time.monotonic(); phase='crawl'; obstacle=None; turning_started=None; turn_left=None
            while time.monotonic()-started<TRIAL_S:
                now=time.monotonic(); elapsed=now-started; scan=reader.scan()
                if phase=='crawl':
                    command=f'D,{SPEED},{SPEED}'
                    if elapsed>=INITIAL_CRAWL_S: phase='approach'; print('initial crawl complete; monitoring front obstacle')
                elif phase=='approach':
                    command=f'D,{SPEED},{SPEED}'; candidate=front_cluster(scan)
                    if candidate:
                        obstacle=candidate; phase='turn'; turning_started=now
                        # A cluster on robot-right (bearing < forward) needs a left turn, and vice versa.
                        turn_left = angle_error(candidate[0], FRONT_DEG) <= 0
                        command=f'D,0,{SPEED}' if turn_left else f'D,{SPEED},0'
                        print(f'front cluster at {candidate[0]:.1f}°, {candidate[1]:.2f}m; turning {"left" if turn_left else "right"}')
                else:
                    if not front_cluster(scan):
                        phase='approach'; command=f'D,{SPEED},{SPEED}'
                        print('obstruction zone clear; resuming forward crawl')
                    else:
                        command=f'D,0,{SPEED}' if turn_left else f'D,{SPEED},0'
                uno.write((command+'\n').encode()); uno.flush(); time.sleep(COMMAND_PERIOD_S)
            else: print('10-second limit reached; stopped')
        finally:
            uno.write(b'S\n'); uno.flush(); reader.stop.set(); reader.join(timeout=.5); print('trial complete; stop sent')

if __name__=='__main__': main()
