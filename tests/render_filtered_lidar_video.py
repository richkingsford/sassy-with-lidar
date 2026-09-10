#!/usr/bin/env python3
"""Render a 10 s COIN-D6 map video from filtered full 360-degree scans.

The LiDAR is read passively.  The decoder matches render_lidar_snapshot.py:
each sample is intensity, distance-low, distance-high.
"""
from collections import deque
import math
import statistics
import subprocess
import time

from PIL import Image, ImageDraw
import imageio_ffmpeg
import serial

PORT = "/dev/ttyTHS1"
BAUD = 230400
OUT = "lidar_filtered_10s.mp4"
SIZE = 1000
METRES_ACROSS = 2.0
FPS = 10
DURATION_S = 10
SMOOTHING_SCANS = 3


def read_scans(seconds: float):
    """Yield scans; each is a list of (angle_degrees, distance_m)."""
    buffer = bytearray()
    scan = []
    previous_start = None
    until = time.monotonic() + seconds
    with serial.Serial(PORT, BAUD, timeout=0.1) as uart:
        uart.reset_input_buffer()
        while time.monotonic() < until:
            buffer.extend(uart.read(4096))
            while True:
                found = buffer.find(b"\xaa\x55\x00\x19")
                if found < 0:
                    buffer[:] = buffer[-3:]
                    break
                if len(buffer) < found + 85:
                    del buffer[:found]
                    break
                packet = bytes(buffer[found:found + 85])
                del buffer[:found + 85]

                start = (int.from_bytes(packet[4:6], "little") >> 1) / 64.0
                end = (int.from_bytes(packet[6:8], "little") >> 1) / 64.0
                if end < start:
                    end += 360.0
                # A decreasing packet start marks the next revolution.
                if previous_start is not None and start + 5.0 < previous_start:
                    if scan:
                        yield scan
                    scan = []
                previous_start = start
                for index in range(25):
                    offset = 10 + index * 3
                    distance = int.from_bytes(packet[offset + 1:offset + 3], "little") / 1000.0
                    if 0.05 <= distance <= 12.0:
                        angle = (start + (end - start) * index / 24.0) % 360.0
                        scan.append((angle, distance))


def filtered_points(scans):
    bins = [[] for _ in range(720)]
    for scan in scans:
        for angle, distance in scan:
            if distance <= METRES_ACROSS / 2:
                bins[int(angle * 2) % 720].append(distance)
    return [(math.radians(index / 2), statistics.median(values))
            for index, values in enumerate(bins) if values]


def draw_frame(points):
    image = Image.new("RGB", (SIZE, SIZE), "#101820")
    draw = ImageDraw.Draw(image)
    centre = SIZE // 2
    scale = SIZE / METRES_ACROSS
    for metres in (0.25, 0.5, 0.75, 1.0):
        radius = metres * scale
        draw.ellipse((centre-radius, centre-radius, centre+radius, centre+radius), outline="#2e4352")
        draw.text((centre + 6, centre-radius + 4), f"{int(metres * 100)} cm", fill="#87a1b1")
    draw.line((centre-18, centre, centre+18, centre), fill="white", width=2)
    draw.line((centre, centre-18, centre, centre+18), fill="white", width=2)
    draw.text((18, 18), "Sassy D6: filtered 360-degree scan", fill="white")
    draw.text((18, 44), "Centre = robot/lidar; top = lidar-forward reference.", fill="#b6cbd8")

    xy = [(centre + distance * scale * math.cos(angle),
           centre - distance * scale * math.sin(angle), distance)
          for angle, distance in points]
    for left, right in zip(xy, xy[1:]):
        if abs(left[2] - right[2]) < 0.12:
            draw.line((left[0], left[1], right[0], right[1]), fill="#48d7bd", width=3)
    for x, y, _ in xy:
        draw.ellipse((x-3, y-3, x+3, y+3), fill="#f4c95d")
    return image


def main():
    target_frames = FPS * DURATION_S
    scans = list(read_scans(DURATION_S + 1.0))
    if not scans:
        raise RuntimeError("No complete D6 scans received")

    encoder = subprocess.Popen(
        [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{SIZE}x{SIZE}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
         "-pix_fmt", "yuv420p", OUT], stdin=subprocess.PIPE)
    history = deque(maxlen=SMOOTHING_SCANS)
    for frame_index in range(target_frames):
        scan_index = min(len(scans) - 1, int(frame_index * len(scans) / target_frames))
        history.append(scans[scan_index])
        encoder.stdin.write(draw_frame(filtered_points(history)).tobytes())
    encoder.stdin.close()
    encoder.wait()
    print(f"{OUT}: {len(scans)} full scans, {target_frames} frames")


if __name__ == "__main__":
    main()
