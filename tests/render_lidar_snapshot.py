#!/usr/bin/env python3
"""Capture 10 seconds of COIN-D6 data and render a close-range occupancy PNG."""
import math
import statistics
import time

from PIL import Image, ImageDraw
import serial

PORT = "/dev/ttyTHS1"
BAUD = 230400
OUT = "lidar_close_objects_moved_2.png"
SIZE = 1000
METRES_ACROSS = 2.0


def packets_for(seconds=10.0):
    buf = bytearray()
    samples = []
    with serial.Serial(PORT, BAUD, timeout=0.1) as uart:
        uart.reset_input_buffer()
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            buf.extend(uart.read(4096))
            while True:
                i = buf.find(b"\xaa\x55\x00\x19")
                if i < 0:
                    buf[:] = buf[-3:]
                    break
                if len(buf) < i + 85:
                    del buf[:i]
                    break
                packet = bytes(buf[i:i + 85])
                del buf[:i + 85]
                start = (int.from_bytes(packet[4:6], "little") >> 1) / 64.0
                end = (int.from_bytes(packet[6:8], "little") >> 1) / 64.0
                if end < start:
                    end += 360.0
                for n in range(25):
                    offset = 10 + n * 3
                    intensity = packet[offset]
                    distance_m = int.from_bytes(packet[offset + 1:offset + 3], "little") / 1000.0
                    if 0.05 <= distance_m <= 12.0:
                        angle = (start + (end - start) * n / 24.0) % 360.0
                        samples.append((angle, distance_m, intensity))
    return samples


def main():
    samples = packets_for()
    # Median per 0.5-degree bin rejects momentary outliers while preserving objects.
    bins = [[] for _ in range(720)]
    for angle, distance, _ in samples:
        bins[int(angle * 2) % 720].append(distance)
    points = []
    for bin_no, distances in enumerate(bins):
        near = [d for d in distances if d <= METRES_ACROSS / 2]
        if near:
            points.append((math.radians(bin_no / 2), statistics.median(near)))

    im = Image.new("RGB", (SIZE, SIZE), "#101820")
    draw = ImageDraw.Draw(im)
    centre = SIZE // 2
    scale = SIZE / METRES_ACROSS
    for radius_m in (0.25, 0.5, 0.75, 1.0):
        r = radius_m * scale
        draw.ellipse((centre-r, centre-r, centre+r, centre+r), outline="#2e4352", width=1)
        draw.text((centre + 5, centre-r + 4), f"{int(radius_m * 100)} cm", fill="#87a1b1")
    draw.line((centre - 18, centre, centre + 18, centre), fill="white", width=2)
    draw.line((centre, centre - 18, centre, centre + 18), fill="white", width=2)
    draw.text((18, 18), "Sassy D6: close objects (within 1 m)", fill="white")
    draw.text((18, 44), "Robot/lidar is at centre; top is lidar forward (arbitrary until mounted).", fill="#b6cbd8")

    # Draw connected local surfaces where adjacent angular bins agree in range.
    xy = []
    for a, d in points:
        xy.append((centre + d * scale * math.cos(a), centre - d * scale * math.sin(a), d))
    for left, right in zip(xy, xy[1:]):
        if abs(left[2] - right[2]) < 0.12:
            draw.line((left[0], left[1], right[0], right[1]), fill="#48d7bd", width=3)
    for x, y, _ in xy:
        draw.ellipse((x-3, y-3, x+3, y+3), fill="#f4c95d")
    im.save(OUT)
    print(f"{OUT}: {len(samples)} valid samples, {len(points)} close-range bins")


if __name__ == "__main__":
    main()
