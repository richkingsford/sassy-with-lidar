#!/usr/bin/env python3
"""Guarded 0.5 s motor-direction test for the Sassy Uno controller.

Run with --run only after lifting the wheels or clearing a safe test area.
"""
import argparse
import time

import serial


def pulse(ser, frame, seconds=0.5):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ser.write((frame + "\n").encode())
        time.sleep(0.08)
    ser.write(b"S\n")
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--run", action="store_true", help="actually move the motors")
    args = parser.parse_args()
    if not args.run:
        print("Dry run only. Add --run after lifting the wheels or clearing the area.")
        return
    with serial.Serial(args.port, 115200, timeout=0.1) as ser:
        time.sleep(0.2)
        for name, frame in (
            ("forward", "C,150,0"),
            ("backward", "C,-150,0"),
            ("sharp right", "D,150,0"),
            ("sharp left", "D,0,150"),
        ):
            print(f"{name}: 0.5 s")
            pulse(ser, frame)
        ser.write(b"S\n")


if __name__ == "__main__":
    main()
