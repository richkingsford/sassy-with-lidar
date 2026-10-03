#!/usr/bin/env python3
"""Supervised four-direction, one-wheel pivot diagnostic for Sassy."""
import time

import serial

UNO = "/dev/leia-uno"
BAUD = 115200
TURN_SECONDS = 1.0
STOP_GAP_SECONDS = 1.0
FULL_POWER = 255

# Labels follow the observed navigation turn and the Uno's D,left,right sketch.
STEPS = (
    ("physical left wheel forward only", b"D,255,0\n"),
    ("physical right wheel backward only", b"D,0,-255\n"),
    ("physical right wheel forward only", b"D,0,255\n"),
    ("physical left wheel backward only", b"D,-255,0\n"),
)


def drive_for(uno, command, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        uno.write(command)
        uno.flush()
        time.sleep(.08)  # Keep the Uno's 250 ms watchdog fed.


def main():
    with serial.Serial(UNO, BAUD, timeout=.2) as uno:
        uno.write(b"S\n"); uno.flush(); time.sleep(1.5)
        for label, command in STEPS:
            print(f"{label}: {TURN_SECONDS:.1f}s at PWM {FULL_POWER}")
            drive_for(uno, command, TURN_SECONDS)
            uno.write(b"S\n"); uno.flush(); time.sleep(STOP_GAP_SECONDS)
    print("turn test complete; STOP sent")


if __name__ == "__main__":
    main()
