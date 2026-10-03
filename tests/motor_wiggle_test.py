#!/usr/bin/env python3
"""One-second full-power forward/backward check for Sassy's Uno."""
import argparse
import time

import serial


def pulse(uno, command, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        uno.write(command)
        uno.flush()
        time.sleep(0.08)  # The Uno stops after 250 ms without a command.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/leia-uno")
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--power", type=int, default=255)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if not 0 < args.seconds <= 3 or not 1 <= args.power <= 255:
        parser.error("seconds must be (0, 3] and power must be 1..255")
    if not args.run:
        print("Ready: forward, backward, then stop. Add --run to move the motors.")
        return

    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=0.2) as uno:
        try:
            uno.write(b"S\n")
            uno.flush()
            time.sleep(1.5)  # Opening serial resets the Uno.
            for direction, power in (("forward", args.power), ("backward", -args.power)):
                print(f"{direction}: {args.seconds:.1f}s, PWM {power}", flush=True)
                pulse(uno, f"D,{power},{power}\n".encode(), args.seconds)
        finally:
            uno.write(b"S\n")
            uno.flush()
            print("STOP sent", flush=True)


if __name__ == "__main__":
    main()
