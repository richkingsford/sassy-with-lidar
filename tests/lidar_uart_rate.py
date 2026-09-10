#!/usr/bin/env python3
"""Passive COIN-D6/WitMotion D6 UART byte-rate monitor."""
import argparse
import time

import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="Report incoming LiDAR UART bytes per second")
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    print(f"Listening on {args.port} at {args.baud} baud for {args.seconds:g} seconds")
    print("Passive read only; no bytes will be transmitted.")

    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    ) as uart:
        uart.reset_input_buffer()
        started = time.monotonic()
        next_report = started + 1.0
        total = 0

        while True:
            now = time.monotonic()
            if now - started >= args.seconds:
                break
            total += len(uart.read(4096))
            now = time.monotonic()
            if now >= next_report:
                elapsed = now - started
                print(f"{elapsed:6.1f}s: {total:8d} total bytes, {total / elapsed:8.1f} B/s")
                next_report += 1.0

        elapsed = time.monotonic() - started
        print(f"Result: {total} bytes in {elapsed:.2f}s ({total / elapsed:.1f} B/s)")


if __name__ == "__main__":
    main()
