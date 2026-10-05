#!/usr/bin/env python3
"""Record one spoken command, transcribe it locally, and optionally execute it."""
import argparse
import json
from pathlib import Path
from urllib import request

from record_and_transcribe import (
    DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL, record, transcribe_whisper,
)
from voice_commands import parse_voice_command


def main():
    parser = argparse.ArgumentParser(description="Listen for one Sassy command.")
    parser.add_argument("--seconds", type=int, default=10, help="Listening window (default: 10).")
    parser.add_argument("--execute", action="store_true", help="Send a recognized plan to the guarded dashboard controller.")
    parser.add_argument("--url", default="http://127.0.0.1:8080/voice", help="Guarded dashboard voice endpoint.")
    args = parser.parse_args()
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    output = Path("artifacts") / "voice_command.wav"
    record(output, args.seconds, "hw:0,0")
    transcript = transcribe_whisper(output, DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL)
    print(f"TRANSCRIPT: {transcript or '[no speech recognized]'}")
    try:
        plan = parse_voice_command(transcript)
    except ValueError as exc:
        print(f"NO ACTION: {exc}")
        return
    print("PLAN:")
    for step in plan:
        print(f"- {step['label']}  [D,{step['left']},{step['right']}] for {step['seconds']:.1f}s")
    if not args.execute:
        print("Preview only. Re-run with --execute after checking the plan.")
        return
    payload = json.dumps({"transcript": transcript}).encode()
    response = request.urlopen(request.Request(args.url, data=payload, headers={"Content-Type": "application/json"}), timeout=5)
    print(response.read().decode())


if __name__ == "__main__":
    main()
