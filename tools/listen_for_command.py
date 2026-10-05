#!/usr/bin/env python3
"""Record one spoken command, transcribe it locally, and optionally execute it."""
import argparse
import json
from datetime import datetime
from pathlib import Path
from urllib import error, request

from record_and_transcribe import (
    DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL, record, transcribe_whisper,
)
from voice_commands import WAKE_PHRASE, command_after_wake_phrase, parse_voice_command
from sassy_speech import choose_response, speak


def main():
    parser = argparse.ArgumentParser(description="Listen for one Sassy command.")
    parser.add_argument("--seconds", type=int, default=10, help="Listening window (default: 10).")
    parser.add_argument("--execute", action="store_true", help="Send a recognized plan to the guarded dashboard controller.")
    parser.add_argument("--url", default="http://127.0.0.1:8080/voice", help="Guarded dashboard voice endpoint.")
    args = parser.parse_args()
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    output = Path("artifacts") / f"voice_command_{datetime.now():%Y%m%d_%H%M%S}.wav"
    record(output, args.seconds, "hw:0,0")
    transcript = transcribe_whisper(output, DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL)
    transcript_path = output.with_suffix(".heard.txt")
    transcript_path.write_text(transcript + "\n", encoding="utf-8")
    print(f"FULL TRANSCRIPT: {transcript or '[no speech recognized]'}")
    print(f"Saved full transcript: {transcript_path}")
    command_text = command_after_wake_phrase(transcript)
    if command_text is None:
        print(f"NO ACTION: wake phrase '{WAKE_PHRASE}' was not heard; staying silent and still")
        return
    print(f"WAKE-PHRASE COMMAND: {command_text}")
    try:
        plan = parse_voice_command(command_text)
    except ValueError as exc:
        print(f"NO ACTION: {exc}")
        print(f"SASSY: {speak(choose_response('unknown'))}")
        return
    print("PLAN:")
    for step in plan:
        print(f"- {step['label']}  [D,{step['left']},{step['right']}] for {step['seconds']:.1f}s")
    if not args.execute:
        print("Preview only. Re-run with --execute after checking the plan.")
        return
    payload = json.dumps({"transcript": command_text}).encode()
    try:
        response = request.urlopen(request.Request(args.url, data=payload, headers={"Content-Type": "application/json"}), timeout=5)
        print(response.read().decode())
    except error.HTTPError as exc:
        print(f"NO ACTION: controller rejected command ({exc.read().decode()})")
        print(f"SASSY: {speak(choose_response('nack'))}")
        return
    print(f"SASSY: {speak(choose_response('ack'))}")


if __name__ == "__main__":
    main()
