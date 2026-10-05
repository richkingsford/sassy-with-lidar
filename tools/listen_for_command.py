#!/usr/bin/env python3
"""Record one spoken command, transcribe it locally, and optionally execute it."""
import argparse
import json
from datetime import datetime
from pathlib import Path
import time
from urllib import error, request

from record_and_transcribe import (
    DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL, record, transcribe_whisper,
)
from voice_commands import (
    WAKE_PHRASE, command_after_wake_phrase, contains_wake_phrase,
    first_sentence, parse_voice_command,
)
from sassy_speech import choose_response, speak


def main():
    parser = argparse.ArgumentParser(description="Listen for one Sassy command.")
    parser.add_argument("--seconds", type=int, default=60, help="Total listening window (default: 60).")
    parser.add_argument("--chunk-seconds", type=int, default=4, help="Maximum recording chunk (default: 4).")
    parser.add_argument("--execute", action="store_true", help="Send a recognized plan to the guarded dashboard controller.")
    parser.add_argument("--url", default="http://127.0.0.1:8080/voice", help="Guarded dashboard voice endpoint.")
    args = parser.parse_args()
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    if args.chunk_seconds <= 0:
        raise ValueError("--chunk-seconds must be positive")
    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    transcript_path = Path("artifacts") / f"voice_command_{session}.heard.txt"
    all_transcript = []
    awake = False
    deadline = time.monotonic() + args.seconds
    chunk_number = 0
    while time.monotonic() < deadline:
        chunk_number += 1
        chunk_seconds = min(args.chunk_seconds, max(1, round(deadline - time.monotonic())))
        output = Path("artifacts") / f"voice_command_{session}_{chunk_number:02d}.wav"
        record(output, chunk_seconds, "hw:0,0")
        transcript = transcribe_whisper(output, DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL)
        if transcript:
            all_transcript.append(transcript)
        transcript_path.write_text("\n".join(all_transcript) + "\n", encoding="utf-8")
        print(f"CHUNK {chunk_number} TRANSCRIPT: {transcript or '[no speech recognized]'}")
        command_text = command_after_wake_phrase(transcript)
        if command_text is None and awake:
            command_text = first_sentence(transcript)
            awake = False
        if command_text is None and contains_wake_phrase(transcript):
            # A natural pause after "Hey Sassy" can cross the 4-second boundary.
            awake = True
            print(f"Wake phrase heard; waiting for the next short command segment.")
            continue
        if command_text is None:
            continue  # No trigger: intentionally silent and motionless.
        print(f"WAKE-PHRASE COMMAND: {command_text}")
        handle_command(command_text, args)
    print(f"FULL TRANSCRIPT: {' '.join(all_transcript) or '[no speech recognized]'}")
    print(f"Saved full transcript: {transcript_path}")


def handle_command(command_text, args):
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
