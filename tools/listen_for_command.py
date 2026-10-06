#!/usr/bin/env python3
"""Continuously listen for wake-phrase-qualified Sassy commands.

The microphone is captured continuously. A small local voice-activity detector
groups a natural utterance into one WAV before Whisper sees it, so command
recognition is not tied to arbitrary fixed recording windows.
"""
import argparse
import audioop
from collections import deque
import json
from datetime import datetime
from pathlib import Path
import queue
import subprocess
import threading
from urllib import error, request
import wave

from record_and_transcribe import DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL, transcribe_whisper
from voice_commands import (
    WAKE_PHRASE, command_after_wake_phrase, contains_wake_phrase,
    command_may_continue, first_sentence, parse_voice_command,
)
from sassy_speech import UNKNOWN_RESPONSE, cannot_do, choose_response, speak


SAMPLE_RATE_HZ = 16_000
CHANNELS = 2
SAMPLE_WIDTH_BYTES = 2
FRAME_MS = 100
SPEECH_START_FRAMES = 2
# Commands end quickly once the speaker stops.  Saying a final "go" makes the
# intended boundary unambiguous, while this short tail avoids a sluggish wait.
SPEECH_END_FRAMES = 3
LEAD_IN_FRAMES = 3
MIN_SPEECH_RMS = 1_200


def _capture_utterances(session, total_seconds, max_utterance_seconds, captured):
    """Continuously capture and enqueue only voice-like utterances.

    The threshold follows quiet room noise but has a conservative floor. The
    recognizer still requires "Hey Sassy", so a television, fan, or Sassy's
    own reply cannot command a motor.
    """
    command = [
        "arecord", "-D", "hw:0,0", "-f", "S16_LE", "-r", str(SAMPLE_RATE_HZ),
        "-c", str(CHANNELS), "-d", str(total_seconds), "-t", "raw",
    ]
    frame_bytes = SAMPLE_RATE_HZ * CHANNELS * SAMPLE_WIDTH_BYTES * FRAME_MS // 1_000
    max_frames = max(1, max_utterance_seconds * 1_000 // FRAME_MS)
    number = 0
    speaking = False
    noise_rms = 500.0
    loud_frames = 0
    quiet_frames = 0
    utterance = bytearray()
    lead_in = deque(maxlen=LEAD_IN_FRAMES)

    def emit():
        nonlocal number, utterance
        if not utterance:
            return
        number += 1
        output = Path("artifacts") / f"voice_command_{session}_{number:02d}.wav"
        output.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(output), "wb") as target:
            target.setnchannels(CHANNELS)
            target.setsampwidth(SAMPLE_WIDTH_BYTES)
            target.setframerate(SAMPLE_RATE_HZ)
            target.writeframes(utterance)
        captured.put((number, output))
        utterance = bytearray()

    print(f"Continuously listening {total_seconds} seconds from hw:0,0. Speak naturally.")
    with subprocess.Popen(command, stdout=subprocess.PIPE) as recorder:
        assert recorder.stdout is not None
        while True:
            frame = recorder.stdout.read(frame_bytes)
            if not frame:
                break
            level = audioop.rms(frame, SAMPLE_WIDTH_BYTES)
            threshold = max(MIN_SPEECH_RMS, noise_rms * 2.2)
            if not speaking:
                lead_in.append(frame)
                if level < threshold:
                    noise_rms = noise_rms * .97 + level * .03
                    loud_frames = 0
                    continue
                loud_frames += 1
                if loud_frames < SPEECH_START_FRAMES:
                    continue
                speaking = True
                quiet_frames = 0
                utterance = bytearray().join(lead_in)
                continue
            utterance.extend(frame)
            if level < threshold:
                quiet_frames += 1
            else:
                quiet_frames = 0
            if quiet_frames >= SPEECH_END_FRAMES or len(utterance) >= max_frames * frame_bytes:
                emit()
                speaking = False
                loud_frames = 0
                quiet_frames = 0
                lead_in.clear()
        if speaking:
            emit()
    captured.put(None)


def _merge_fragments(left, right):
    """Join adjacent speech, removing a repeated word at the boundary."""
    left_words = left.split()
    right_words = right.split()
    max_overlap = min(len(left_words), len(right_words))
    for count in range(max_overlap, 0, -1):
        if [word.lower() for word in left_words[-count:]] == [word.lower() for word in right_words[:count]]:
            return " ".join(left_words + right_words[count:])
    return " ".join(left_words + right_words)


def main():
    parser = argparse.ArgumentParser(description="Listen for one Sassy command.")
    parser.add_argument("--seconds", type=int, default=60, help="Total listening window (default: 60).")
    parser.add_argument("--chunk-seconds", type=int, default=8, help="Maximum natural utterance length (default: 8).")
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
    pending_command = ""
    captured = queue.Queue()
    capture_thread = threading.Thread(
        target=_capture_utterances,
        args=(session, args.seconds, args.chunk_seconds, captured),
        daemon=True,
    )
    capture_thread.start()
    while True:
        item = captured.get()
        if item is None:
            break
        chunk_number, output = item
        transcript = transcribe_whisper(output, DEFAULT_WHISPER_CLI, DEFAULT_WHISPER_MODEL)
        if transcript:
            all_transcript.append(transcript)
        transcript_path.write_text("\n".join(all_transcript) + "\n", encoding="utf-8")
        print(f"CHUNK {chunk_number} TRANSCRIPT: {transcript or '[no speech recognized]'}")
        command_text = command_after_wake_phrase(transcript)
        if command_text is None and awake:
            command_text = first_sentence(transcript)
        if command_text is None and contains_wake_phrase(transcript):
            # A wake phrase may end one chunk and its command begin in the next.
            awake = True
            print("Wake phrase heard; waiting for the command.")
            continue
        if command_text is None:
            continue  # No trigger: intentionally silent and motionless.
        if pending_command:
            command_text = _merge_fragments(pending_command, command_text)
        awake = False
        print(f"WAKE-PHRASE COMMAND: {command_text}")
        if handle_command(command_text, args):
            pending_command = ""
            continue
        if command_may_continue(command_text):
            # Do not reply to a fragment such as "go forward for".  Carry it
            # to the next audio chunk, which keeps natural speech usable.
            pending_command = command_text
            awake = True
            print("Command may continue in the next audio chunk; listening on.")
        else:
            pending_command = ""
            print(f"SASSY: {speak(UNKNOWN_RESPONSE)}")
    capture_thread.join()
    if pending_command:
        print(f"NO ACTION: incomplete or unsupported command: {pending_command}")
        print(f"SASSY: {speak(UNKNOWN_RESPONSE)}")
    print(f"FULL TRANSCRIPT: {' '.join(all_transcript) or '[no speech recognized]'}")
    print(f"Saved full transcript: {transcript_path}")


def handle_command(command_text, args):
    try:
        plan = parse_voice_command(command_text)
    except ValueError as exc:
        print(f"NO ACTION: {exc}")
        return False
    print("PLAN:")
    for step in plan:
        print(f"- {step['label']}  [D,{step['left']},{step['right']}] for {step['seconds']:.1f}s")
    if not args.execute:
        print("Preview only. Re-run with --execute after checking the plan.")
        return True
    payload = json.dumps({"transcript": command_text}).encode()
    try:
        response = request.urlopen(request.Request(args.url, data=payload, headers={"Content-Type": "application/json"}), timeout=5)
        print(response.read().decode())
    except error.HTTPError as exc:
        print(f"NO ACTION: controller rejected command ({exc.read().decode()})")
        print(f"SASSY: {speak(cannot_do(command_text))}")
        return True
    print(f"SASSY: {speak(choose_response('ack'))}")
    return True


if __name__ == "__main__":
    main()
