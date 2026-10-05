#!/usr/bin/env python3
"""Record Sassy's ReSpeaker Lite, then transcribe it locally with Vosk.

The default is deliberately a single, supervised ten-second capture.  It does
not listen continuously and makes no network requests while recording or
transcribing.
"""
import argparse
import audioop
from datetime import datetime
import json
from pathlib import Path
import subprocess
import wave

from vosk import KaldiRecognizer, Model, SetLogLevel


RECORD_DEVICE = "hw:0,0"
SAMPLE_RATE_HZ = 16_000
CHANNELS = 2
DEFAULT_MODEL = Path("models/vosk-model-small-en-us-0.15")


def parse_args():
    parser = argparse.ArgumentParser(description="Record and locally transcribe the ReSpeaker Lite.")
    parser.add_argument("--seconds", type=int, default=10, help="Capture length (default: 10).")
    parser.add_argument("--device", default=RECORD_DEVICE, help="ALSA capture device.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="Vosk model directory.")
    parser.add_argument("--output", type=Path, help="Output WAV path; a timestamped artifacts path is the default.")
    return parser.parse_args()


def record(output, seconds, device):
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "arecord", "-D", device, "-f", "S16_LE", "-r", str(SAMPLE_RATE_HZ),
        "-c", str(CHANNELS), "-d", str(seconds), "-t", "wav", str(output),
    ]
    print(f"Recording {seconds} seconds from {device}. Speak now.")
    subprocess.run(command, check=True)
    print(f"Saved recording: {output}")


def transcribe(audio_path, model_path):
    if not model_path.is_dir():
        raise FileNotFoundError(f"Vosk model directory is missing: {model_path}")
    with wave.open(str(audio_path), "rb") as source:
        if source.getframerate() != SAMPLE_RATE_HZ or source.getsampwidth() != 2:
            raise ValueError("Expected 16 kHz, 16-bit PCM WAV capture")
        recognizer = KaldiRecognizer(Model(str(model_path)), SAMPLE_RATE_HZ)
        fragments = []
        while True:
            frames = source.readframes(4_000)
            if not frames:
                break
            # The ReSpeaker captures stereo; Vosk expects mono signed PCM.
            mono = audioop.tomono(frames, source.getsampwidth(), 0.5, 0.5) if source.getnchannels() == 2 else frames
            if recognizer.AcceptWaveform(mono):
                text = json.loads(recognizer.Result()).get("text", "").strip()
                if text:
                    fragments.append(text)
        final_text = json.loads(recognizer.FinalResult()).get("text", "").strip()
        if final_text:
            fragments.append(final_text)
    return " ".join(fragments).strip()


def main():
    args = parse_args()
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    output = args.output or Path("artifacts") / f"voice_capture_{datetime.now():%Y%m%d_%H%M%S}.wav"
    SetLogLevel(-1)
    record(output, args.seconds, args.device)
    text = transcribe(output, args.model)
    transcript_path = output.with_suffix(".txt")
    transcript_path.write_text(text + "\n", encoding="utf-8")
    print("TRANSCRIPT:")
    print(text or "[No speech recognized]")
    print(f"Saved transcript: {transcript_path}")


if __name__ == "__main__":
    main()
