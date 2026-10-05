#!/usr/bin/env python3
"""Record Sassy's ReSpeaker Lite, then transcribe it locally.

The default is deliberately a single, supervised ten-second capture.  It does
not listen continuously.  Whisper is the default, higher-accuracy offline
engine; Vosk remains available as a lightweight fallback.
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
DEFAULT_WHISPER_CLI = Path(".cache/whisper.cpp/build/bin/whisper-cli")
DEFAULT_WHISPER_MODEL = Path(".cache/whisper.cpp/models/ggml-base.en.bin")


def parse_args():
    parser = argparse.ArgumentParser(description="Record and locally transcribe the ReSpeaker Lite.")
    parser.add_argument("--seconds", type=int, default=10, help="Capture length (default: 10).")
    parser.add_argument("--device", default=RECORD_DEVICE, help="ALSA capture device.")
    parser.add_argument("--engine", choices=("whisper", "vosk"), default="whisper", help="Local transcription engine.")
    parser.add_argument("--vosk-model", type=Path, default=DEFAULT_MODEL, help="Vosk model directory.")
    parser.add_argument("--whisper-cli", type=Path, default=DEFAULT_WHISPER_CLI, help="whisper.cpp executable.")
    parser.add_argument("--whisper-model", type=Path, default=DEFAULT_WHISPER_MODEL, help="Whisper model file.")
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


def transcribe_vosk(audio_path, model_path):
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


def transcribe_whisper(audio_path, cli_path, model_path):
    """Downmix the ReSpeaker WAV and transcribe it with local whisper.cpp."""
    audio_path = Path(audio_path)
    cli_path = Path(cli_path)
    model_path = Path(model_path)
    if not cli_path.is_file():
        raise FileNotFoundError(f"whisper.cpp executable is missing: {cli_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"Whisper model is missing: {model_path}")
    mono_path = audio_path.with_name(f"{audio_path.stem}_mono.wav")
    result_base = audio_path.with_name(f"{audio_path.stem}_whisper")
    subprocess.run(["sox", str(audio_path), "-c", "1", "-r", str(SAMPLE_RATE_HZ), "-b", "16", str(mono_path)], check=True)
    subprocess.run([
        str(cli_path), "-m", str(model_path), "-f", str(mono_path), "-l", "en",
        "-t", "6", "-nt", "-np", "-otxt", "-of", str(result_base),
    ], check=True)
    return result_base.with_suffix(".txt").read_text(encoding="utf-8").strip()


def main():
    args = parse_args()
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    output = args.output or Path("artifacts") / f"voice_capture_{datetime.now():%Y%m%d_%H%M%S}.wav"
    SetLogLevel(-1)
    record(output, args.seconds, args.device)
    if args.engine == "whisper":
        text = transcribe_whisper(output, args.whisper_cli, args.whisper_model)
    else:
        text = transcribe_vosk(output, args.vosk_model)
    transcript_path = output.with_suffix(".txt")
    transcript_path.write_text(text + "\n", encoding="utf-8")
    print("TRANSCRIPT:")
    print(text or "[No speech recognized]")
    print(f"Saved transcript: {transcript_path}")


if __name__ == "__main__":
    main()
