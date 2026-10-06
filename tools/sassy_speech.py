"""Sassy's spoken acknowledgements and deterministic safety responses."""
from pathlib import Path
import os
import random
import re
import subprocess
import sys
import threading


RESPONSE_FILE = Path("docs/acks nacks and I dont follow.txt")
SINK = "alsa_output.usb-Seeed_Studio_ReSpeaker_Lite_0000000001-00.analog-stereo"
# Natural speech still plays directly through the ReSpeaker USB device.  The
# confirmation beep was removed; this is not a beep setting.
ALSA_DEVICE = "hw:0,0"
SPEECH_LOCK = threading.Lock()
FALLBACK = {
    "ack": ("Command accepted.",),
}
UNKNOWN_RESPONSE = "I hear you, but don't understand."


def _responses():
    """Read the user-authored response sets without duplicating their text in code."""
    groups = {"ack": []}
    current = None
    try:
        lines = RESPONSE_FILE.read_text(encoding="utf-8").splitlines()
    except OSError:
        return FALLBACK
    for line in lines:
        lower = line.lower()
        if lower.startswith("## acks"):
            current = "ack"
        elif lower.startswith("##"):
            current = None
        elif current:
            match = re.search(r"“(.+?)”", line)
            if match:
                groups[current].append(match.group(1).strip())
    return {name: tuple(items) or FALLBACK[name] for name, items in groups.items()}


def choose_response(kind):
    if kind != "ack":
        raise ValueError("Only acknowledgements are randomly selected")
    return random.choice(_responses()[kind])


def cannot_do(command_description):
    """Describe a rejected request consistently, using what Sassy heard."""
    description = re.sub(r"\s+", " ", str(command_description)).strip(" .")
    return f"I can't do this command: {description or 'no command'}."


def speak(text):
    """Speak naturally through the ReSpeaker, with a local fallback if offline."""
    # A single media path is safe because concurrent requests serialize here.
    # This also prevents an avoidance warning from interrupting an ACK.
    with SPEECH_LOCK:
        Path(".cache").mkdir(exist_ok=True)
        media = Path(".cache/sassy_response.mp3")
        try:
            subprocess.run([
                sys.executable, "-m", "edge_tts", "--voice", "en-US-JennyNeural",
                "--rate=-5%", "--text", text, "--write-media", str(media),
            ], check=True, timeout=30)
            subprocess.run([
                "gst-launch-1.0", "-q", "filesrc", f"location={media}", "!", "decodebin", "!",
                "audioconvert", "!", "audioresample", "!", "alsasink", f"device={ALSA_DEVICE}", "sync=true",
            ], check=True, timeout=30)
        except (OSError, subprocess.SubprocessError) as exc:
            print(f"Sassy speech synthesis/playback failed: {exc}; using fallback voice.")
            environment = os.environ | {"PULSE_SINK": SINK}
            subprocess.run(["spd-say", "--wait", "--volume", "-20", text], env=environment, check=False)
    return text


def speak_async(text):
    """Speak without blocking time-sensitive motor or sensor loops."""
    threading.Thread(target=speak, args=(text,), daemon=True).start()
