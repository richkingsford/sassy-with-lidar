"""Sassy's natural spoken acknowledgements, sourced from the project prompt list."""
from pathlib import Path
import os
import random
import subprocess
import sys


RESPONSE_FILE = Path("docs/acks nacks and I dont follow.txt")
SINK = "alsa_output.usb-Seeed_Studio_ReSpeaker_Lite_0000000001-00.analog-stereo"
FALLBACK = {
    "ack": ("Command accepted.",),
    "nack": ("I cannot safely do that right now.",),
    "unknown": ("I heard you, but I did not understand a command.",),
}


def _responses():
    """Read the user-authored response sets without duplicating their text in code."""
    groups = {"ack": [], "nack": [], "unknown": []}
    current = None
    try:
        lines = RESPONSE_FILE.read_text(encoding="utf-8").splitlines()
    except OSError:
        return FALLBACK
    for line in lines:
        lower = line.lower()
        if "positive acks" in lower:
            current = "ack"
        elif "positive" in lower and "nacks" in lower:
            current = "nack"
        elif "no command detected" in lower:
            current = "unknown"
        elif current and line.strip().startswith("“"):
            phrase = line.strip().strip("“”").strip()
            if phrase:
                groups[current].append(phrase)
    return {name: tuple(items) or FALLBACK[name] for name, items in groups.items()}


def choose_response(kind):
    return random.choice(_responses()[kind])


def speak(text):
    """Speak naturally through the ReSpeaker, with a local fallback if offline."""
    Path(".cache").mkdir(exist_ok=True)
    media = Path(".cache/sassy_response.mp3")
    try:
        subprocess.run([
            sys.executable, "-m", "edge_tts", "--voice", "en-US-JennyNeural",
            "--rate", "-5%", "--text", text, "--write-media", str(media),
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        subprocess.run([
            "gst-launch-1.0", "-q", "filesrc", f"location={media}", "!", "decodebin", "!",
            "audioconvert", "!", "audioresample", "!", "pulsesink", f"device={SINK}",
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
    except (OSError, subprocess.SubprocessError):
        environment = os.environ | {"PULSE_SINK": SINK}
        subprocess.run(["spd-say", "--wait", "--volume", "-20", text], env=environment, check=False)
    return text
