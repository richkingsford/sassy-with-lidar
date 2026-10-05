"""Parse Sassy's initial small, safety-bounded spoken-command vocabulary."""
import re


MAX_DRIVE_SECONDS = 10.0
TURN_90_SECONDS = 1.0  # Calibrate on the real floor before relying on geometry.
VOICE_POWER = 178  # 70%, matching the proven one-wheel turn test.

NUMBER_WORDS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4,
    "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
    "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
    "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
    "eighteen": 18, "nineteen": 19, "twenty": 20,
}


def _normalise(text):
    return re.sub(r"\s+", " ", re.sub(r"[^a-z0-9. ]", " ", text.lower())).strip()


def _seconds(text):
    """Accept digits and basic spoken numbers, with a deliberately small cap."""
    value = text.strip()
    try:
        seconds = float(value)
    except ValueError:
        words = value.replace("-", " ").split()
        if len(words) == 1 and words[0] in NUMBER_WORDS:
            seconds = float(NUMBER_WORDS[words[0]])
        elif len(words) == 2 and words[0] in ("twenty",) and words[1] in NUMBER_WORDS:
            seconds = float(NUMBER_WORDS[words[0]] + NUMBER_WORDS[words[1]])
        else:
            raise ValueError(f"I could not understand the duration '{text}'")
    if not .2 <= seconds <= MAX_DRIVE_SECONDS:
        raise ValueError(f"Duration must be between 0.2 and {MAX_DRIVE_SECONDS:g} seconds")
    return seconds


def _step(label, left, right, seconds):
    return {"label": label, "left": left, "right": right, "seconds": seconds}


def turn_right_90():
    return _step("TURN RIGHT 90 degrees", VOICE_POWER, 0, TURN_90_SECONDS)


def turn_left_90():
    return _step("TURN LEFT 90 degrees", 0, VOICE_POWER, TURN_90_SECONDS)


def parse_voice_command(transcript):
    """Return a safe motion plan or raise ValueError for an unsupported phrase."""
    text = _normalise(transcript)
    if not text:
        raise ValueError("I did not hear a command")

    if re.fullmatch(r"(?:flip a? 180|turn around|u turn)", text):
        return [_step("TURN RIGHT 180 degrees", VOICE_POWER, 0, TURN_90_SECONDS * 2)]

    # Keep this before the generic forward pattern so the two-step command has
    # one unambiguous interpretation.
    sequence = re.fullmatch(
        r"(?:go )?forwards? for (.+?) seconds?(?: then)? (?:turn )?right (?:90|ninety)(?: degrees?)?",
        text,
    )
    if sequence:
        seconds = _seconds(sequence.group(1))
        return [_step(f"FORWARD for {seconds:g} seconds", VOICE_POWER, VOICE_POWER, seconds), turn_right_90()]

    if re.fullmatch(r"(?:turn )?right (?:90|ninety)(?: degrees?)?", text):
        return [turn_right_90()]
    if re.fullmatch(r"(?:turn )?left (?:90|ninety)(?: degrees?)?", text):
        return [turn_left_90()]

    forward = re.fullmatch(r"(?:go )?forwards? for (.+?) seconds?", text)
    if forward:
        seconds = _seconds(forward.group(1))
        return [_step(f"FORWARD for {seconds:g} seconds", VOICE_POWER, VOICE_POWER, seconds)]
    backward = re.fullmatch(r"(?:go )?(?:back|backward|backwards) for (.+?) seconds?", text)
    if backward:
        seconds = _seconds(backward.group(1))
        return [_step(f"BACKWARD for {seconds:g} seconds", -VOICE_POWER, -VOICE_POWER, seconds)]
    raise ValueError("Unsupported command. Try 'forward for two seconds' or 'turn left 90 degrees'.")
