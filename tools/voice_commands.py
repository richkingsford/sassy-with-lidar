"""Parse Sassy's safety-bounded spoken driving vocabulary.

The parser accepts many natural phrasings, but it intentionally produces only
short, explicit direct-tread plans. Ambiguous speech remains a no-motion
response rather than a guess.
"""
import re


MAX_DRIVE_SECONDS = 90.0
MAX_TURN_DEGREES = 360.0
MAX_TURN_SECONDS = 4.0
TURN_90_SECONDS = 1.5  # Full-power calibration: spoken 90° pivot duration.
# Full PWM is the user's current operating setting for every spoken movement:
# straight driving, reversing, and one-wheel pivots.
VOICE_POWER = 255
WAKE_PHRASE = "hey sassy"

NUMBER_WORDS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4,
    "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
    "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
    "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
    "eighteen": 18, "nineteen": 19, "twenty": 20, "thirty": 30,
    "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70,
    "eighty": 80, "ninety": 90, "hundred": 100,
}


def _normalise(text):
    text = re.sub(r"[^a-z0-9. ]", " ", text.lower())
    # "turn right to 90 degrees" and "turn to the right" mean the same
    # controlled pivot as their shorter equivalents.
    text = re.sub(r"\bto(?: the)?\b", "", text)
    # Whisper occasionally repeats an adjacent direction word at a chunk edge.
    text = re.sub(r"\b(?:go )?back backwards\b", "go backwards", text)
    text = re.sub(r"\b(?:go )?forward forwards\b", "go forward", text)
    # A common Whisper homophone in commands such as "turn left ninety
    # degrees". Limit the repair to the unit-bearing form to avoid guessing.
    text = re.sub(r"\btonight(?= degrees\b)", "ninety", text)
    text = re.sub(r"\bstraight ahead\b|\bahead\b", "forward", text)
    text = re.sub(r"\bin reverse\b|\breverse\b", "backward", text)
    text = re.sub(r"\b(?:about|approximately|roughly)\b", "", text)
    text = re.sub(r"\bdegrees?\b", "degrees", text)
    # A final "go" is a spoken submit button, not a movement instruction.
    text = re.sub(r"\bgo\s*$", "", text)
    return re.sub(r"\s+", " ", text).strip()


def command_after_wake_phrase(transcript):
    """Return the first sentence after the last wake phrase, or None if asleep."""
    pieces = re.split(r"\bhey\s+sassy\b", transcript, flags=re.IGNORECASE)
    if len(pieces) < 2:
        return None
    candidate = re.split(r"[.!?]", pieces[-1], maxsplit=1)[0].strip(" ,;:-")
    return candidate or None


def contains_wake_phrase(transcript):
    return re.search(r"\bhey\s+sassy\b", transcript, flags=re.IGNORECASE) is not None


def first_sentence(transcript):
    """Return the next spoken sentence for a wake phrase spanning two chunks."""
    return re.split(r"[.!?]", transcript, maxsplit=1)[0].strip(" ,;:-") or None


def command_may_continue(transcript):
    """Whether a wake-qualified fragment could become a supported command."""
    text = _normalise(transcript)
    if not text:
        return True
    starters = (
        "flip", "turn", "rotate", "spin", "make", "right", "left",
        "forward", "back", "backward", "backwards", "go", "move",
        "drive", "roll", "crawl",
    )
    return any(starter.startswith(text) or text.startswith(starter) for starter in starters)


def _number(text):
    value = text.strip()
    try:
        return float(value)
    except ValueError:
        pass
    current = 0
    for word in value.replace("-", " ").split():
        if word == "and":
            continue
        if word not in NUMBER_WORDS:
            raise ValueError(f"I could not understand the number '{text}'")
        number = NUMBER_WORDS[word]
        if word == "hundred":
            current = max(1, current) * number
        else:
            current += number
    return float(current)


def _bounded_number(text, label, minimum, maximum):
    value = _number(text)
    if not minimum <= value <= maximum:
        raise ValueError(f"{label} must be between {minimum:g} and {maximum:g}")
    return value


def _seconds(text):
    return _bounded_number(text, "Duration", .2, MAX_DRIVE_SECONDS)


def _step(label, left, right, seconds):
    return {"label": label, "left": left, "right": right, "seconds": seconds}


def turn_step(direction, degrees=90.0):
    seconds = TURN_90_SECONDS * degrees / 90.0
    if direction == "right":
        return _step(f"TURN RIGHT {degrees:g} degrees", VOICE_POWER, 0, seconds)
    return _step(f"TURN LEFT {degrees:g} degrees", 0, VOICE_POWER, seconds)


def _strip_politeness(text):
    return re.sub(r"^(?:please |can you |could you |would you |will you )+", "", text).strip()


def _parse_motion(text):
    direction = r"(?P<direction>forward|forwards|back|backward|backwards)"
    verb = r"(?:(?:go|move|drive|roll|crawl)(?: straight)? )?"
    for pattern in (
        rf"{verb}{direction}(?: for)? (?P<seconds>[a-z0-9. -]+?) (?:seconds?|secs?)",
        rf"{direction}(?: for)? (?P<seconds>[a-z0-9. -]+?) (?:seconds?|secs?)",
    ):
        match = re.fullmatch(pattern, text)
        if not match:
            continue
        seconds = _seconds(match.group("seconds"))
        forward = match.group("direction").startswith("forward")
        sign = 1 if forward else -1
        label = "FORWARD" if forward else "BACKWARD"
        return _step(f"{label} for {seconds:g} seconds", sign * VOICE_POWER, sign * VOICE_POWER, seconds)
    return None


def _parse_turn(text):
    if text in ("flip", "flip a 180", "turn around", "u turn", "uturn"):
        return turn_step("right", 180)
    tokens = text.split()
    directions = [token for token in tokens if token in ("right", "left")]
    if len(directions) != 1:
        return None
    direction = directions[0]
    turn_words = {"turn", "rotate", "spin", "pivot", "make"}
    if not (set(tokens) & turn_words or tokens[0] == direction):
        return None
    if any(word in {"forward", "forwards", "back", "backward", "backwards"} for word in tokens):
        return None
    units = [token for token in tokens if token in ("degrees", "second", "seconds", "sec", "secs")]
    if len(set(units)) > 1 or len(units) > 1:
        return None
    permitted = turn_words | {"a", "right", "left", "by", "for", "degrees", "second", "seconds", "sec", "secs"}
    number_words = [token for token in tokens if token not in permitted]
    if any(token != "and" and token not in NUMBER_WORDS and not re.fullmatch(r"\d+(?:\.\d+)?", token) for token in number_words):
        return None
    number_text = " ".join(number_words)
    if not units and not number_text:
        return turn_step(direction, 90)
    if not number_text:
        return None
    if units and units[0] == "degrees":
        degrees = _bounded_number(number_text, "Turn angle", 5, MAX_TURN_DEGREES)
        return turn_step(direction, degrees)
    if units and units[0] in ("second", "seconds", "sec", "secs"):
        seconds = _bounded_number(number_text, "Turn duration", .2, MAX_TURN_SECONDS)
        return _step(f"TURN {direction.upper()} for {seconds:g} seconds", VOICE_POWER if direction == "right" else 0, 0 if direction == "right" else VOICE_POWER, seconds)
    # "turn right 90" is naturally understood as degrees; retain the safe
    # configured range instead of guessing a duration.
    return turn_step(direction, _bounded_number(number_text, "Turn angle", 5, MAX_TURN_DEGREES))


def _parse_one(text):
    text = _strip_politeness(_normalise(text))
    if not text:
        raise ValueError("I did not hear a command")
    motion = _parse_motion(text)
    if motion is not None:
        return motion
    turn = _parse_turn(text)
    if turn is not None:
        return turn
    raise ValueError("Unsupported command. Try 'forward for two seconds' or 'turn right 90 degrees'.")


def parse_voice_command(transcript):
    """Return a safe multi-step plan or raise ValueError for unsupported speech.

    Commands may contain up to three explicit steps joined with "then",
    "and then", or "after that". This keeps every movement reviewable and
    prevents a long unbounded spoken route from being executed.
    """
    text = _normalise(transcript)
    parts = [part.strip() for part in re.split(r"\b(?:and then|then|after that|afterwards)\b", text) if part.strip()]
    if not 1 <= len(parts) <= 3:
        raise ValueError("Use one to three movement steps joined with 'then'")
    return [_parse_one(part) for part in parts]
