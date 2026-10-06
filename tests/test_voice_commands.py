"""Regression tests for Sassy's bounded natural-language command grammar."""
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from voice_commands import parse_voice_command  # noqa: E402


class VoiceCommandTests(unittest.TestCase):
    def assert_step(self, phrase, label, left, right, seconds):
        plan = parse_voice_command(phrase)
        self.assertEqual(1, len(plan), phrase)
        step = plan[0]
        self.assertEqual(label, step["label"], phrase)
        self.assertEqual((left, right), (step["left"], step["right"]), phrase)
        self.assertAlmostEqual(seconds, step["seconds"], places=5, msg=phrase)

    def test_forward_and_backward_variants(self):
        self.assert_step("forward for 1.5 seconds", "FORWARD for 1.5 seconds", 255, 255, 1.5)
        self.assert_step("drive straight ahead for three secs", "FORWARD for 3 seconds", 255, 255, 3)
        self.assert_step("roll backward for nine seconds", "BACKWARD for 9 seconds", -255, -255, 9)
        self.assert_step("crawl in reverse for two seconds", "BACKWARD for 2 seconds", -255, -255, 2)
        self.assert_step("go forward for four seconds go", "FORWARD for 4 seconds", 255, 255, 4)

    def test_turn_variants_and_angles(self):
        phrases = (
            "turn right 90 degrees", "turn right for 90 degrees",
            "turn right by ninety degrees", "turn 90 degrees right",
            "rotate to the right 90 degrees", "right 90",
        )
        for phrase in phrases:
            self.assert_step(phrase, "TURN RIGHT 90 degrees", 255, 0, 1.5)
        self.assert_step("pivot left one hundred and eighty degrees", "TURN LEFT 180 degrees", 0, 255, 3)
        self.assert_step("spin right for 2.5 seconds", "TURN RIGHT for 2.5 seconds", 255, 0, 2.5)

    def test_bare_turn_and_multi_step_route(self):
        self.assert_step("make a left turn", "TURN LEFT 90 degrees", 0, 255, 1.5)
        plan = parse_voice_command("move forward for two seconds then turn left 45 degrees afterwards back for one second")
        self.assertEqual(["FORWARD for 2 seconds", "TURN LEFT 45 degrees", "BACKWARD for 1 seconds"], [step["label"] for step in plan])

    def test_out_of_bounds_commands_are_rejected(self):
        with self.assertRaises(ValueError):
            parse_voice_command("forward for 91 seconds")
        with self.assertRaises(ValueError):
            parse_voice_command("turn left 361 degrees")


if __name__ == "__main__":
    unittest.main()
