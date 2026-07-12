"""Regression checks for telemetry that shares the step-generation loop."""

from pathlib import Path
import re
import unittest


SKETCH = Path(__file__).resolve().parents[1] / "PyroRotator.ino"


class TrackingTelemetryTests(unittest.TestCase):
    def test_active_tracking_uses_compact_telemetry_on_both_transports(self):
        source = SKETCH.read_text(encoding="utf-8")
        compact = 'printf("T %.4f %.4f %.4f %.4f\\n"'

        self.assertEqual(source.count(compact), 2)
        self.assertRegex(
            source,
            re.compile(
                r"if \(g_trackStreamActive\) \{\s*"
                r"(?:\/\/[^\n]*\s*)*rotClient\.printf\(\"T %\.4f",
                re.MULTILINE,
            ),
        )
        self.assertRegex(
            source,
            re.compile(
                r"if \(g_trackStreamActive\) \{\s*"
                r"Serial\.printf\(\"T %\.4f",
                re.MULTILINE,
            ),
        )


if __name__ == "__main__":
    unittest.main()
