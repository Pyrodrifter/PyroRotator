import re
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class WebUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = (ROOT / "index_html.h").read_text(encoding="utf-8")

    def test_ui_stays_small_enough_for_flash_serving(self):
        self.assertLess(len(self.header.encode("utf-8")), 100_000)

    def test_trim_and_mission_elements_exist(self):
        for marker in ('data-tab="trim"', 'id="trimAz"', 'id="trimEl"', 'id="missionTarget"', '/api/trim'):
            self.assertIn(marker, self.header)

    def test_embedded_javascript_parses(self):
        scripts = re.findall(r"<script>(.*?)</script>", self.header, flags=re.S)
        self.assertEqual(len(scripts), 1)
        with tempfile.NamedTemporaryFile("w", suffix=".js", encoding="utf-8", delete=False) as temp:
            temp.write(scripts[0]); name = temp.name
        try:
            result = subprocess.run(["node", "--check", name], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
        finally:
            Path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
