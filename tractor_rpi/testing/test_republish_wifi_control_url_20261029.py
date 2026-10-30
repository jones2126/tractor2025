#!/usr/bin/env python3

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WEBRTC = ROOT / "tractor_rpi" / "testing" / "webrtc"
if str(WEBRTC) not in sys.path:
    sys.path.insert(0, str(WEBRTC))
SOURCE = WEBRTC / "republish_wifi_control_url_20261029.py"
SPEC = importlib.util.spec_from_file_location("republish_wifi_control", SOURCE)
republish = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(republish)


class RepublishWifiControlUrlTests(unittest.TestCase):
    def test_reads_valid_persistent_key(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "key"
            path.write_text("A_valid-key_12345678901234567890\n", encoding="ascii")
            self.assertEqual(
                "A_valid-key_12345678901234567890",
                republish.read_operator_key(path),
            )

    def test_rejects_invalid_key_without_publishing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "key"
            path.write_text("short key with spaces\n", encoding="ascii")
            with self.assertRaises(ValueError):
                republish.read_operator_key(path)


if __name__ == "__main__":
    unittest.main()

