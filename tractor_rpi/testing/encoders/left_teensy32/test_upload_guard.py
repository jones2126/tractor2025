"""Offline upload targeting checks; no loader, USB or serial device accessed."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).parent
SPEC = importlib.util.spec_from_file_location("left_upload", ROOT / "upload_left.py")
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


def device(path="1-2.3", product="0483", serial="5879620"):
    return dict(path=path, vendor="16c0", product=product, serial=serial)


class GuardTests(unittest.TestCase):
    def test_correct_left_identity_selected(self):
        self.assertEqual(guard.target_path([device(), device("1-2.1", serial="16883790")]), "1-2.3")

    def test_missing_or_duplicate_identity_refused(self):
        for inventory in ([], [device(), device("1-2.4")]):
            with self.assertRaises(RuntimeError):
                guard.target_path(inventory)

    def test_preexisting_programming_mode_refused(self):
        with self.assertRaises(RuntimeError):
            guard.target_path([device(), device("1-2.1", "0478", None)])

    def test_wrong_usb_location_or_two_bootloaders_refused(self):
        for inventory in ([device("1-2.1", "0478", None)],
                          [device("1-2.3", "0478", None), device("1-2.1", "0478", None)]):
            with self.assertRaises(RuntimeError):
                guard.bootloader_ready(inventory, "1-2.3")

    def test_correct_location_accepted_only_after_button(self):
        self.assertFalse(guard.bootloader_ready([device()], "1-2.3"))
        self.assertTrue(guard.bootloader_ready([device(product="0478", serial=None)], "1-2.3"))

    def test_encoder_source_matches_reference_with_only_left_and_cpp_header_changes(self):
        reference = ROOT.parent / "wheel_encoder_status_20261117_reference.md"
        original = reference.read_text().split("```cpp\n", 1)[1].split("```", 1)[0]
        expected = "#include <Arduino.h>\n" + original.replace("#define WHEEL_LEFT 0", "#define WHEEL_LEFT 1", 1)
        self.assertEqual((ROOT / "src/main.cpp").read_text(), expected)


if __name__ == "__main__":
    unittest.main()
