"""Hardware-free boot-wrapper behavior using fake commands; no ntfy or hardware."""
import configparser
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / "tractor_rpi/testing/run_startup_phone_preflight_20261116.sh"
INSTALLER = ROOT / "tractor_rpi/setup/prepare_startup_repair_20261116.sh"
UNIT = ROOT / "tractor_rpi/setup/tractor-startup-preflight.service"
BASH = str(Path("C:/Program Files/Git/bin/bash.exe")) if os.name == "nt" else shutil.which("bash")


@unittest.skipUnless(BASH, "Bash is required for startup-wrapper behavioral tests")
class StartupRepairTests(unittest.TestCase):
    def run_wrapper(self, **overrides):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            commands = {
                "ip": '''[[ "${TEST_NETWORK_DOWN:-no}" != yes ]] || exit 1
                  case "$*" in
                  *"address show"*) echo "2: eth0 inet 192.168.8.20/24";;
                  *"1.1.1.1"*) echo "1.1.1.1 via 192.168.8.1 dev eth0 src 192.168.8.20";;
                  *"192.168.193.88"*) echo "192.168.193.88 dev ztTEST src 192.168.193.76";;
                  *) exit 1;; esac''',
                "ping": 'exit 0',
                "systemctl": '''echo "systemctl $*" >> "$TEST_LOG"
                  [[ "$*" != *"${TEST_FAILED_SERVICE:-NO_MATCH}"* ]]''',
                "curl": '''echo "curl $*" >> "$TEST_LOG"
                  exit "${TEST_CURL_EXIT:-0}"''',
                "python3": '''echo "python3 $*" >> "$TEST_LOG"
                  echo "Fake preflight checks"
                  if [[ "${TEST_MARKER:-yes}" == yes ]]; then echo "MISSION PREFLIGHT PASS"; fi
                  exit "${TEST_PREFLIGHT_EXIT:-0}"''',
            }
            for name, body in commands.items():
                script = directory / name
                script.write_text("#!/usr/bin/env bash\n" + body + "\n", encoding="utf-8", newline="\n")
                script.chmod(0o755)
            log = directory / "calls.txt"
            environment = {**os.environ, "TEST_LOG": log.as_posix(),
                           "TRACTOR_REPO": ROOT.as_posix(), "TRACTOR_NETWORK_WAIT_SECONDS": "1",
                           "TRACTOR_NETWORK_RETRY_SECONDS": "1", **overrides}
            # Explicit POSIX path ensures mocks take precedence on Git Bash too.
            result = subprocess.run([BASH, "-c", 'mock_dir="$1"; if command -v cygpath >/dev/null; then mock_dir="$(cygpath -u "$mock_dir")"; fi; export PATH="$mock_dir:/usr/bin:/bin"; exec bash "$2"',
                                     "test", directory.as_posix(), WRAPPER.as_posix()],
                                    env=environment, capture_output=True, text=True, timeout=12)
            return result, log.read_text() if log.exists() else ""

    def test_current_stack_can_pass_and_notifies_without_motion_commands(self):
        result, calls = self.run_wrapper()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("mission_preflight_phone_control_20261109.py", calls)
        self.assertIn("--expected-firmware teensy_main_20261110_rpi_command_lease_v5", calls)
        self.assertIn("Title: Tractor01 startup preflight PASS", calls)
        self.assertNotIn("tractor-wifi-control.service", calls)
        self.assertNotIn("teensy-bridge.service", calls)
        self.assertNotIn("systemctl start", calls)
        self.assertNotIn("systemctl restart", calls)

    def test_ntfy_failure_does_not_skip_local_preflight_or_change_pass(self):
        result, calls = self.run_wrapper(TEST_CURL_EXIT="22")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("python3 -u", calls)
        self.assertIn("best-effort ntfy publish failed", result.stderr)

    def test_nonzero_preflight_cannot_publish_pass_even_with_marker(self):
        result, calls = self.run_wrapper(TEST_PREFLIGHT_EXIT="1")
        self.assertEqual(result.returncode, 1)
        self.assertIn("startup preflight FAILED", calls)
        self.assertNotIn("Title: Tractor01 startup preflight PASS", calls)

    def test_zero_exit_without_exact_final_pass_is_failure(self):
        result, calls = self.run_wrapper(TEST_MARKER="no")
        self.assertEqual(result.returncode, 1)
        self.assertNotIn("Title: Tractor01 startup preflight PASS", calls)

    def test_missing_production_service_retries_without_preflight_or_notices(self):
        result, calls = self.run_wrapper(TEST_FAILED_SERVICE="tractor-phone-arbitrator.service")
        self.assertEqual(result.returncode, 75)
        self.assertNotIn("python3 ", calls)
        self.assertNotIn("curl ", calls)

    def test_invalid_intervals_fail_before_any_commands(self):
        result, calls = self.run_wrapper(TRACTOR_NETWORK_WAIT_SECONDS="0")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(calls, "")

    def test_wrong_network_never_runs_checks_or_sends_notices(self):
        result, calls = self.run_wrapper(TEST_NETWORK_DOWN="yes")
        self.assertEqual(result.returncode, 75)
        self.assertEqual(calls, "")

    def test_unit_uses_current_owners_and_complete_failure_is_not_retried(self):
        config = configparser.ConfigParser(interpolation=None)
        config.read(UNIT)
        requires = set(config["Unit"]["Requires"].split())
        self.assertEqual(requires, {"rtcm-server.service", "tractor-rpi-arbitrator.service",
                                    "tractor-phone-arbitrator.service"})
        self.assertNotIn("teensy-bridge", UNIT.read_text())
        self.assertNotIn("tractor-wifi-control", UNIT.read_text())
        self.assertIn(WRAPPER.name, config["Service"]["ExecStart"])
        self.assertEqual(config["Service"]["RestartPreventExitStatus"].split(), ["1", "2"])
        self.assertEqual(config["Service"]["TimeoutStartSec"], "480")

    def test_installer_plan_and_refusal_paths_do_not_mutate(self):
        result = subprocess.run([BASH, INSTALLER.as_posix(), "--plan"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("NO CHANGES MADE", result.stdout)
        refused = subprocess.run([BASH, INSTALLER.as_posix(), "--apply"], capture_output=True, text=True)
        self.assertEqual(refused.returncode, 2)
        text = INSTALLER.read_text()
        self.assertLess(text.index("cp -a --"), text.index("systemctl stop tractor-startup-preflight.service"))
        self.assertIn("systemctl mask teensy-bridge.service", text)
        self.assertIn("Refusing to alter an active production stack", text)
        self.assertNotIn("systemctl start ", text)
        self.assertNotIn("systemctl restart ", text)


if __name__ == "__main__":
    unittest.main()
