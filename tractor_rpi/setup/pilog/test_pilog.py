"""Isolated Linux checks: no Pi hardware, real notifications or system paths."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class DailyStorageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        scripts = {
            'date': '''#!/bin/bash
case "$1" in
  +%F) echo "$FAKE_DAY";;
  '+%F %T') echo "$FAKE_DAY 12:00:00";;
  +%Y-%m-%d_%H%M%S) echo "${FAKE_DAY}_120000";;
  *) exit 1;;
esac
''',
            'vcgencmd': '''#!/bin/bash
case "$1" in
 pmic_read_adc) echo 'EXT5V_V volt(24)=5.100V';;
 get_throttled) echo 'throttled=0x0';;
 measure_clock) echo 'frequency(0)=2400000000';;
 measure_temp) echo "temp=45.0'C";;
esac
''',
            'curl': '#!/bin/bash\necho notice >> "$NOTICE_LOG"\nprintf "%s" "${CURL_STATUS:-200}"\nexit "${CURL_EXIT:-0}"\n',
            'sync': '#!/bin/bash\nexit 0\n',
            'install': '#!/bin/bash\nmkdir -p "${@: -1}"\n',
        }
        for name, body in scripts.items():
            p = self.bin / name
            p.write_text(body)
            p.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.bin)+':'+os.environ['PATH'],
                        PILOG_LOG_ROOT=str(self.root/'logs'),
                        PILOG_STATE=str(self.root/'state'),
                        NOTICE_LOG=str(self.root/'notices'))

    def run_sample(self, day='2026-11-19'):
        self.env['FAKE_DAY'] = day
        subprocess.run(['bash', str(Path(__file__).with_name('pilog.sh'))],
                       env=self.env, check=True, capture_output=True)

    def test_samples_append_and_boot_notice_is_once(self):
        self.run_sample()
        self.run_sample()
        rows = next((self.root/'logs/2026-11-19').glob('*.csv')).read_text().splitlines()
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0], 'time,uptime_s,ext5v_v,throttled,load1,load5,arm_mhz,temp_c')
        self.assertIn(',5.100,0x0,', rows[1])
        self.assertEqual((self.root/'notices').read_text().splitlines(), ['notice'])

    def test_midboot_migration_preserves_old_file_without_boot_notice(self):
        old = self.root/'legacy.csv'
        old.write_text('preserved\n')
        (self.root/'state').write_text(str(old)+'\n')
        (self.root/'state.notice-sent').write_text('already accepted this boot\n')
        self.run_sample()
        self.assertEqual(old.read_text(), 'preserved\n')
        self.assertTrue(list((self.root/'logs/2026-11-19').glob('*.csv')))
        self.assertFalse((self.root/'notices').exists())

    def test_midnight_rotates_without_another_boot_notice(self):
        self.run_sample()
        self.run_sample('2026-11-20')
        for day in ('2026-11-19', '2026-11-20'):
            rows = next((self.root/'logs'/day).glob('*.csv')).read_text().splitlines()
            self.assertEqual(len(rows), 2)
        self.assertEqual((self.root/'notices').read_text().splitlines(), ['notice'])

    def test_network_failure_retries_later_without_losing_samples(self):
        self.env['CURL_EXIT'] = '6'
        self.env['CURL_STATUS'] = '000'
        self.run_sample()
        pending = (self.root/'state.notice-pending').read_bytes()
        self.assertFalse((self.root/'state.notice-sent').exists())
        self.run_sample('2026-11-20')
        self.assertEqual((self.root/'state.notice-pending').read_bytes(), pending)
        self.env['CURL_EXIT'] = '0'
        self.env['CURL_STATUS'] = '200'
        self.run_sample('2026-11-20')
        self.run_sample('2026-11-20')
        self.assertTrue((self.root/'state.notice-sent').exists())
        self.assertFalse((self.root/'state.notice-pending').exists())
        self.assertEqual(len((self.root/'notices').read_text().splitlines()), 3)
        self.assertEqual(sum(len(p.read_text().splitlines())-1 for p in (self.root/'logs').glob('*/*.csv')), 4)

    def test_non_success_http_remains_pending(self):
        for code in ('403', '429', '503', '302'):
            self.env['CURL_STATUS'] = code
            self.run_sample()
            self.assertFalse((self.root/'state.notice-sent').exists())
            self.assertTrue((self.root/'state.notice-pending').exists())
        self.env['CURL_STATUS'] = '201'
        self.run_sample()
        self.assertTrue((self.root/'state.notice-sent').exists())

    def test_upgrade_without_delivery_state_sends_one_labelled_sample(self):
        old = self.root/'legacy.csv'
        old.write_text('preserved\n')
        (self.root/'state').write_text(str(old)+'\n')
        self.env['CURL_EXIT'] = '6'
        self.run_sample()
        body = (self.root/'state.notice-pending').read_text()
        self.assertIn('Sample recorded at', body)
        self.assertIn('5V rail: 5.100 V', body)
        self.assertNotIn('booted at', body)


if __name__ == '__main__':
    unittest.main()
