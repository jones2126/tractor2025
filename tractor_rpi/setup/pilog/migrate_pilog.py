#!/usr/bin/env python3
"""Operator-approved power logger migration; cleanup is a separate verified step."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent
OLD = Path('/var/log/pilog')
LIVE = Path('/usr/local/bin/pilog.sh')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_archive(path, expected):
    with tarfile.open(path, 'r:gz') as archive:
        members = archive.getmembers()
        if {m.name for m in members} != set(expected) or len(members) != len(expected):
            raise RuntimeError('Unexpected archive members')
        for member in members:
            if not member.isfile() or '/' in member.name or member.name in ('.', '..'):
                raise RuntimeError('Unsafe archive member')
            if hashlib.sha256(archive.extractfile(member).read()).hexdigest() != expected[member.name]:
                raise RuntimeError('Archive content mismatch: '+member.name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--apply', action='store_true')
    mode.add_argument('--update-notifier', action='store_true')
    mode.add_argument('--remove-archived-logs', type=Path, metavar='MANIFEST')
    parser.add_argument('--verified-off-pi-sha256')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run with sudo in PuTTY; no password is stored by this program')
    if args.update_notifier:
        if digest(LIVE) == digest(HERE/'pilog.sh'):
            print('Reviewed notification update is already installed.')
            return
        if digest(LIVE) != '8f81cf60175639d62dc48448bccf2d719cace75c50d83c44a2c121f8f099e562':
            raise RuntimeError('Installed logger differs from reviewed daily-storage version')
        subprocess.run(['bash','-n',str(HERE/'pilog.sh')],check=True)
        timer_active = subprocess.run(['systemctl','is-active','--quiet','pilog.timer']).returncode == 0
        subprocess.run(['systemctl','stop','pilog.timer'],check=True)
        try:
            if subprocess.run(['systemctl','is-active','--quiet','pilog.service']).returncode == 0:
                raise RuntimeError('A sample is running; retry after it finishes')
            stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            backup = Path('/var/backups')/('pilog-notifier-'+stamp)
            backup.mkdir()
            shutil.copy2(LIVE,backup/'pilog.sh')
            shutil.copyfile(HERE/'pilog.sh',LIVE)
            LIVE.chmod(0o755)
            subprocess.run(['systemctl','start','pilog.service'],check=True)
            if digest(LIVE) != digest(HERE/'pilog.sh'):
                raise RuntimeError('Installed source verification failed')
            print('Notification update installed; sample completed. Backup:',backup)
            state = Path('/run/pilog.current')
            print('LOG:',state.read_text().strip())
            print('NOTICE:', 'accepted by ntfy' if Path(str(state)+'.notice-sent').exists() else 'pending for next timer sample')
        finally:
            if timer_active:
                subprocess.run(['systemctl','start','pilog.timer'],check=True)
        return
    if args.remove_archived_logs:
        manifest = json.loads(args.remove_archived_logs.read_text())
        archive = args.remove_archived_logs.parent / 'pilog_legacy.tgz'
        expected = manifest['files']
        if args.verified_off_pi_sha256 != manifest['archive_sha256'] or digest(archive) != manifest['archive_sha256']:
            parser.error('Supply the independently verified off-Pi archive SHA256')
        check_archive(archive, expected)
        if OLD.is_symlink() or OLD.resolve() != Path('/var/log/pilog'):
            raise RuntimeError('Unexpected legacy directory')
        if OLD.exists():
            if {p.name for p in OLD.iterdir()} != set(expected):
                raise RuntimeError('Legacy contents changed; do not delete')
            for name, sha in expected.items():
                if Path(name).name != name or (OLD/name).is_symlink() or digest(OLD/name) != sha:
                    raise RuntimeError('Legacy file changed; do not delete')
            active_log = Path('/run/pilog.current').read_text().strip()
            if str(OLD) in active_log:
                raise RuntimeError('Logger still points at old directory')
            for name in expected:
                (OLD/name).unlink()
            OLD.rmdir()
        print('Verified legacy CSV originals removed; archive and rollback backup retained.')
        return

    if digest(LIVE) != digest(HERE/'installed_pilog_20261119.sh.txt'):
        raise RuntimeError('Installed script differs from reviewed original; inspect before migration')
    subprocess.run(['bash', '-n', str(HERE/'pilog.sh')], check=True)
    timer_active = subprocess.run(['systemctl','is-active','--quiet','pilog.timer']).returncode == 0
    subprocess.run(['systemctl','stop','pilog.timer'], check=True)
    try:
        if subprocess.run(['systemctl','is-active','--quiet','pilog.service']).returncode == 0:
            raise RuntimeError('A sample is running; retry after it finishes')
        stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        backup = Path('/var/backups') / ('pilog-migration-'+stamp)
        backup.mkdir()
        shutil.copy2(LIVE, backup/'pilog.sh')
        state = Path('/run/pilog.current')
        if state.exists():
            shutil.copy2(state, backup/'pilog.current')
        if OLD.is_symlink():
            raise RuntimeError('Legacy directory must not be a symlink')
        files = sorted(OLD.iterdir())
        if any(not p.is_file() or p.is_symlink() for p in files):
            raise RuntimeError('Unexpected legacy directory entries')
        day = dt.datetime.now(ZoneInfo('America/New_York')).strftime('%Y-%m-%d')
        daily = Path('/home/al/field_logs')/day
        if not daily.exists():
            daily.mkdir()
            shutil.chown(daily, user='al', group='al')
        target = daily / ('pilog_legacy_archive_'+stamp)
        target.mkdir(mode=0o750)
        shutil.chown(target, user='al', group='al')
        hashes = {p.name:digest(p) for p in files}
        archive = target/'pilog_legacy.tgz'
        with tarfile.open(archive,'w:gz') as tar:
            for p in files:
                tar.add(p,arcname=p.name,recursive=False)
        check_archive(archive,hashes)
        manifest = dict(archive_sha256=digest(archive), files=hashes,
                        backup=str(backup), original_script_sha256=digest(LIVE))
        (target/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (target/'pilog_legacy.tgz.sha256').write_text(manifest['archive_sha256']+'  pilog_legacy.tgz\n')
        for p in target.iterdir():
            p.chmod(0o640)
            shutil.chown(p,user='al',group='al')
        shutil.copyfile(HERE/'pilog.sh',LIVE)
        LIVE.chmod(0o755)
        subprocess.run(['systemctl','start','pilog.service'],check=True)
        current = Path(state.read_text().strip())
        if current.parent != daily or len(current.read_text().splitlines()) < 2:
            raise RuntimeError('New daily sample missing; inspect backup '+str(backup))
        print('INSTALLED and sample verified:',current)
        print(current.read_text())
        print('ARCHIVE:',target)
        print('SHA256:',manifest['archive_sha256'])
        print('Legacy originals retained until independent off-Pi verification.')
        print('BACKUP:',backup)
    finally:
        if timer_active:
            subprocess.run(['systemctl','start','pilog.timer'],check=True)


if __name__ == '__main__':
    main()
