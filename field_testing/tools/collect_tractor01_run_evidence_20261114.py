#!/usr/bin/env python3
"""Build a verified, non-destructive Tractor01 post-mission evidence archive.

The mission launcher supplies a JSON run manifest containing exact mission,
Pursuit, route, audit, and dashboard-log paths.  This collector copies those
files plus current production diagnostics, phone logs, recovery logs, and Pi
power/resource logs.  Source files are never deleted or modified.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
from typing import Iterable


CURRENT_SERVICES = (
    "rtcm-server.service",
    "tractor-rpi-arbitrator.service",
    "tractor-phone-arbitrator.service",
)
REQUIRED_MANIFEST_FIELDS = (
    "schema",
    "run_id",
    "mission_label",
    "started_at_local",
    "started_at_epoch",
    "repository",
    "field_log",
    "pursuit_log",
    "dashboard_log",
    "mission_file",
    "audit_file",
    "expected_firmware",
    "archive_prefix",
)


class CollectionError(RuntimeError):
    """An error that makes the evidence package incomplete."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def safe_component(value: str, field: str) -> str:
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-")
    if not value or any(character not in allowed for character in value):
        raise CollectionError(f"manifest field {field!r} is not a safe filename component")
    return value


def load_manifest(path: Path) -> dict:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CollectionError(f"could not read run manifest {path}: {exc}") from exc
    missing = [field for field in REQUIRED_MANIFEST_FIELDS if field not in manifest]
    if missing:
        raise CollectionError(f"run manifest is missing: {', '.join(missing)}")
    if manifest["schema"] != 1:
        raise CollectionError(f"unsupported run-manifest schema: {manifest['schema']!r}")
    safe_component(str(manifest["run_id"]), "run_id")
    safe_component(str(manifest["mission_label"]), "mission_label")
    safe_component(str(manifest["archive_prefix"]), "archive_prefix")
    try:
        manifest["started_at_epoch"] = float(manifest["started_at_epoch"])
    except (TypeError, ValueError) as exc:
        raise CollectionError("started_at_epoch is not numeric") from exc
    return manifest


def require_source(path_value: str, label: str) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        raise CollectionError(f"{label} path is not absolute: {path}")
    if not path.is_file():
        raise CollectionError(f"required {label} is missing: {path}")
    if path.stat().st_size == 0:
        raise CollectionError(f"required {label} is empty: {path}")
    return path


def copy_required(path_value: str, destination: Path, label: str) -> Path:
    source = require_source(path_value, label)
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / source.name
    shutil.copy2(source, target)
    if target.stat().st_size != source.stat().st_size:
        raise CollectionError(f"copied {label} has the wrong size: {target}")
    return target


def iter_recent_files(
    root: Path,
    *,
    since_epoch: float,
    patterns: Iterable[str],
    max_depth: int | None = None,
) -> Iterable[Path]:
    if not root.is_dir():
        return
    seen: set[Path] = set()
    for pattern in patterns:
        for candidate in root.rglob(pattern):
            if not candidate.is_file() or candidate in seen:
                continue
            if max_depth is not None:
                relative_depth = len(candidate.relative_to(root).parts) - 1
                if relative_depth > max_depth:
                    continue
            try:
                modified = candidate.stat().st_mtime
            except OSError:
                continue
            if modified >= since_epoch - 1.0:
                seen.add(candidate)
                yield candidate


def copy_optional_files(
    sources: Iterable[Path], destination: Path, warnings: list[str]
) -> int:
    destination.mkdir(parents=True, exist_ok=True)
    count = 0
    for source in sorted(sources):
        target = destination / source.name
        if target.exists() and target.resolve() != source.resolve():
            target = destination / f"{source.parent.name}_{source.name}"
        try:
            shutil.copy2(source, target)
            count += 1
        except OSError as exc:
            warnings.append(f"could not copy optional file {source}: {exc}")
    return count


def capture_command(
    destination: Path,
    commands: list[list[str]],
    *,
    timeout: int = 45,
) -> None:
    lines: list[str] = []
    for command in commands:
        lines.append(f"$ {' '.join(command)}")
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                errors="replace",
                timeout=timeout,
                check=False,
            )
            lines.append(f"exit_code={result.returncode}")
            if result.stdout:
                lines.append(result.stdout.rstrip())
            if result.stderr:
                lines.append("[stderr]")
                lines.append(result.stderr.rstrip())
        except FileNotFoundError:
            lines.append("unavailable: command not installed")
        except subprocess.TimeoutExpired as exc:
            lines.append(f"timed out after {timeout} seconds")
            if exc.stdout:
                lines.append(str(exc.stdout))
        lines.append("")
    destination.write_text("\n".join(lines), encoding="utf-8")


def write_internal_manifest(evidence_root: Path) -> Path:
    manifest_path = evidence_root / "SHA256SUMS"
    rows = []
    for path in sorted(evidence_root.rglob("*")):
        if path.is_file() and path != manifest_path:
            relative = path.relative_to(evidence_root).as_posix()
            rows.append(f"{sha256_file(path)}  ./{relative}")
    if not rows:
        raise CollectionError("evidence directory contains no files")
    manifest_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return manifest_path


def verify_internal_manifest(evidence_root: Path, manifest_path: Path) -> None:
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        candidate = evidence_root / relative.removeprefix("./")
        if not candidate.is_file() or sha256_file(candidate) != expected:
            raise CollectionError(f"internal SHA-256 verification failed: {relative}")


def collect(args: argparse.Namespace) -> tuple[Path, Path, Path, list[str]]:
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = load_manifest(manifest_path)
    home_dir = Path(args.home_dir).expanduser().resolve()
    field_logs_root = Path(args.field_logs_root).expanduser().resolve()
    phone_log_dir = Path(args.phone_log_dir).expanduser().resolve()
    recovery_log_dir = Path(args.recovery_log_dir).expanduser().resolve()
    pilog_dir = Path(args.pilog_dir).expanduser().resolve()
    diagnostics_json = Path(args.diagnostics_json).expanduser().resolve()

    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    run_root = field_logs_root / manifest["run_id"]
    incomplete = run_root / f"evidence_incomplete_{stamp}"
    complete = run_root / f"evidence_complete_{stamp}"
    archive = home_dir / f"{manifest['archive_prefix']}_{stamp}_complete_evidence.tgz"
    checksum = Path(f"{archive}.sha256")
    for output in (incomplete, complete, archive, checksum):
        if output.exists():
            raise CollectionError(f"refusing to overwrite existing output: {output}")

    warnings: list[str] = []
    incomplete.mkdir(parents=True)
    mission_dir = incomplete / "mission"
    pursuit_dir = incomplete / "pursuit"
    phone_dir = incomplete / "phone"
    recovery_dir = incomplete / "recovery"
    package_dir = incomplete / "mission_package"
    diagnostics_dir = incomplete / "diagnostics"
    pilog_destination = diagnostics_dir / "pilog"
    diagnostics_dir.mkdir(parents=True)

    copy_required(manifest["field_log"], mission_dir, "mission CSV")
    copy_required(manifest["pursuit_log"], pursuit_dir, "Pursuit CSV")
    copy_required(manifest["dashboard_log"], diagnostics_dir, "dashboard transcript")
    copy_required(manifest["mission_file"], package_dir, "mission route")
    copy_required(manifest["audit_file"], package_dir, "mission audit")
    shutil.copy2(manifest_path, diagnostics_dir / "run_manifest.json")

    since_epoch = manifest["started_at_epoch"]
    phone_count = copy_optional_files(
        iter_recent_files(
            phone_log_dir,
            since_epoch=since_epoch,
            patterns=("wifi_primary_control_*.jsonl", "phone_flight_recorder_*.jsonl"),
            max_depth=1,
        ),
        phone_dir,
        warnings,
    )
    if phone_count == 0:
        warnings.append(f"no recent phone JSONL files found under {phone_log_dir}")

    recovery_sources = list(
        iter_recent_files(
            recovery_log_dir,
            since_epoch=since_epoch,
            patterns=("*.jsonl", "*.csv", "*.txt"),
            max_depth=2,
        )
    )
    recovery_sources.extend(
        iter_recent_files(
            field_logs_root,
            since_epoch=since_epoch,
            patterns=("*recovery*.jsonl",),
            max_depth=2,
        )
    )
    copy_optional_files(set(recovery_sources), recovery_dir, warnings)

    pilog_count = copy_optional_files(
        iter_recent_files(
            pilog_dir,
            # pilog normally samples every two minutes.  Include a five-minute
            # lead-in so a short mission still retains its pre-run power state.
            since_epoch=since_epoch - 300.0,
            patterns=("*",),
            max_depth=1,
        ),
        pilog_destination,
        warnings,
    )
    if pilog_count == 0 and not args.allow_missing_pilog:
        raise CollectionError(f"no readable Pi logs covering the run were found in {pilog_dir}")
    if pilog_count == 0:
        warnings.append(f"no Pi logs covering the run were found in {pilog_dir}")

    if diagnostics_json.is_file():
        shutil.copy2(diagnostics_json, diagnostics_dir / "phone_diagnostics.json")
    else:
        warnings.append(f"runtime phone diagnostics were unavailable: {diagnostics_json}")

    ended_at = dt.datetime.now().astimezone()
    metadata = {
        "collection_schema": 1,
        "collected_at_local": ended_at.isoformat(),
        "hostname": os.uname().nodename if hasattr(os, "uname") else os.environ.get("COMPUTERNAME"),
        "source_manifest": str(manifest_path),
        "run": manifest,
        "phone_files_collected": phone_count,
        "pilog_files_collected": pilog_count,
    }
    (diagnostics_dir / "collection_metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    if not args.skip_live_diagnostics:
        capture_command(
            diagnostics_dir / "service_status.txt",
            [["systemctl", "status", *CURRENT_SERVICES, "--no-pager", "-l"]],
        )
        capture_command(
            diagnostics_dir / "service_journal.log",
            [[
                "journalctl",
                *sum((["-u", service] for service in CURRENT_SERVICES), []),
                "--since",
                manifest["started_at_local"],
                "--no-pager",
            ]],
            timeout=90,
        )
        capture_command(
            diagnostics_dir / "network_state.txt",
            [
                ["ip", "-4", "address", "show"],
                ["ip", "-4", "route", "show"],
                ["zerotier-cli", "listnetworks"],
                ["zerotier-cli", "peers"],
                ["ss", "-lntup"],
            ],
        )
        capture_command(
            diagnostics_dir / "repository_state.txt",
            [
                ["git", "-C", manifest["repository"], "rev-parse", "HEAD"],
                ["git", "-C", manifest["repository"], "status", "--short", "--branch"],
            ],
        )
        capture_command(
            diagnostics_dir / "pilog_timer_status.txt",
            [
                ["systemctl", "status", "pilog.timer", "pilog.service", "--no-pager", "-l"],
                ["systemctl", "list-timers", "pilog.timer", "--no-pager"],
            ],
        )

    (diagnostics_dir / "collection_warnings.txt").write_text(
        ("\n".join(warnings) + "\n") if warnings else "None\n",
        encoding="utf-8",
    )
    internal_manifest = write_internal_manifest(incomplete)
    verify_internal_manifest(incomplete, internal_manifest)

    with tarfile.open(archive, "w:gz") as tar:
        tar.add(incomplete, arcname=complete.name)
    if not archive.is_file() or archive.stat().st_size == 0:
        raise CollectionError(f"archive was not created: {archive}")
    with tarfile.open(archive, "r:gz") as tar:
        members = [member for member in tar.getmembers() if member.isfile()]
        if not members:
            raise CollectionError("archive verification found no files")
    archive_hash = sha256_file(archive)
    checksum.write_text(f"{archive_hash}  {archive.name}\n", encoding="utf-8")
    if checksum.read_text(encoding="utf-8").split()[0] != sha256_file(archive):
        raise CollectionError("archive SHA-256 verification failed")
    incomplete.rename(complete)
    return complete, archive, checksum, warnings


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, help="Exact JSON run manifest")
    parser.add_argument("--home-dir", default="/home/al")
    parser.add_argument("--field-logs-root", default="/home/al/field_logs")
    parser.add_argument("--phone-log-dir", default="/home/al/field_logs/phone_arbitrator_production")
    parser.add_argument("--recovery-log-dir", default="/home/al/field_logs/manual_recovery")
    parser.add_argument("--pilog-dir", default="/var/log/pilog")
    parser.add_argument("--diagnostics-json", default="/run/tractor-phone-control/diagnostics.json")
    parser.add_argument(
        "--allow-missing-pilog",
        action="store_true",
        help="Create an archive with a warning if no applicable pilog file is readable",
    )
    parser.add_argument(
        "--skip-live-diagnostics",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        complete, archive, checksum, warnings = collect(args)
    except (CollectionError, OSError, tarfile.TarError) as exc:
        print("", flush=True)
        print("EVIDENCE COLLECTION FAILED", flush=True)
        print(f"Reason: {exc}", flush=True)
        print("TRACTOR IS STOPPED, BUT EVIDENCE HAS NOT BEEN PACKAGED.", flush=True)
        print("Do not shut down Tractor01 until manual collection succeeds.", flush=True)
        return 1

    print("", flush=True)
    print("EVIDENCE COLLECTION PASSED", flush=True)
    print(f"Evidence: {complete}", flush=True)
    print(f"Archive:  {archive}", flush=True)
    print(f"Checksum: {checksum}", flush=True)
    if warnings:
        print(f"Warnings: {len(warnings)} (see diagnostics/collection_warnings.txt)", flush=True)
    print("Copy the archive and checksum off Tractor01 and verify them before shutdown.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
