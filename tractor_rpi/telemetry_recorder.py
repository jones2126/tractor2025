#!/usr/bin/env python3
# Production-folder placement reviewed 2026-11-20; runtime logic unchanged.
"""Continuous raw telemetry and read-only preflight windows. DRAFT: field untested."""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict, deque
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import hashlib
import json
from pathlib import Path
import selectors
import shutil
import signal
import socket
import subprocess
import time
import uuid
from zoneinfo import ZoneInfo

from tractor_rpi.telemetry_ipc import SOCKET_PATH, MAX_FRAME, encode
from tractor_rpi.field_telemetry_schema import CSV_COLUMNS, build_row

CHANNELS = {"imu", "encoders", "gnss", "pose", "events", "status", "router"}


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


class Evidence:
    """Single-thread file owner; only closed segments receive final hashes."""

    def __init__(self, root, zone="America/New_York", min_free=256 * 1024 * 1024):
        self.root = Path(root)
        self.zone = ZoneInfo(zone) if isinstance(zone, str) else zone
        self.min_free = min_free
        self.session = uuid.uuid4().hex
        boot_file = Path("/proc/sys/kernel/random/boot_id")
        self.boot_id = boot_file.read_text().strip() if boot_file.exists() else "unavailable"
        self.rings = {name: deque(maxlen=4096) for name in CHANNELS}
        self.counts = defaultdict(int)
        self.sequences = {}
        self.losses = 0
        self.started = time.monotonic()
        self.segment_started = None
        self.handles = {}
        self.directory = None
        self.error = None
        self.last_flush = self.started
        self.last_write = None
        self.last_status = None
        self.idle_start = None
        self.marker_sent = False
        self.latest = {"gnss": {}, "status": {}, "router": {}}
        self.csv_handle = None
        self.hasher = ThreadPoolExecutor(max_workers=1, thread_name_prefix="closed-segment-hashes")
        self.hash_jobs = []
        self.commit = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                                     text=True, timeout=5).stdout.strip()
        self.root.mkdir(parents=True, exist_ok=True)
        incomplete = []
        for prior in sorted(self.root.glob("*/telemetry_*/metadata.json")):
            try:
                if json.loads(prior.read_text())["state"] == "RECORDING":
                    incomplete.append(str(prior.parent))
            except (OSError, ValueError, KeyError):
                incomplete.append(str(prior.parent))
        self.rotate()
        self.append("events", dict(event="recorder_start", session=self.session, boot_id=self.boot_id,
                                   prior_incomplete_segments=incomplete,
                                   note="Prior raw files were preserved, not repaired or deleted"))

    def day(self):
        return dt.datetime.now(self.zone).strftime("%Y-%m-%d")

    def rotate(self):
        if self.handles:
            self.close("ROTATED")
        if shutil.disk_usage(self.root).free < self.min_free:
            raise OSError("Insufficient free space; original evidence was not deleted")
        self.directory = self.root / self.day() / ("telemetry_" + dt.datetime.now(dt.timezone.utc).strftime("%H%M%S_%fZ"))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.segment_started = time.monotonic()
        self.segment_counts = defaultdict(int)
        self.handles = {name: (self.directory / (name + ".jsonl")).open("x", encoding="utf-8") for name in CHANNELS}
        self.csv_handle = (self.directory / "field.csv").open("x", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_handle, CSV_COLUMNS)
        self.csv_writer.writeheader()
        self.save_metadata("RECORDING")

    def save_metadata(self, state):
        data = dict(schema=1, state=state, session=self.session, boot_id=self.boot_id, commit=self.commit,
                    timezone=str(self.zone), updated_utc=utc_now(), counts=dict(self.segment_counts),
                    losses=self.losses, error=self.error, valid_for_control=False)
        target = self.directory / "metadata.json"
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        temporary.replace(target)

    def append(self, channel, payload, mono=None, source=None):
        if channel not in CHANNELS:
            raise ValueError("Unknown telemetry channel")
        now = time.monotonic()
        if self.day() != self.directory.parent.name or now - self.segment_started >= 900:
            self.rotate()
        record = dict(utc=utc_now(), monotonic_s=now if mono is None else mono,
                      received_mono=now, payload=payload, source=source)
        text = json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n"
        self.handles[channel].write(text)
        self.rings[channel].append(record)
        self.counts[channel] += 1
        self.segment_counts[channel] += 1
        self.last_write = now
        if channel in self.latest:
            self.latest[channel] = payload
        if channel == "status":
            self.csv_writer.writerow(build_row(self.latest["gnss"], payload, self.latest["router"],
                                                record["utc"], now - self.started))
        if channel == "gnss":
            speed = payload.get("speed_mps")
            stationary = (payload.get("fix_quality") == "RTK Fixed" and isinstance(speed, (int, float))
                          and 0 <= speed <= .05)
            if not stationary:
                self.idle_start = None
                self.marker_sent = False
            else:
                self.idle_start = self.idle_start or now
                if now - self.idle_start >= 10 and not self.marker_sent:
                    self.marker_sent = True
                    self.append("events", dict(event="candidate_ten_second_stop", start_mono=self.idle_start,
                                               note="Inferred stop, not confirmed operator intent"))

    def ingest(self, message):
        if message.get("schema") != 1 or message.get("channel") not in CHANNELS:
            raise ValueError("Unsupported record schema/channel")
        session = message["session"]
        sequence = message["sequence"]
        if not isinstance(session, str) or len(session) > 64 or type(sequence) is not int or sequence < 0:
            raise ValueError("Invalid producer identity/sequence")
        previous = self.sequences.get(session, -1)
        if session not in self.sequences and len(self.sequences) >= 128:
            raise ValueError("Too many producer sessions in one recorder process")
        if sequence <= previous:
            return dict(ack=sequence)
        if sequence != previous + 1:
            self.losses += sequence - previous - 1
            self.append("events", dict(event="producer_gap", session=session, previous=previous, sequence=sequence))
        self.append(message["channel"], message["payload"], source=dict(session=session, sequence=sequence,
                    monotonic_s=message["source_mono"]))
        self.sequences[session] = sequence
        return dict(ack=sequence)

    def window(self, since, channel):
        if channel not in CHANNELS or not isinstance(since, (float, int)):
            raise ValueError("Invalid window")
        now = time.monotonic()
        if not now - 10 <= since <= now:
            raise ValueError("Window must begin within last ten seconds")
        records = [r for r in self.rings[channel] if r["received_mono"] >= since]
        return dict(schema=1, session=self.session, records=records, health=self.health())

    def health(self):
        return dict(session=self.session, now=time.monotonic(), directory=str(self.directory),
                    error=self.error, losses=self.losses, counts=dict(self.counts),
                    last_write=self.last_write, free_bytes=shutil.disk_usage(self.root).free,
                    min_free_bytes=self.min_free)

    def flush(self):
        if time.monotonic() - self.last_flush >= 1:
            if shutil.disk_usage(self.root).free < self.min_free:
                raise OSError("Low storage space; no original logs deleted")
            for handle in self.handles.values():
                handle.flush()
            self.csv_handle.flush()
            self.save_metadata("RECORDING")
            self.last_flush = time.monotonic()

    def close(self, state="STOPPED"):
        for handle in self.handles.values():
            handle.close()
        self.handles = {}
        if self.csv_handle:
            self.csv_handle.close()
        self.save_metadata(state)
        self.hash_jobs = [job for job in self.hash_jobs if not job.done() or job.exception() is not None]
        for job in self.hash_jobs:
            if job.done():
                job.result()
        if len(self.hash_jobs) >= 4:
            raise RuntimeError("Closed-segment checksum worker fell behind")
        self.hash_jobs.append(self.hasher.submit(hash_segment, self.directory))


def hash_segment(directory):
        lines = []
        for path in sorted(directory.iterdir()):
            if path.is_file() and path.name != "SHA256SUMS":
                digest = hashlib.sha256()
                with path.open("rb") as handle:
                    for block in iter(lambda: handle.read(1048576), b""):
                        digest.update(block)
                lines.append(digest.hexdigest() + "  " + path.name)
        (directory / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")


class Server:
    def __init__(self, evidence, path=SOCKET_PATH):
        self.evidence = evidence
        self.path = Path(path)
        self.selector = selectors.DefaultSelector()
        self.buffers = {}
        self.outgoing = {}
        self.last_sequence = None
        self.stopping = False
        self.bound = False

    def listen(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Never unlink another recorder's live socket. RuntimeDirectory is private.
        if self.path.exists():
            raise RuntimeError("Recorder socket already exists; inspect owner before recovery")
        local = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        local.bind(str(self.path))
        self.bound = True
        self.path.chmod(0o660)
        local.listen(8)
        local.setblocking(False)
        self.selector.register(local, selectors.EVENT_READ, "accept")
        for port, channel in ((6009, "gnss"), (6003, "status")):
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if port == 6003:
                udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp.bind(("", port))
            udp.setblocking(False)
            self.selector.register(udp, selectors.EVENT_READ, channel)
        router = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        router.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        router.bind(("", 6005))
        router.listen(2)
        router.setblocking(False)
        self.selector.register(router, selectors.EVENT_READ, "router_accept")

    def disconnect(self, connection):
        self.selector.unregister(connection)
        self.buffers.pop(connection, None)
        self.outgoing.pop(connection, None)
        connection.close()

    def respond(self, connection, response=None):
        pending = self.outgoing.setdefault(connection, bytearray())
        if response is not None:
            pending.extend(encode(response))
        if len(pending) > MAX_FRAME:
            raise ValueError("Slow telemetry client exceeded response bound")
        if pending:
            try:
                sent = connection.send(pending[:65536])
                del pending[:sent]
            except BlockingIOError:
                pass
        self.selector.modify(connection, selectors.EVENT_READ | (selectors.EVENT_WRITE if pending else 0), "client")

    def step(self):
        for key, mask in self.selector.select(.1):
            sock, kind = key.fileobj, key.data
            if kind == "client" and mask & selectors.EVENT_WRITE:
                try:
                    self.respond(sock)
                except (OSError, ValueError):
                    self.disconnect(sock)
                    continue
                if not mask & selectors.EVENT_READ:
                    continue
            if kind in ("accept", "router_accept"):
                connection, _ = sock.accept()
                if len(self.buffers) >= 12:
                    connection.close()
                    continue
                connection.setblocking(False)
                self.buffers[connection] = bytearray()
                self.selector.register(connection, selectors.EVENT_READ, "client" if kind == "accept" else "router_client")
            elif kind in ("gnss", "status"):
                data, _ = sock.recvfrom(65535)
                try:
                    message = json.loads(data)
                    if not isinstance(message, dict):
                        raise ValueError("not an object")
                    if kind == "status":
                        sequence = message.get("steering", {}).get("sequence")
                        if sequence == self.last_sequence:
                            continue
                        self.last_sequence = sequence
                    self.evidence.append(kind, message)
                except (ValueError, TypeError) as exc:
                    self.evidence.append("events", dict(event="malformed_udp", channel=kind, error=str(exc)))
            else:
                try:
                    block = sock.recv(65536)
                    if not block:
                        if kind == "router_client" and self.buffers[sock]:
                            try:
                                self.evidence.append("router", json.loads(self.buffers[sock]))
                            except ValueError:
                                self.evidence.append("events", dict(event="malformed_router"))
                        self.disconnect(sock)
                        continue
                    buffer = self.buffers[sock]
                    buffer.extend(block)
                    if len(buffer) > MAX_FRAME:
                        raise ValueError("Oversized client frame")
                    # Bound each client to avoid starving UDP capture.
                    for _ in range(32):
                        if b"\n" not in buffer:
                            break
                        line, _, rest = buffer.partition(b"\n")
                        buffer[:] = rest
                        message = json.loads(line)
                        if kind == "router_client":
                            self.evidence.append("router", message)
                            continue
                        op = message.get("op")
                        if op == "record":
                            response = self.evidence.ingest(message)
                        elif op == "window":
                            response = self.evidence.window(message["since"], message["channel"])
                        elif op == "health":
                            response = self.evidence.health()
                        elif op == "confirm_engine_off":
                            self.evidence.append("events", dict(event="engine_off_confirmation",
                                                 note="Explicit operator assertion, not automatic detection"))
                            response = dict(accepted=True)
                        elif op == "seal":
                            closed = str(self.evidence.directory)
                            self.evidence.rotate()
                            response = dict(closed_directory=closed, recording_continues=True)
                        else:
                            raise ValueError("Unknown operation")
                        self.respond(sock, response)
                except (ConnectionError, socket.timeout, ValueError, KeyError, TypeError):
                    self.disconnect(sock)
        self.evidence.flush()

    def close(self):
        for key in list(self.selector.get_map().values()):
            key.fileobj.close()
        self.selector.close()
        if self.bound and self.path.exists():
            self.path.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/home/al/field_logs"))
    parser.add_argument("--socket", default=SOCKET_PATH)
    args = parser.parse_args(argv)
    evidence = Evidence(args.root)
    server = Server(evidence, args.socket)
    signal.signal(signal.SIGTERM, lambda *_: setattr(server, "stopping", True))
    signal.signal(signal.SIGINT, lambda *_: setattr(server, "stopping", True))
    state = "STOPPED"
    try:
        server.listen()
        while not server.stopping:
            server.step()
    except Exception as exc:
        evidence.error = str(exc)
        state = "FAILED"
        print(f"Telemetry recording failed: {exc}", flush=True)
    finally:
        server.close()
        evidence.close(state)
        evidence.hasher.shutdown(wait=True)
        for job in evidence.hash_jobs:
            job.result()
    return 0 if state == "STOPPED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
