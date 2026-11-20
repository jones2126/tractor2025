# Production-folder placement reviewed 2026-11-20; runtime logic unchanged.
"""Bounded local telemetry transport; no network ports or hardware ownership."""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
import uuid

SOCKET_PATH = "/run/tractor-telemetry/recorder.sock"
MAX_FRAME = 2 * 1024 * 1024


def encode(message):
    data = json.dumps(message, allow_nan=False, separators=(",", ":")).encode() + b"\n"
    if len(data) > MAX_FRAME:
        raise ValueError("Telemetry frame too large")
    return data


def receive(connection, buffer):
    while b"\n" not in buffer:
        block = connection.recv(65536)
        if not block:
            raise ConnectionError("Telemetry peer disconnected")
        buffer.extend(block)
        if len(buffer) > MAX_FRAME:
            raise ValueError("Telemetry frame too large")
    line, _, rest = buffer.partition(b"\n")
    buffer[:] = rest
    return json.loads(line)


def request(message, path=SOCKET_PATH, timeout=3):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(path))
        connection.sendall(encode(message))
        result = receive(connection, bytearray())
        if result.get("error"):
            raise RuntimeError(result["error"])
        return result


class Publisher:
    """Ordered records with receiver ACKs; uncertain retransmissions deduplicated."""

    def __init__(self, stop, path=SOCKET_PATH):
        self.stop = stop
        self.path = path
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.lock = threading.Lock()
        self.queue = queue.Queue(maxsize=8192)
        self.error = None
        self.finishing = threading.Event()
        self.thread = threading.Thread(target=self.run, name="telemetry-publisher", daemon=True)

    def emit(self, channel, payload):
        if channel == "console":
            print(payload, flush=True)
            return
        with self.lock:
            record = dict(op="record", schema=1, session=self.session, sequence=self.sequence,
                          channel=channel, source_mono=time.monotonic(), payload=payload)
            # Serialize before queueing so later model updates cannot alter evidence.
            data = encode(record)
            try:
                self.queue.put_nowait((self.sequence, data))
                self.sequence += 1
            except queue.Full:
                self.error = "Telemetry transport overflow"
                self.stop.set()

    def run(self):
        connection = None
        pending = None
        buffer = bytearray()
        disconnected = None
        try:
            while not self.finishing.is_set() or pending is not None or not self.queue.empty():
                if pending is None:
                    try:
                        pending = self.queue.get(timeout=.1)
                    except queue.Empty:
                        continue
                try:
                    if connection is None:
                        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        connection.settimeout(1)
                        connection.connect(str(self.path))
                        buffer.clear()
                    connection.sendall(pending[1])
                    reply = receive(connection, buffer)
                    if reply.get("ack") != pending[0] or reply.get("error"):
                        raise RuntimeError(f"Recorder rejected record: {reply}")
                    pending = None
                    disconnected = None
                except OSError:
                    if connection:
                        connection.close()
                    connection = None
                    disconnected = disconnected or time.monotonic()
                    if time.monotonic() - disconnected > 10 or self.finishing.is_set():
                        raise RuntimeError("Recorder unavailable; telemetry transport incomplete")
                    time.sleep(.1)
        except Exception as exc:
            self.error = str(exc)
            self.stop.set()
        finally:
            if connection:
                connection.close()
