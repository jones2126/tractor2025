#!/usr/bin/env python3
"""DRAFT - UNTESTED IN THE FIELD. Observation only; never sends robot commands."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import socket
import statistics
import struct
import subprocess
import time

GYRO_SCALE = 0.00875  # degrees/second per count, +/-250 dps
ACCEL_SCALE = 0.000061  # g per count, +/-2 g
GNSS_FIELDS = (
    "lat", "lon", "fix_quality", "speed_mps", "speed_timestamp",
    "heading_deg", "headValid", "carrier", "relpos_length_m",
    "relpos_heading_accuracy_deg", "expectedErrDeg", "relposned_count",
    "relposned_timestamp", "relpos_valid", "relpos_gnss_fix_ok",
    "relpos_ref_pos_miss", "relpos_ref_obs_miss", "timestamp", "fatal_error",
)
FIELDS = (
    "utc", "monotonic_s", "sample", "phase", "dt_s", "timing_gap",
    "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
    "accel_x_raw", "accel_y_raw", "accel_z_raw",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_corrected_dps", "gyro_y_corrected_dps", "gyro_z_corrected_dps",
    "relative_yaw_ccw_deg", "yaw_segment", "imu_compass_once_anchored_deg",
    "imu_minus_gnss_deg", "gnss_packet_age_s", "gnss_heading_age_s",
    "gnss_heading_usable", "gnss_socket_available",
) + tuple("gnss_" + key for key in GNSS_FIELDS)


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="microseconds")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def wrap180(value):
    return (value + 180.0) % 360.0 - 180.0


def compass_from_yaw(anchor_heading, yaw_since_anchor):
    return (anchor_heading - yaw_since_anchor) % 360.0


def decode_sample(block):
    if len(block) != 12:
        raise ValueError("IMU sample must contain exactly 12 bytes")
    raw = struct.unpack("<6h", bytes(block))
    return raw, tuple(x * GYRO_SCALE for x in raw[:3]), tuple(x * ACCEL_SCALE for x in raw[3:])


def bias_statistics(samples):
    if len(samples) < 2:
        raise ValueError("not enough stationary samples")
    axes = list(zip(*samples))
    if len(axes) != 3 or any(not finite(v) for axis in axes for v in axis):
        raise ValueError("invalid gyro calibration samples")
    return [statistics.mean(a) for a in axes], [statistics.stdev(a) for a in axes]


def parse_gnss(payload):
    try:
        message = json.loads(payload)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(message, dict) or not {"lat", "lon", "headValid", "relposned_timestamp"}.issubset(message):
        return None
    return {key: message.get(key) for key in GNSS_FIELDS}


def timestamp_age(timestamp, now_epoch):
    try:
        parsed = dt.datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            return None
        age = now_epoch - parsed.timestamp()
        return age if math.isfinite(age) and age >= -0.1 else None
    except (AttributeError, ValueError, TypeError, OverflowError):
        return None


def heading_usable(message, packet_age, heading_age):
    return bool(
        message and packet_age is not None and 0 <= packet_age <= 1.0
        and heading_age is not None and -0.1 <= heading_age <= 1.0
        and message.get("headValid") is True and message.get("carrier") == "fixed"
        and message.get("fix_quality") == "RTK Fixed"
        and message.get("relpos_valid") is True
        and message.get("relpos_gnss_fix_ok") is True
        and message.get("relpos_ref_pos_miss") is False
        and message.get("relpos_ref_obs_miss") is False
        and message.get("fatal_error") is False
        and finite(message.get("heading_deg")) and 0 <= message["heading_deg"] < 360
        and finite(message.get("relpos_heading_accuracy_deg"))
        and 0 <= message["relpos_heading_accuracy_deg"] <= 1.0
    )


class GnssReceiver:
    """Exclusive receiver: deliberately does not set SO_REUSEADDR/REUSEPORT."""
    def __init__(self):
        self.sock = None
        self.message = {}
        self.received_at = None
        self.error = None

    def open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", 6009))
            sock.setblocking(False)
        except OSError as exc:
            sock.close()
            self.error = str(exc)
            print(f"WARNING: GPS UDP 6009 unavailable: {exc}. IMU-only logging continues.", flush=True)
            return
        self.sock = sock

    def poll(self, raw_file):
        if self.sock is None:
            return
        # Bound work so an unexpected datagram flood cannot starve IMU reads.
        for _ in range(32):
            try:
                payload, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError as exc:
                self.error = str(exc)
                self.close()
                print(f"WARNING: GPS socket lost: {exc}; continuing IMU-only.", flush=True)
                break
            received = time.monotonic()
            parsed = parse_gnss(payload)
            if parsed is not None:
                self.message, self.received_at = parsed, received
                raw_file.write(json.dumps({"utc": utc_now(), "monotonic_s": received,
                                           "message": json.loads(payload)}) + "\n")

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None


class Sensor:
    def __init__(self, bus):
        self.bus = bus
        self.address = 0x6B

    def configure(self):
        identity = self.bus.read_byte_data(self.address, 0x0F)
        if identity != 0x6B:
            raise RuntimeError(f"WHO_AM_I expected 0x6b, got {identity:#04x}")
        # BDU prevents mixed-byte samples; IF_INC permits the 12-byte burst.
        for register, value in ((0x12, 0x44), (0x10, 0x40), (0x11, 0x40)):
            self.bus.write_byte_data(self.address, register, value)
            if self.bus.read_byte_data(self.address, register) != value:
                raise RuntimeError(f"IMU configuration readback failed at {register:#04x}")

    def read(self):
        if self.bus.read_byte_data(self.address, 0x1E) & 0x03 != 0x03:
            return None
        return decode_sample(self.bus.read_i2c_block_data(self.address, 0x22, 12))


class YawTracker:
    def __init__(self):
        self.yaw = 0.0
        self.segment = 0
        self.previous_rate = None
        self.anchor_heading = None
        self.anchor_yaw = None

    def update(self, rate, delta, heading=None):
        # Never bridge a missed interval by pretending the last rate persisted.
        gap = delta > 0.05
        if gap:
            self.segment += 1
            self.yaw = 0.0
            self.anchor_heading = self.anchor_yaw = None
            self.previous_rate = None
        if self.previous_rate is not None:
            self.yaw += 0.5 * (self.previous_rate + rate) * delta
        self.previous_rate = rate
        if self.anchor_heading is None and heading is not None:
            self.anchor_heading, self.anchor_yaw = heading, self.yaw
        compass = None if self.anchor_heading is None else compass_from_yaw(
            self.anchor_heading, self.yaw - self.anchor_yaw)
        return gap, compass


def repository_state():
    root = Path(__file__).resolve().parents[3]
    def git(*args):
        return subprocess.check_output(["git", "-C", str(root), *args], text=True, timeout=5).strip()
    return {"repository": str(root), "commit": git("rev-parse", "HEAD"),
            "working_tree": git("status", "--porcelain")}


def write_metadata(path, metadata):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def hash_outputs(directory):
    rows = []
    for path in sorted(directory.iterdir()):
        if path.is_file() and path.name != "SHA256SUMS":
            digest = hashlib.sha256()
            with path.open("rb") as handle:
                for block in iter(lambda: handle.read(1024 * 1024), b""):
                    digest.update(block)
            rows.append(f"{digest.hexdigest()}  {path.name}")
    (directory / "SHA256SUMS").write_text("\n".join(rows) + "\n", encoding="utf-8")


def run(args):
    if not args.stationary:
        raise ValueError("Keep the tractor stationary, then supply --stationary to confirm calibration conditions.")
    repo = repository_state()
    if repo["working_tree"]:
        raise ValueError("Repository has uncommitted changes; use a clean committed revision for field evidence.")
    from smbus2 import SMBus  # Hardware dependency is deliberately lazy.

    directory = args.output_dir.expanduser() / dt.datetime.now(dt.timezone.utc).strftime("imu_%Y%m%dT%H%M%S_%fZ")
    directory.mkdir(parents=True, exist_ok=False)
    meta_path = directory / "metadata.json"
    metadata = {"schema": 1, "status": "STARTING", "started_utc": utc_now(), **repo,
                "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "axes": "X forward, Y left, Z up; positive yaw CCW (expected, verify from data)",
                "imu": {"bus": 1, "address": "0x6b", "odr_hz": 104,
                        "accel_range_g": 2, "gyro_range_dps": 250, "ctrl3_c": "0x44",
                        "gyro_dps_per_count": GYRO_SCALE, "accel_g_per_count": ACCEL_SCALE},
                "calibration_seconds": args.calibration_seconds,
                "yaw_policy": "trapezoidal; one GNSS anchor per segment; gaps >50 ms start a new segment",
                "note": "Observation only. Host timestamps; no hardware FIFO. Missed samples cannot be recovered."}
    write_metadata(meta_path, metadata)
    print(f"Evidence directory: {directory}", flush=True)
    receiver, bus = GnssReceiver(), None
    sample_count = 0
    bias, noise, calibration, accel_calibration = None, None, [], []
    tracker = YawTracker()
    outcome = 1
    try:
        bus = SMBus(1)
        sensor = Sensor(bus)
        sensor.configure()
        receiver.open()
        metadata["gnss_initial_error"] = receiver.error
        # Discard startup settling samples for one second.
        settle_until = time.monotonic() + 1.0
        calibration_start = None
        previous = None
        last_sample = time.monotonic()
        last_print = last_flush = 0.0
        print(f"Keep stationary with engine OFF. Calibrating gyro for {args.calibration_seconds:g} seconds; wait for READY.", flush=True)
        with (directory / "samples.csv").open("x", newline="", encoding="utf-8") as csv_file, \
             (directory / "gnss.jsonl").open("x", encoding="utf-8") as gnss_file:
            writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
            writer.writeheader()
            while True:
                receiver.poll(gnss_file)
                reading = sensor.read()
                now = time.monotonic()
                if reading is None:
                    if now - last_sample > 1.0:
                        raise RuntimeError("No fresh IMU sample for one second; logging stopped.")
                    time.sleep(0.001)
                    continue
                last_sample = now
                if now < settle_until:
                    continue
                raw, gyro, accel = reading
                delta = 0.0 if previous is None else now - previous
                previous = now
                if calibration_start is None:
                    calibration_start = now
                phase = "calibration" if bias is None else "observation"
                if bias is None:
                    calibration.append(gyro)
                    accel_calibration.append(accel)
                    if now - calibration_start >= args.calibration_seconds:
                        bias, noise = bias_statistics(calibration)
                        accel_noise = [statistics.stdev(a) for a in zip(*accel_calibration)]
                        accel_magnitude = statistics.mean(math.sqrt(sum(v*v for v in a)) for a in accel_calibration)
                        metadata["calibration"] = {"samples": len(calibration), "bias_dps": bias,
                            "stddev_dps": noise, "accel_stddev_g": accel_noise,
                            "accel_mean_magnitude_g": accel_magnitude}
                        if (len(calibration) < args.calibration_seconds * 80 or max(noise) > 0.5
                                or max(abs(v) for v in bias) > 5 or max(accel_noise) > 0.03
                                or not 0.85 <= accel_magnitude <= 1.15):
                            raise RuntimeError("Stationary calibration failed quality checks; keep engine off and retry.")
                        metadata["status"] = "RECORDING"
                        write_metadata(meta_path, metadata)
                        print(f"READY — observation logging. Bias dps={bias}; noise dps={noise}", flush=True)
                packet_age = None if receiver.received_at is None else now - receiver.received_at
                heading_age = timestamp_age(receiver.message.get("relposned_timestamp"), time.time())
                usable = heading_usable(receiver.message, packet_age, heading_age)
                row = {"utc": utc_now(), "monotonic_s": now, "sample": sample_count,
                       "phase": phase, "dt_s": delta, "gnss_packet_age_s": packet_age,
                       "gnss_heading_age_s": heading_age, "gnss_heading_usable": int(usable),
                       "gnss_socket_available": int(receiver.sock is not None)}
                for names, values in ((FIELDS[6:12], raw), (FIELDS[12:15], gyro), (FIELDS[15:18], accel)):
                    row.update(zip(names, values))
                row.update({"gnss_" + key: receiver.message.get(key) for key in GNSS_FIELDS})
                if phase == "observation":
                    corrected = [v - b for v, b in zip(gyro, bias)]
                    row.update(zip(FIELDS[18:21], corrected))
                    heading = receiver.message.get("heading_deg") if usable else None
                    gap, compass = tracker.update(corrected[2], delta, heading)
                    row.update(timing_gap=int(gap), relative_yaw_ccw_deg=tracker.yaw,
                               yaw_segment=tracker.segment, imu_compass_once_anchored_deg=compass,
                               imu_minus_gnss_deg=wrap180(compass - heading) if compass is not None and heading is not None else None)
                writer.writerow(row)
                sample_count += 1
                if now - last_flush >= 1.0:
                    csv_file.flush()
                    gnss_file.flush()
                    last_flush = now
                if now - last_print >= 0.2:
                    print(f"{phase} samples={sample_count} gyroZ={gyro[2]:+.3f} dps "
                          f"yaw={tracker.yaw:+.2f} GPS={'valid' if usable else 'missing/stale/invalid'}", flush=True)
                    last_print = now
                if args.duration and now - calibration_start >= args.calibration_seconds + args.duration:
                    metadata["status"] = "COMPLETED"
                    outcome = 0
                    break
    except KeyboardInterrupt:
        metadata["status"] = "STOPPED" if bias is not None else "CALIBRATION_INTERRUPTED"
        outcome = 0 if bias is not None else 1
    except Exception as exc:
        metadata["status"], metadata["error"] = "FAILED", str(exc)
        print(f"LOGGER FAILED: {exc}", flush=True)
    finally:
        receiver.close()
        if bus is not None:
            bus.close()
        metadata.update(ended_utc=utc_now(), sample_rows=sample_count,
                        yaw_segments=tracker.segment + 1, gnss_final_error=receiver.error)
        write_metadata(meta_path, metadata)
        hash_outputs(directory)
        print(f"Saved {metadata['status']}: {directory} (includes SHA256SUMS)", flush=True)
    return outcome


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stationary", action="store_true", help="Confirm engine-off stationary startup calibration")
    parser.add_argument("--output-dir", type=Path, default=Path("/home/al/field_logs/imu"))
    parser.add_argument("--calibration-seconds", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0, help="Observation seconds after calibration; zero runs until Ctrl-C")
    args = parser.parse_args(argv)
    if not finite(args.calibration_seconds) or args.calibration_seconds < 10:
        parser.error("calibration must be at least 10 finite seconds")
    if not finite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and nonnegative")
    try:
        return run(args)
    except (ValueError, OSError, ImportError, subprocess.SubprocessError) as exc:
        print(f"LOGGER NOT STARTED: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
