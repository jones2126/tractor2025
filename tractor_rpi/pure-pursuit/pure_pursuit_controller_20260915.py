"""
pure_pursuit_controller_20260915.py -- Pure Pursuit with guarded forward recovery.

This is a field-review successor to the proven 20260714 controller.  It adds:

* monotonic along-path progress and an interpolated lookahead target;
* handheld mode monitoring on Teensy status UDP 6003;
* frozen progress while the handheld is not in AUTO;
* guarded forward-only reacquisition after Manual/Pause or loss of RTK Fixed;
* refusal to resume when the closest forward path choice is ambiguous;
* bounded, phase-locked recovery so a nearby later ring cannot be selected;
* fixed-carrier, baseline-length, heading-accuracy, and stable-time gates.

The older controller remains unchanged for comparison and rollback.

Lineage: pure_pursuit_grok_20251112.py -> pure_pursuit_live_20260713.py -> this file.

Change log vs pure_pursuit_live_20260713.py:

  NEW   Per-cycle CSV logger (PursuitLogger class). Writes one row every
        control cycle (20 Hz) to ~/repos/field-testing-data/ alongside the
        field_test_logger CSV. Captures all internal pure pursuit state:
        actual lat/lon/heading, target waypoint lat/lon, lateral error (yt),
        cross-track error, steering angle, normalized steer command, speed,
        waypoint index, lookahead distance, fix quality, goal_reached flag.
        Enabled by default in live mode; disable with --no-pursuit-log.
        CSV filename: pursuit_log_YYYYMMDD_HHMMSS.csv
  CHANGED linear_x is now commanded in METERS PER SECOND, POSITIVE = FORWARD.
  REMOVED speed_to_linear_x() and the max-speed-as-calibration-scale concept.
  NEW   clamp_speed() -- safety cap only, default 1.5 m/s.
  CHANGED gps_offset_x/y set to 0.0 (antenna is at base_link on tractor01).

cmd_vel JSON sent on UDP 6004:
    {"linear_x": <m/s, POSITIVE = FORWARD, 0 = stop>,
     "angular_z": <-1..+1, +1 = full left, 0 = center>,
     "timestamp": <epoch>}

CSV columns written by PursuitLogger (one row per 20 Hz cycle):
    timestamp, elapsed_s, cycle,
    lat, lon, heading_compass_deg, fix_quality, head_valid, gps_age_s,
    pos_x_m, pos_y_m,
    waypoint_idx, waypoints_total, goal_reached,
    target_lat, target_lon, target_x_m, target_y_m, lookahead_dist_m,
    yt_m, cross_track_err_m, alpha_deg, delta_deg,
    steer_normalized, speed_cmd_mps,
    driving, wait_reason
"""

import csv
import math
import os
import socket
import json
import time
import argparse
import signal
import sys
import threading
from datetime import datetime

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GPS_UDP_PORT = 6002
TEENSY_STATUS_UDP_PORT = 6003
CMD_VEL_UDP_PORT = 6004
GPS_STALE_TIMEOUT_S = 0.5
STATUS_STALE_TIMEOUT_S = 0.75
DEFAULT_MAX_SPEED_MPS = 1.5
LOG_DIR = os.path.expanduser("~/repos/field-testing-data")

_FIX_RANK = {
    None: 0, "Unknown": 0, "Invalid": 0,
    "GPS Fix": 1, "DGPS": 1,
    "RTK Float": 2,
    "RTK Fixed": 3,
}

# Column name, description (description written as row 2 for self-documentation)
CSV_COLUMNS = [
    ("timestamp",           "Unix epoch seconds"),
    ("elapsed_s",           "Seconds since controller started"),
    ("cycle",               "Control cycle counter at 20 Hz"),
    ("lat",                 "Actual latitude from F9P (decimal degrees)"),
    ("lon",                 "Actual longitude from F9P (decimal degrees)"),
    ("heading_compass_deg", "Actual heading compass degrees from true north"),
    ("fix_quality",         "RTK Fixed / RTK Float / GPS Fix / etc."),
    ("head_valid",          "True if F9P dual-antenna heading is valid"),
    ("heading_carrier",     "RELPOSNED carrier solution: fixed / float / none"),
    ("relpos_length_m",     "Measured moving-baseline length meters"),
    ("heading_accuracy_deg", "RELPOSNED heading accuracy estimate degrees"),
    ("relpos_gnss_fix_ok",  "RELPOSNED gnssFixOK flag"),
    ("relpos_diff_solution", "RELPOSNED diffSoln flag"),
    ("relpos_valid",        "RELPOSNED relPosValid flag"),
    ("relpos_moving",       "RELPOSNED isMoving flag"),
    ("relpos_ref_pos_miss", "RELPOSNED refPosMiss flag"),
    ("relpos_ref_obs_miss", "RELPOSNED refObsMiss flag"),
    ("relpos_normalized",   "RELPOSNED relPosNormalized flag"),
    ("relposned_count",     "Cumulative decoded RELPOSNED frame count"),
    ("relposned_itow_ms",   "RELPOSNED GNSS time of week milliseconds"),
    ("heading_numSV_used",  "Satellites used by heading receiver"),
    ("heading_cno_mean_dbhz", "Heading receiver mean carrier-to-noise dB-Hz"),
    ("gps_age_s",           "Age of GPS packet seconds"),
    ("pos_x_m",             "Actual position local x meters east of origin"),
    ("pos_y_m",             "Actual position local y meters north of origin"),
    ("waypoint_idx",        "Index of active lookahead waypoint"),
    ("waypoints_total",     "Total waypoints in mission"),
    ("goal_reached",        "True when mission end-of-path declared"),
    ("target_lat",          "Lookahead waypoint latitude decimal degrees"),
    ("target_lon",          "Lookahead waypoint longitude decimal degrees"),
    ("target_x_m",          "Lookahead waypoint local x meters"),
    ("target_y_m",          "Lookahead waypoint local y meters"),
    ("lookahead_dist_m",    "Lookahead distance ld in use this cycle meters"),
    ("yt_m",                "Lateral error signed: + = target left of heading - = right meters"),
    ("cross_track_err_m",   "Absolute cross-track error abs(yt) meters"),
    ("alpha_deg",           "Heading error angle alpha = asin(yt/ld) degrees"),
    ("delta_deg",           "Steering angle delta: + = turn left - = turn right degrees"),
    ("steer_normalized",    "Normalized steer command sent: +1.0=full left -1.0=full right"),
    ("speed_cmd_mps",       "Commanded speed m/s positive = forward"),
    ("actual_speed_mps",    "GPS ground speed m/s"),
    ("driving",             "True if cmd_vel sent this cycle False = WAIT state"),
    ("wait_reason",         "Reason not driving this cycle empty string if driving"),
    ("software_paused",     "True while the local mission dashboard requests Pause"),
    ("handheld_mode",       "Teensy steering mode: 0 Auto, 1 Manual, 2 Pause, 9 radio loss"),
    ("handheld_state",      "Teensy steering state string"),
    ("path_progress_m",     "Monotonic distance along mission path"),
    ("reacquire_state",     "TRACKING, REQUIRED, or BLOCKED"),
    ("reacquire_detail",    "Most recent forward-reacquisition decision"),
]

CSV_FIELDNAMES = [c[0] for c in CSV_COLUMNS]


def _fix_ok(fix_quality, min_fix):
    if min_fix is None:
        return True
    return _FIX_RANK.get(fix_quality, 0) >= _FIX_RANK.get(min_fix, 3)


# ---------------------------------------------------------------------------
# Per-cycle CSV logger
# ---------------------------------------------------------------------------

class PursuitLogger:
    """Writes one CSV row per control cycle (20 Hz) capturing all pure pursuit
    internal state for post-mission analysis.

    Output: ~/repos/field-testing-data/pursuit_log_YYYYMMDD_HHMMSS.csv
    Row 1: column names. Row 2: descriptions. Row 3+: data.
    At 20 Hz a 3-minute mission produces ~3600 data rows (~300 KB).
    Flushes every write so data survives a crash or power loss.
    """

    def __init__(self, log_dir=LOG_DIR):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"pursuit_log_{ts}.csv")
        self._f = open(self.path, 'w', newline='')
        self._writer = csv.DictWriter(self._f, fieldnames=CSV_FIELDNAMES)
        self._writer.writeheader()
        # Row 2: descriptions so the file is self-documenting in Excel/Calc
        self._writer.writerow({col: desc for col, desc in CSV_COLUMNS})
        self._f.flush()

    def write(self, row: dict):
        """Write one row. Missing keys default to empty string."""
        full_row = {k: '' for k in CSV_FIELDNAMES}
        full_row.update(row)
        self._writer.writerow(full_row)
        self._f.flush()

    def close(self):
        try:
            self._f.close()
        except OSError:
            pass
        print(f"Pursuit log closed: {self.path}")


# ---------------------------------------------------------------------------
# Live GPS listener
# ---------------------------------------------------------------------------

class GPSReceiver:
    """Background UDP listener for rtcm_server broadcast on port 6002.
    SO_REUSEPORT allows coexistence with field_test_logger on the same port.
    """

    def __init__(self, port=GPS_UDP_PORT, min_fix="RTK Fixed", require_head_valid=True,
                 require_carrier_fixed=True, baseline_min_m=0.80,
                 baseline_max_m=1.30, heading_accuracy_max_deg=1.0):
        self.port = port
        self.min_fix = min_fix
        self.require_head_valid = require_head_valid
        self.require_carrier_fixed = require_carrier_fixed
        self.baseline_min_m = baseline_min_m
        self.baseline_max_m = baseline_max_m
        self.heading_accuracy_max_deg = heading_accuracy_max_deg
        self._lock = threading.Lock()
        self._latest = None
        self._last_update = 0.0
        self._running = True

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._sock.bind(('', self.port))
        self._sock.settimeout(0.5)

        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                continue
            self._parse(msg)

    def _parse(self, msg):
        lat = msg.get('lat')
        lon = msg.get('lon')
        heading_deg = msg.get('heading_deg')
        if lat is None or lon is None or heading_deg is None:
            return
        heading_rad = math.radians(90.0 - heading_deg)
        with self._lock:
            self._latest = {
                'lat': lat,
                'lon': lon,
                'heading_rad': heading_rad,
                'heading_compass_deg': heading_deg,
                'speed_mps': msg.get('speed_mps'),
                'fix_quality': msg.get('fix_quality'),
                'headValid': bool(msg.get('headValid')),
                'carrier': msg.get('carrier'),
                'relpos_length_m': msg.get('relpos_length_m'),
                'heading_accuracy_deg': msg.get('relpos_heading_accuracy_deg'),
                'relpos_gnss_fix_ok': msg.get('relpos_gnss_fix_ok'),
                'relpos_diff_solution': msg.get('relpos_diff_solution'),
                'relpos_valid': msg.get('relpos_valid'),
                'relpos_moving': msg.get('relpos_moving'),
                'relpos_ref_pos_miss': msg.get('relpos_ref_pos_miss'),
                'relpos_ref_obs_miss': msg.get('relpos_ref_obs_miss'),
                'relpos_normalized': msg.get('relpos_normalized'),
                'relposned_count': msg.get('relposned_count'),
                'relposned_itow_ms': msg.get('relposned_itow_ms'),
                'heading_numSV_used': msg.get('heading_numSV_used'),
                'heading_cno_mean_dbhz': msg.get('heading_cno_mean_dbhz'),
                'fatal_error': bool(msg.get('fatal_error', False)),
                'fatal_base_reason': msg.get('fatal_base_reason'),
                'fatal_heading_reason': msg.get('fatal_heading_reason'),
            }
            self._last_update = time.time()

    def get_pose(self):
        with self._lock:
            if self._latest is None:
                return None
            pose = dict(self._latest)
            pose['age'] = time.time() - self._last_update
            return pose

    def is_drivable(self, pose):
        if pose is None or pose['age'] > GPS_STALE_TIMEOUT_S:
            return False, "stale or no GPS"
        if pose['fatal_error']:
            return False, (f"rtcm_server fatal_error "
                           f"(base={pose['fatal_base_reason']}, "
                           f"heading={pose['fatal_heading_reason']})")
        if not _fix_ok(pose['fix_quality'], self.min_fix):
            return False, f"fix_quality={pose['fix_quality']!r} below --min-fix {self.min_fix!r}"
        if self.require_head_valid and not pose['headValid']:
            return False, (
                "headValid=False "
                f"(carrier={pose.get('carrier')!r}, "
                f"fixOK={pose.get('relpos_gnss_fix_ok')!r}, "
                f"diff={pose.get('relpos_diff_solution')!r}, "
                f"relPosValid={pose.get('relpos_valid')!r}, "
                f"moving={pose.get('relpos_moving')!r}, "
                f"refPosMiss={pose.get('relpos_ref_pos_miss')!r}, "
                f"refObsMiss={pose.get('relpos_ref_obs_miss')!r}, "
                f"baseline={pose.get('relpos_length_m')!r} m, "
                f"accuracy={pose.get('heading_accuracy_deg')!r} deg, "
                f"headingSV={pose.get('heading_numSV_used')!r}, "
                f"headingCNO={pose.get('heading_cno_mean_dbhz')!r} dB-Hz)")
        if self.require_carrier_fixed and pose['carrier'] != 'fixed':
            return False, f"heading carrier={pose['carrier']!r}, expected 'fixed'"
        try:
            baseline_m = float(pose['relpos_length_m'])
        except (TypeError, ValueError):
            return False, "heading baseline length is absent"
        if (not math.isfinite(baseline_m)
                or not self.baseline_min_m <= baseline_m <= self.baseline_max_m):
            return False, (
                f"heading baseline={baseline_m:.3f} m outside "
                f"{self.baseline_min_m:.2f}-{self.baseline_max_m:.2f} m")
        try:
            heading_accuracy_deg = float(pose['heading_accuracy_deg'])
        except (TypeError, ValueError):
            return False, "heading accuracy is absent"
        if (not math.isfinite(heading_accuracy_deg)
                or heading_accuracy_deg > self.heading_accuracy_max_deg):
            return False, (
                f"heading accuracy={heading_accuracy_deg:.3f} deg exceeds "
                f"{self.heading_accuracy_max_deg:.2f} deg")
        return True, ""

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


class HandheldStatusReceiver:
    """Latest Teensy steering mode from the bridge broadcast on UDP 6003."""

    MODE_NAMES = {0: "AUTO", 1: "MANUAL", 2: "PAUSE", 9: "RADIO_LOSS"}

    def __init__(self, port=TEENSY_STATUS_UDP_PORT):
        self.port = port
        self._lock = threading.Lock()
        self._latest = None
        self._last_update = 0.0
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._sock.bind(("", port))
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                message = json.loads(data.decode("utf-8"))
                steering = message.get("steering", {})
                mode = int(steering.get("mode"))
            except (TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            with self._lock:
                self._latest = {
                    "mode": mode,
                    "mode_name": self.MODE_NAMES.get(mode, f"UNKNOWN_{mode}"),
                    "state": str(steering.get("state", "UNKNOWN")),
                }
                self._last_update = time.time()

    def get_status(self):
        with self._lock:
            if self._latest is None:
                return None
            status = dict(self._latest)
            status["age"] = time.time() - self._last_update
            return status

    def auto_ready(self):
        status = self.get_status()
        if status is None or status["age"] > STATUS_STALE_TIMEOUT_S:
            return False, "stale or no Teensy status", status
        if status["mode"] != 0:
            return False, f"handheld mode={status['mode_name']}", status
        return True, "", status

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


class MissionControlReceiver:
    """Local-only UDP Pause/Resume input for the mission dashboard."""

    def __init__(self, port):
        self.port = port
        self._paused = False
        self._lock = threading.Lock()
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('127.0.0.1', port))
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                command = json.loads(data.decode('utf-8')).get('command', '').lower()
            except (ValueError, UnicodeDecodeError):
                continue
            if command in ('pause', 'resume'):
                with self._lock:
                    self._paused = command == 'pause'
                print(f"[CONTROL] Software {command.upper()} requested")

    def is_paused(self):
        with self._lock:
            return self._paused

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Pure Pursuit controller
# ---------------------------------------------------------------------------

class PurePursuit:
    def __init__(self, wheelbase=1.27, max_steer=0.623, pos_tol=0.5,
                 target_ip='127.0.0.1', target_port=CMD_VEL_UDP_PORT,
                 rate_hz=20.0, max_speed_mps=DEFAULT_MAX_SPEED_MPS,
                 tracking_window_m=12.0, reacquire_distance_m=2.0,
                 reacquire_heading_deg=60.0, ambiguity_distance_m=0.35,
                 ambiguity_progress_m=8.0, reacquire_max_advance_m=30.0):
        self.L = wheelbase
        self.delta_max = max_steer
        self.pos_tol = pos_tol
        self.max_speed_mps = max_speed_mps
        self.gps_offset_x = 0.0
        self.gps_offset_y = 0.0

        self.path = []
        self.idx = 0
        self.goal_reached = True
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.cumulative_s = []
        self.progress_s = 0.0
        self.tracking_window_m = tracking_window_m
        self.reacquire_distance_m = reacquire_distance_m
        self.reacquire_heading_rad = math.radians(reacquire_heading_deg)
        self.ambiguity_distance_m = ambiguity_distance_m
        self.ambiguity_progress_m = ambiguity_progress_m
        self.reacquire_max_advance_m = reacquire_max_advance_m
        self.path_phases = []
        self.reacquire_state = "REQUIRED"
        self.reacquire_detail = "mission not started"

        self.target_ip = target_ip
        self.target_port = target_port
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.messages_sent = 0
        self.start_time = None
        self.last_stats_time = None
        self.last_stats_count = 0
        self.running = True

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_h = 0.0

        # Internal state set by compute_steering(), read by run_live() for logging
        self._last_yt = 0.0
        self._last_ld = 0.0
        self._last_delta = 0.0
        self._last_target_idx = 0
        self._last_target_xy = (0.0, 0.0)

    # -- path loading --------------------------------------------------------

    def load_path(self, filename):
        self.path = []
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
        except FileNotFoundError:
            print(f"Error: Mission file '{filename}' not found.")
            return

        first = True
        for line_num, line in enumerate(lines, 1):
            parts = line.strip().split()
            if len(parts) != 5:
                print(f"Warning: Skipping invalid line {line_num}: {line.strip()}")
                continue
            try:
                lat, lon, yaw, ld, v = map(float, parts)
                if first:
                    self.ref_lat = lat
                    self.ref_lon = lon
                    first = False
                x, y = self.latlon_to_xy(lat, lon)
                self.path.append((x, y, yaw, ld, v))
            except ValueError:
                print(f"Warning: Skipping invalid numeric line {line_num}: {line.strip()}")
                continue

        self.idx = 0
        self.goal_reached = False if self.path else True
        self._build_arc_lengths()
        self.progress_s = 0.0
        self.reacquire_state = "REQUIRED" if self.path else "BLOCKED"
        self.reacquire_detail = "initial position has not been acquired"
        print(f"Loaded {len(self.path)} waypoints from {filename}.")
        if self.path:
            v_max = max(p[4] for p in self.path)
            if v_max > self.max_speed_mps:
                print(f"NOTE: mission file commands up to {v_max:.2f} m/s; "
                      f"will be clamped to --max-speed {self.max_speed_mps:.2f} m/s.")

    def load_audit_phases(self, filename):
        with open(filename, newline='', encoding='utf-8-sig') as handle:
            phases = [str(row.get('phase', '')) for row in csv.DictReader(handle)]
        if len(phases) != len(self.path):
            raise ValueError(
                f"audit has {len(phases)} rows but mission has {len(self.path)} waypoints")
        if any(not phase for phase in phases):
            raise ValueError("audit contains an empty mission phase")
        self.path_phases = phases
        print(f"Loaded {len(phases)} waypoint phases from {filename}.")

    def _build_arc_lengths(self):
        self.cumulative_s = [0.0] if self.path else []
        for previous, current in zip(self.path, self.path[1:]):
            self.cumulative_s.append(
                self.cumulative_s[-1] + math.hypot(
                    current[0] - previous[0], current[1] - previous[1]))

    @staticmethod
    def _angle_difference(a, b):
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    def _segment_projection(self, segment_index, x, y):
        ax, ay = self.path[segment_index][0:2]
        bx, by = self.path[segment_index + 1][0:2]
        vx, vy = bx - ax, by - ay
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-12:
            return None
        fraction = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / length_sq))
        px, py = ax + fraction * vx, ay + fraction * vy
        segment_length = math.sqrt(length_sq)
        return {
            "segment": segment_index,
            "fraction": fraction,
            "x": px,
            "y": py,
            "distance": math.hypot(x - px, y - py),
            "s": self.cumulative_s[segment_index] + fraction * segment_length,
            "heading": math.atan2(vy, vx),
        }

    def _projection_candidates(self, x, y, heading, end_s=None):
        candidates = []
        for segment_index in range(max(0, self.idx - 1), len(self.path) - 1):
            if self.cumulative_s[segment_index + 1] < self.progress_s - 0.25:
                continue
            if end_s is not None and self.cumulative_s[segment_index] > end_s:
                break
            candidate = self._segment_projection(segment_index, x, y)
            if candidate is None or candidate["s"] < self.progress_s - 0.25:
                continue
            if end_s is not None and candidate["s"] > end_s:
                continue
            candidate["heading_error"] = abs(
                self._angle_difference(candidate["heading"], heading))
            candidates.append(candidate)
        return candidates

    def _segment_phase(self, segment_index):
        if not self.path_phases or segment_index + 1 >= len(self.path_phases):
            return None
        first = self.path_phases[segment_index]
        second = self.path_phases[segment_index + 1]
        return first if first == second else None

    def reacquire_forward(self, x, y, heading):
        """Select a close, heading-compatible point at or ahead of progress.

        A near-tie far apart along the mission is rejected instead of guessing at
        crossings, parallel passes, or later phases that happen to be nearby.
        """
        if len(self.path) < 2:
            self.reacquire_state = "BLOCKED"
            self.reacquire_detail = "path has fewer than two waypoints"
            return False
        end_s = min(
            self.cumulative_s[-1], self.progress_s + self.reacquire_max_advance_m)
        current_phase = (
            self.path_phases[min(self.idx, len(self.path_phases) - 1)]
            if self.path_phases else None
        )
        candidates = [
            candidate for candidate in self._projection_candidates(
                x, y, heading, end_s=end_s)
            if candidate["distance"] <= self.reacquire_distance_m
            and candidate["heading_error"] <= self.reacquire_heading_rad
            and (
                current_phase is None
                or self._segment_phase(candidate["segment"]) == current_phase
            )
        ]
        if not candidates:
            self.reacquire_state = "BLOCKED"
            self.reacquire_detail = (
                f"no forward path within {self.reacquire_distance_m:.2f} m "
                f"and {math.degrees(self.reacquire_heading_rad):.0f} deg, "
                f"within {self.reacquire_max_advance_m:.1f} m of progress"
                + (f" and phase {current_phase!r}" if current_phase else ""))
            return False
        candidates.sort(key=lambda candidate: (candidate["distance"], candidate["s"]))
        best = candidates[0]
        ambiguous = [
            candidate for candidate in candidates[1:]
            if candidate["distance"] <= best["distance"] + self.ambiguity_distance_m
            and abs(candidate["s"] - best["s"]) >= self.ambiguity_progress_m
        ]
        if ambiguous:
            other = min(ambiguous, key=lambda candidate: candidate["distance"])
            self.reacquire_state = "BLOCKED"
            self.reacquire_detail = (
                f"ambiguous forward paths at {best['distance']:.2f} m and "
                f"{other['distance']:.2f} m, separated by "
                f"{abs(other['s'] - best['s']):.1f} m of mission progress")
            return False
        self.progress_s = max(self.progress_s, best["s"])
        self.idx = best["segment"]
        self.reacquire_state = "TRACKING"
        self.reacquire_detail = (
            f"acquired segment {self.idx} at {best['distance']:.2f} m, "
            f"heading error {math.degrees(best['heading_error']):.1f} deg")
        return True

    def _interpolate_path(self, target_s):
        target_s = max(0.0, min(target_s, self.cumulative_s[-1]))
        index = max(0, min(self.idx, len(self.path) - 2))
        while index + 1 < len(self.cumulative_s) and self.cumulative_s[index + 1] < target_s:
            index += 1
        while index > 0 and self.cumulative_s[index] > target_s:
            index -= 1
        if index >= len(self.path) - 1:
            return self.path[-1], len(self.path) - 1
        span = self.cumulative_s[index + 1] - self.cumulative_s[index]
        fraction = 0.0 if span <= 1e-12 else (target_s - self.cumulative_s[index]) / span
        a, b = self.path[index], self.path[index + 1]
        yaw_delta = self._angle_difference(b[2], a[2])
        point = (
            a[0] + fraction * (b[0] - a[0]),
            a[1] + fraction * (b[1] - a[1]),
            a[2] + fraction * yaw_delta,
            a[3] + fraction * (b[3] - a[3]),
            a[4] + fraction * (b[4] - a[4]),
        )
        return point, index

    def latlon_to_xy(self, lat, lon):
        lat0_rad = math.radians(self.ref_lat)
        x = 111320.0 * (lon - self.ref_lon) * math.cos(lat0_rad)
        y = 110540.0 * (lat - self.ref_lat)
        return x, y

    def xy_to_latlon(self, x, y):
        """Inverse projection -- used to recover target lat/lon for logging."""
        lat0_rad = math.radians(self.ref_lat)
        dlat = y / 110540.0
        dlon = x / (111320.0 * math.cos(lat0_rad))
        return self.ref_lat + dlat, self.ref_lon + dlon

    def gps_to_base(self, gps_lat, gps_lon, heading_rad):
        gps_x, gps_y = self.latlon_to_xy(gps_lat, gps_lon)
        c, s = math.cos(heading_rad), math.sin(heading_rad)
        base_x = gps_x - (c * self.gps_offset_x - s * self.gps_offset_y)
        base_y = gps_y - (s * self.gps_offset_x + c * self.gps_offset_y)
        return base_x, base_y, heading_rad

    # -- core Pure Pursuit ---------------------------------------------------

    def compute_steering(self, curr_x, curr_y, curr_h):
        """Advance monotonically, then steer to an arc-length lookahead point."""
        if self.goal_reached or len(self.path) < 2:
            self._last_yt = 0.0
            self._last_ld = 0.0
            self._last_delta = 0.0
            self._last_target_xy = (curr_x, curr_y)
            self._last_target_idx = self.idx
            return 0.0, 0.0

        end_s = min(self.cumulative_s[-1], self.progress_s + self.tracking_window_m)
        candidates = self._projection_candidates(curr_x, curr_y, curr_h, end_s=end_s)
        if candidates:
            nearest = min(candidates, key=lambda candidate: candidate["distance"])
            self.progress_s = max(self.progress_s, nearest["s"])
            self.idx = nearest["segment"]

        remaining = self.cumulative_s[-1] - self.progress_s
        end_x, end_y = self.path[-1][0:2]
        if remaining <= self.pos_tol and math.hypot(end_x - curr_x, end_y - curr_y) <= self.pos_tol:
            self.goal_reached = True
            self.idx = len(self.path) - 1
            self._last_yt = 0.0
            self._last_ld = 0.0
            self._last_delta = 0.0
            self._last_target_xy = (end_x, end_y)
            self._last_target_idx = self.idx
            return 0.0, 0.0

        local_point, _local_index = self._interpolate_path(self.progress_s)
        ld = max(0.05, local_point[3])
        target, target_index = self._interpolate_path(self.progress_s + ld)
        px, py, _pyaw, _target_ld, v = target
        dx, dy = px - curr_x, py - curr_y
        rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
        yt = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
        actual_ld = max(0.05, math.hypot(rel_x, yt))
        delta = math.atan2(2.0 * yt * self.L, actual_ld ** 2)
        delta = max(min(delta, self.delta_max), -self.delta_max)

        self._last_yt = yt
        self._last_ld = actual_ld
        self._last_delta = delta
        self._last_target_xy = (px, py)
        self._last_target_idx = target_index

        return delta, v

    # -- conversions ---------------------------------------------------------

    def angle_to_normalized(self, delta):
        delta = max(min(delta, self.delta_max), -self.delta_max)
        return delta / self.delta_max

    def clamp_speed(self, v_mps):
        return max(0.0, min(self.max_speed_mps, v_mps))

    # -- UDP / stats ---------------------------------------------------------

    def send_cmd_vel(self, linear_x_mps, angular_z_normalized):
        cmd = {'linear_x': linear_x_mps,
               'angular_z': angular_z_normalized,
               'timestamp': time.time()}
        self.sock.sendto(json.dumps(cmd).encode('utf-8'),
                         (self.target_ip, self.target_port))
        self.messages_sent += 1

    def _send_stop(self):
        try:
            self.send_cmd_vel(0.0, 0.0)
        except OSError:
            pass

    def print_statistics(self):
        if self.start_time is None:
            return
        now = time.time()
        if self.last_stats_time is None or now - self.last_stats_time >= 5.0:
            elapsed = now - self.last_stats_time if self.last_stats_time else now - self.start_time
            msgs = self.messages_sent - (self.last_stats_count or 0)
            total = now - self.start_time
            print(f"\n=== SENDER STATISTICS ===")
            print(f"Messages sent: {self.messages_sent}")
            print(f"Actual rate (5s): {msgs/elapsed:.1f} Hz  Overall: {self.messages_sent/total:.1f} Hz")
            print(f"Running time: {total:.1f} seconds")
            print(f"=========================\n")
            self.last_stats_time = now
            self.last_stats_count = self.messages_sent

    # -- run modes -----------------------------------------------------------

    def run_interactive(self):
        print("Pure Pursuit ready (interactive mode).")
        print("Format: lat lon heading_deg_MATH_FRAME (CCW from east). 'q' to quit.\n")
        self.start_time = time.time()
        self.last_stats_time = time.time()
        while self.running:
            try:
                inp = input("GPS input (lat lon heading_deg) or 'q': ").strip()
                if inp.lower() == 'q':
                    break
                parts = inp.split()
                if len(parts) != 3:
                    print("Invalid input. Need exactly 3 values.")
                    continue
                lat, lon, h_deg = map(float, parts)
                h_rad = math.radians(h_deg)
                self.last_x, self.last_y, self.last_h = self.gps_to_base(lat, lon, h_rad)
                delta, v = self.compute_steering(self.last_x, self.last_y, self.last_h)
                steer_cmd = self.angle_to_normalized(delta)
                speed_cmd = self.clamp_speed(v)
                self.send_cmd_vel(speed_cmd, steer_cmd)
                print(f"x={self.last_x:.3f}m y={self.last_y:.3f}m h={math.degrees(self.last_h):.1f}deg")
                print(f"idx={self.idx}/{len(self.path)} delta={math.degrees(delta):.2f}deg "
                      f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f}m/s")
                if self.goal_reached:
                    print("Goal reached!")
                    break
                self.print_statistics()
            except ValueError:
                print("Invalid numeric input.")
            except Exception as e:
                print(f"Error: {e}")
        self.cleanup()

    def run_timed(self):
        if not self.path:
            print("No path loaded.")
            return
        print(f"[SIMULATION] {self.rate_hz} Hz, target {self.target_ip}:{self.target_port}")
        self.start_time = time.time()
        self.last_stats_time = self.start_time
        next_send = self.start_time
        try:
            while self.running:
                now = time.time()
                if now >= next_send:
                    delta, v = self.compute_steering(self.last_x, self.last_y, self.last_h)
                    self.send_cmd_vel(self.clamp_speed(v), self.angle_to_normalized(delta))
                    if self.messages_sent % 100 == 0:
                        print(f"#{self.messages_sent} idx={self.idx} "
                              f"delta={math.degrees(delta):.1f}deg")
                    next_send += self.period
                    if next_send < now:
                        next_send = now + self.period
                self.print_statistics()
                sleep = next_send - time.time()
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.cleanup()

    def run_live(self, gps_receiver, logger=None, control_receiver=None,
                 handheld_receiver=None, telemetry_port=0,
                 resume_stable_seconds=5.0):
        """Field-test mode. Logs every cycle to CSV if logger provided."""
        if not self.path:
            print("No path loaded.")
            return

        print(f"[LIVE] GPS UDP {gps_receiver.port} -> cmd_vel UDP {self.target_port} "
              f"@ {self.rate_hz} Hz")
        print(f"Gate: min-fix={gps_receiver.min_fix!r}, headValid={gps_receiver.require_head_valid}")
        print(f"Speed cap: {self.max_speed_mps:.2f} m/s")
        if logger:
            print(f"Pursuit CSV: {logger.path}")
        print("Press Ctrl+C to stop\n")

        self.start_time = time.time()
        self.last_stats_time = self.start_time
        loop_count = 0
        telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if telemetry_port else None
        operator_cycle_required = handheld_receiver is not None
        operator_non_auto_seen = handheld_receiver is None
        drivable_since = None

        try:
            while self.running:
                loop_start = time.time()
                now = loop_start
                elapsed = now - self.start_time

                pose = gps_receiver.get_pose()
                raw_ok, reason = gps_receiver.is_drivable(pose)
                if raw_ok:
                    if drivable_since is None:
                        drivable_since = now
                    stable_seconds = now - drivable_since
                    ok = stable_seconds >= resume_stable_seconds
                    if not ok:
                        reason = (
                            f"GPS/heading stable for {stable_seconds:.1f}/"
                            f"{resume_stable_seconds:.1f} s")
                else:
                    drivable_since = None
                    ok = False
                software_paused = bool(control_receiver and control_receiver.is_paused())
                handheld_ok, handheld_reason, handheld = (
                    handheld_receiver.auto_ready() if handheld_receiver
                    else (True, "", None)
                )
                if (handheld is not None
                        and handheld["age"] <= STATUS_STALE_TIMEOUT_S
                        and handheld["mode"] != 0):
                    operator_non_auto_seen = True

                # Base CSV row -- filled in for every cycle regardless of state
                row = {
                    'timestamp':      f"{now:.3f}",
                    'elapsed_s':      f"{elapsed:.3f}",
                    'cycle':          loop_count,
                    'driving':        False,
                    'wait_reason':    '',
                    'goal_reached':   self.goal_reached,
                    'waypoints_total': len(self.path),
                    'waypoint_idx':   self.idx,
                    'software_paused': software_paused,
                    'handheld_mode': '' if handheld is None else handheld['mode'],
                    'handheld_state': '' if handheld is None else handheld['state'],
                    'path_progress_m': f"{self.progress_s:.3f}",
                    'reacquire_state': self.reacquire_state,
                    'reacquire_detail': self.reacquire_detail,
                }

                if pose is not None:
                    row.update({
                        'lat':                 f"{pose['lat']:.8f}",
                        'lon':                 f"{pose['lon']:.8f}",
                        'heading_compass_deg': f"{pose['heading_compass_deg']:.3f}",
                        'fix_quality':         pose['fix_quality'],
                        'head_valid':          pose['headValid'],
                        'heading_carrier':     pose.get('carrier'),
                        'relpos_length_m':     pose.get('relpos_length_m'),
                        'heading_accuracy_deg': pose.get('heading_accuracy_deg'),
                        'relpos_gnss_fix_ok':  pose.get('relpos_gnss_fix_ok'),
                        'relpos_diff_solution': pose.get('relpos_diff_solution'),
                        'relpos_valid':        pose.get('relpos_valid'),
                        'relpos_moving':       pose.get('relpos_moving'),
                        'relpos_ref_pos_miss': pose.get('relpos_ref_pos_miss'),
                        'relpos_ref_obs_miss': pose.get('relpos_ref_obs_miss'),
                        'relpos_normalized':   pose.get('relpos_normalized'),
                        'relposned_count':     pose.get('relposned_count'),
                        'relposned_itow_ms':   pose.get('relposned_itow_ms'),
                        'heading_numSV_used':  pose.get('heading_numSV_used'),
                        'heading_cno_mean_dbhz': pose.get('heading_cno_mean_dbhz'),
                        'gps_age_s':           f"{pose['age']:.3f}",
                        'actual_speed_mps':    pose.get('speed_mps'),
                    })

                if software_paused:
                    self.reacquire_state = "REQUIRED"
                    self.reacquire_detail = "software pause"
                    self._send_stop()
                    row['wait_reason'] = 'software pause'
                    if pose is not None:
                        bx, by, bh = self.gps_to_base(
                            pose['lat'], pose['lon'], pose['heading_rad'])
                        self.last_x, self.last_y, self.last_h = bx, by, bh
                        row.update({'pos_x_m': f"{bx:.4f}", 'pos_y_m': f"{by:.4f}"})
                    if loop_count % 20 == 0:
                        print("[WAIT] software pause")
                elif not ok:
                    operator_cycle_required = handheld_receiver is not None
                    self.reacquire_state = "REQUIRED"
                    self.reacquire_detail = (
                        f"{reason}; select Manual/Pause, then return to AUTO after recovery")
                    self._send_stop()
                    row['wait_reason'] = self.reacquire_detail
                    if loop_count % 20 == 0:
                        print(f"[WAIT] not driving: {reason}")
                elif not handheld_ok:
                    operator_cycle_required = True
                    self.reacquire_state = "REQUIRED"
                    self.reacquire_detail = handheld_reason
                    self._send_stop()
                    row['wait_reason'] = handheld_reason
                    if pose is not None:
                        bx, by, bh = self.gps_to_base(
                            pose['lat'], pose['lon'], pose['heading_rad'])
                        self.last_x, self.last_y, self.last_h = bx, by, bh
                        row.update({'pos_x_m': f"{bx:.4f}", 'pos_y_m': f"{by:.4f}"})
                    if loop_count % 20 == 0:
                        print(f"[WAIT] {handheld_reason}; path progress frozen")
                else:
                    bx, by, bh = self.gps_to_base(
                        pose['lat'], pose['lon'], pose['heading_rad'])
                    self.last_x, self.last_y, self.last_h = bx, by, bh

                    if operator_cycle_required and not operator_non_auto_seen:
                        self.reacquire_state = "REQUIRED"
                        self.reacquire_detail = (
                            "operator acknowledgement required: select Manual/Pause, "
                            "then return to AUTO")

                    if self.reacquire_state != "TRACKING":
                        if operator_cycle_required and not operator_non_auto_seen:
                            self._send_stop()
                            row['wait_reason'] = self.reacquire_detail
                            row.update({'pos_x_m': f"{bx:.4f}", 'pos_y_m': f"{by:.4f}"})
                            if loop_count % 20 == 0:
                                print(f"[WAIT] {self.reacquire_detail}")
                            if logger:
                                logger.write(row)
                            if telemetry_sock:
                                live = dict(row)
                                live['controller_state'] = 'WAITING_OPERATOR_CYCLE'
                                try:
                                    telemetry_sock.sendto(
                                        json.dumps(live).encode('utf-8'),
                                        ('127.0.0.1', telemetry_port),
                                    )
                                except OSError:
                                    pass
                            loop_count += 1
                            sleep = self.period - (time.time() - loop_start)
                            if sleep > 0:
                                time.sleep(sleep)
                            continue
                        acquired = self.reacquire_forward(bx, by, bh)
                        row['reacquire_state'] = self.reacquire_state
                        row['reacquire_detail'] = self.reacquire_detail
                        row['path_progress_m'] = f"{self.progress_s:.3f}"
                        if not acquired:
                            self._send_stop()
                            row['wait_reason'] = self.reacquire_detail
                            row.update({'pos_x_m': f"{bx:.4f}", 'pos_y_m': f"{by:.4f}"})
                            if loop_count % 20 == 0:
                                print(f"[WAIT] reacquisition blocked: {self.reacquire_detail}")
                            if logger:
                                logger.write(row)
                            if telemetry_sock:
                                live = dict(row)
                                live['controller_state'] = 'WAITING_REACQUIRE'
                                try:
                                    telemetry_sock.sendto(
                                        json.dumps(live).encode('utf-8'),
                                        ('127.0.0.1', telemetry_port),
                                    )
                                except OSError:
                                    pass
                            loop_count += 1
                            sleep = self.period - (time.time() - loop_start)
                            if sleep > 0:
                                time.sleep(sleep)
                            continue
                        operator_cycle_required = False
                        operator_non_auto_seen = False
                        print(f"[RECOVERY] {self.reacquire_detail}")

                    delta, v = self.compute_steering(bx, by, bh)
                    steer_cmd = self.angle_to_normalized(delta)
                    speed_cmd = self.clamp_speed(v)
                    self.send_cmd_vel(speed_cmd, steer_cmd)

                    # Recover target lat/lon from local frame for the log
                    tx, ty = self._last_target_xy
                    t_lat, t_lon = self.xy_to_latlon(tx, ty)

                    # alpha = heading error angle
                    safe_ld = self._last_ld if self._last_ld > 0 else 1e-6
                    alpha_deg = math.degrees(
                        math.asin(max(-1.0, min(1.0, self._last_yt / safe_ld))))

                    row.update({
                        'pos_x_m':           f"{bx:.4f}",
                        'pos_y_m':           f"{by:.4f}",
                        'waypoint_idx':      self._last_target_idx,
                        'goal_reached':      self.goal_reached,
                        'target_lat':        f"{t_lat:.8f}",
                        'target_lon':        f"{t_lon:.8f}",
                        'target_x_m':        f"{tx:.4f}",
                        'target_y_m':        f"{ty:.4f}",
                        'lookahead_dist_m':  f"{self._last_ld:.3f}",
                        'yt_m':              f"{self._last_yt:.4f}",
                        'cross_track_err_m': f"{abs(self._last_yt):.4f}",
                        'alpha_deg':         f"{alpha_deg:.3f}",
                        'delta_deg':         f"{math.degrees(self._last_delta):.3f}",
                        'steer_normalized':  f"{steer_cmd:+.4f}",
                        'speed_cmd_mps':     f"{speed_cmd:.3f}",
                        'driving':           True,
                        'path_progress_m':   f"{self.progress_s:.3f}",
                        'reacquire_state':   self.reacquire_state,
                        'reacquire_detail':  self.reacquire_detail,
                    })

                    if loop_count % 20 == 0:
                        print(f"idx={self._last_target_idx}/{len(self.path)} "
                              f"x={bx:.2f} y={by:.2f} "
                              f"h={math.degrees(bh):.1f}deg "
                              f"yt={self._last_yt:+.3f}m "
                              f"delta={math.degrees(delta):.1f}deg "
                              f"steer={steer_cmd:+.2f} "
                              f"speed={speed_cmd:.2f}m/s "
                              f"fix={pose['fix_quality']}")

                    if self.goal_reached:
                        print("Goal reached -- sending stop.")
                        self._send_stop()
                        if logger:
                            logger.write(row)
                        break

                row['path_progress_m'] = f"{self.progress_s:.3f}"
                row['reacquire_state'] = self.reacquire_state
                row['reacquire_detail'] = self.reacquire_detail

                if logger:
                    logger.write(row)

                if telemetry_sock:
                    live = dict(row)
                    live['controller_state'] = (
                        'PAUSED' if software_paused else
                        ('WAITING' if (not ok or not handheld_ok) else 'RUNNING')
                    )
                    try:
                        telemetry_sock.sendto(
                            json.dumps(live).encode('utf-8'),
                            ('127.0.0.1', telemetry_port),
                        )
                    except OSError:
                        pass

                self.print_statistics()
                loop_count += 1
                sleep = self.period - (time.time() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            print("\nInterrupted -- stopping.")
        finally:
            if telemetry_sock:
                telemetry_sock.close()
            self.cleanup()

    def cleanup(self):
        self.running = False
        self._send_stop()
        if self.start_time:
            total = time.time() - self.start_time
            print(f"\n=== FINAL STATISTICS ===")
            print(f"Total messages sent: {self.messages_sent}")
            print(f"Total time: {total:.1f} seconds")
            print(f"Average rate: {self.messages_sent/total:.1f} Hz")
            print(f"========================")
        self.sock.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Pure Pursuit controller')
    parser.add_argument('mission_file', nargs='?',
                        help='Mission file (required for timed/live modes)')
    parser.add_argument('--ip', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=CMD_VEL_UDP_PORT)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--mode', choices=['interactive', 'timed', 'live'],
                        default='interactive')
    parser.add_argument('--gps-port', type=int, default=GPS_UDP_PORT)
    parser.add_argument('--status-port', type=int, default=TEENSY_STATUS_UDP_PORT,
                        help='Teensy bridge status port used to gate AUTO mode')
    parser.add_argument('--min-fix', choices=['RTK Fixed', 'RTK Float', 'any'],
                        default='RTK Fixed')
    parser.add_argument('--allow-head-invalid', action='store_true')
    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help=f'Speed cap m/s (default {DEFAULT_MAX_SPEED_MPS})')
    parser.add_argument('--pos-tol', type=float, default=0.5,
                        help='Along-track goal window meters (default 0.5)')
    parser.add_argument('--tracking-window', type=float, default=12.0,
                        help='Normal forward projection window in meters')
    parser.add_argument('--reacquire-distance', type=float, default=2.0,
                        help='Maximum cross-track distance for AUTO resumption')
    parser.add_argument('--reacquire-heading', type=float, default=60.0,
                        help='Maximum heading error degrees for AUTO resumption')
    parser.add_argument('--ambiguity-distance', type=float, default=0.35,
                        help='Near-tie distance that blocks ambiguous recovery')
    parser.add_argument('--ambiguity-progress', type=float, default=8.0,
                        help='Along-mission separation that makes a near tie ambiguous')
    parser.add_argument('--reacquire-max-advance', type=float, default=30.0,
                        help='Maximum along-path progress a recovery may advance')
    parser.add_argument('--audit-file',
                        help='Waypoint audit CSV; recovery remains in the current phase')
    parser.add_argument('--resume-stable-seconds', type=float, default=5.0,
                        help='Continuous healthy GPS/heading time required before driving')
    parser.add_argument('--no-pursuit-log', action='store_true',
                        help='Disable per-cycle CSV logging (on by default in live mode)')
    parser.add_argument('--control-port', type=int, default=0,
                        help='Local UDP port for dashboard Pause/Resume commands')
    parser.add_argument('--telemetry-port', type=int, default=0,
                        help='Local UDP port for live dashboard controller telemetry')
    args = parser.parse_args()

    pp = PurePursuit(
        target_ip=args.ip,
        target_port=args.port,
        rate_hz=args.rate,
        max_speed_mps=args.max_speed,
        pos_tol=args.pos_tol,
        tracking_window_m=args.tracking_window,
        reacquire_distance_m=args.reacquire_distance,
        reacquire_heading_deg=args.reacquire_heading,
        ambiguity_distance_m=args.ambiguity_distance,
        ambiguity_progress_m=args.ambiguity_progress,
        reacquire_max_advance_m=args.reacquire_max_advance,
    )

    if args.mode in ('timed', 'live') and not args.mission_file:
        print(f"Error: --mode {args.mode} requires mission_file.")
        sys.exit(1)

    if args.mission_file:
        pp.load_path(args.mission_file)
    if args.audit_file:
        pp.load_audit_phases(args.audit_file)

    gps_receiver = None
    logger = None
    control_receiver = None
    handheld_receiver = None

    if args.mode == 'live':
        min_fix = None if args.min_fix == 'any' else args.min_fix
        gps_receiver = GPSReceiver(port=args.gps_port, min_fix=min_fix,
                                   require_head_valid=not args.allow_head_invalid)
        handheld_receiver = HandheldStatusReceiver(port=args.status_port)
        if not args.no_pursuit_log:
            logger = PursuitLogger()
        if args.control_port:
            control_receiver = MissionControlReceiver(args.control_port)

    def signal_handler(sig, frame):
        pp.cleanup()
        if gps_receiver:
            gps_receiver.stop()
        if logger:
            logger.close()
        if control_receiver:
            control_receiver.stop()
        if handheld_receiver:
            handheld_receiver.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        if args.mode == 'interactive':
            pp.run_interactive()
        elif args.mode == 'timed':
            pp.run_timed()
        else:
            pp.run_live(
                gps_receiver,
                logger=logger,
                control_receiver=control_receiver,
                handheld_receiver=handheld_receiver,
                telemetry_port=args.telemetry_port,
                resume_stable_seconds=args.resume_stable_seconds,
            )
            gps_receiver.stop()
    finally:
        if control_receiver:
            control_receiver.stop()
        if handheld_receiver:
            handheld_receiver.stop()
        if logger:
            logger.close()


if __name__ == "__main__":
    main()
