"""
pure_pursuit_controller_20260714.py -- Pure Pursuit controller with live F9P/GPS input.

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
CMD_VEL_UDP_PORT = 6004
GPS_STALE_TIMEOUT_S = 0.5
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
    ("driving",             "True if cmd_vel sent this cycle False = WAIT state"),
    ("wait_reason",         "Reason not driving this cycle empty string if driving"),
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

    def __init__(self, port=GPS_UDP_PORT, min_fix="RTK Fixed", require_head_valid=True):
        self.port = port
        self.min_fix = min_fix
        self.require_head_valid = require_head_valid
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
                'fix_quality': msg.get('fix_quality'),
                'headValid': bool(msg.get('headValid')),
                'carrier': msg.get('carrier'),
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
            return False, "headValid=False"
        return True, ""

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
                 rate_hz=20.0, max_speed_mps=DEFAULT_MAX_SPEED_MPS):
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
        print(f"Loaded {len(self.path)} waypoints from {filename}.")
        if self.path:
            v_max = max(p[4] for p in self.path)
            if v_max > self.max_speed_mps:
                print(f"NOTE: mission file commands up to {v_max:.2f} m/s; "
                      f"will be clamped to --max-speed {self.max_speed_mps:.2f} m/s.")

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
        """Compute steering angle (rad) and speed (m/s).
        Sets self._last_* fields for per-cycle logging after every call."""
        if self.goal_reached or not self.path:
            self._last_yt = 0.0
            self._last_ld = 0.0
            self._last_delta = 0.0
            self._last_target_xy = (curr_x, curr_y)
            self._last_target_idx = self.idx
            return 0.0, 0.0

        found = False
        for i in range(self.idx, len(self.path)):
            px, py, _, p_ld, p_v = self.path[i]
            if math.hypot(px - curr_x, py - curr_y) > p_ld:
                self.idx = i
                ld, v = p_ld, p_v
                found = True
                break

        if not found:
            self.idx = len(self.path)
            if self.idx == 0:
                return 0.0, 0.0
            px, py, pyaw, ld, v = self.path[-1]
            dx, dy = px - curr_x, py - curr_y
            rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
            rel_y = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
            if rel_x <= self.pos_tol:
                self.goal_reached = True
                self._last_yt = rel_y
                self._last_ld = ld
                self._last_delta = 0.0
                self._last_target_xy = (px, py)
                self._last_target_idx = len(self.path) - 1
                return 0.0, 0.0
            k_end = math.tan(pyaw - curr_h)
            l_end = rel_y - k_end * rel_x
            a = 1.0 + k_end ** 2
            b = 2.0 * k_end * l_end
            c = l_end ** 2 - ld ** 2
            disc = b ** 2 - 4.0 * a * c
            if disc < 0:
                dist_end = math.hypot(rel_x, rel_y)
                yt = rel_y * (ld / dist_end) if dist_end > 0 else rel_y
            else:
                x_ld = (-b + math.copysign(math.sqrt(disc), v)) / (2.0 * a)
                yt = k_end * x_ld + l_end
            target_xy, target_idx = (px, py), len(self.path) - 1
        else:
            px, py = self.path[self.idx][0], self.path[self.idx][1]
            dx, dy = px - curr_x, py - curr_y
            rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
            rel_y = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
            yt = rel_y
            ld, v = self.path[self.idx][3], self.path[self.idx][4]
            target_xy, target_idx = (px, py), self.idx

        delta = math.atan2(2.0 * yt * self.L, ld ** 2)
        delta = max(min(delta, self.delta_max), -self.delta_max)

        self._last_yt = yt
        self._last_ld = ld
        self._last_delta = delta
        self._last_target_xy = target_xy
        self._last_target_idx = target_idx

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

    def run_live(self, gps_receiver, logger=None):
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

        try:
            while self.running:
                loop_start = time.time()
                now = loop_start
                elapsed = now - self.start_time

                pose = gps_receiver.get_pose()
                ok, reason = gps_receiver.is_drivable(pose)

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
                }

                if pose is not None:
                    row.update({
                        'lat':                 f"{pose['lat']:.8f}",
                        'lon':                 f"{pose['lon']:.8f}",
                        'heading_compass_deg': f"{pose['heading_compass_deg']:.3f}",
                        'fix_quality':         pose['fix_quality'],
                        'head_valid':          pose['headValid'],
                        'gps_age_s':           f"{pose['age']:.3f}",
                    })

                if not ok:
                    self._send_stop()
                    row['wait_reason'] = reason
                    if loop_count % 20 == 0:
                        print(f"[WAIT] not driving: {reason}")
                else:
                    bx, by, bh = self.gps_to_base(
                        pose['lat'], pose['lon'], pose['heading_rad'])
                    self.last_x, self.last_y, self.last_h = bx, by, bh

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

                if logger:
                    logger.write(row)

                self.print_statistics()
                loop_count += 1
                sleep = self.period - (time.time() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            print("\nInterrupted -- stopping.")
        finally:
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
    parser.add_argument('--min-fix', choices=['RTK Fixed', 'RTK Float', 'any'],
                        default='RTK Fixed')
    parser.add_argument('--allow-head-invalid', action='store_true')
    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help=f'Speed cap m/s (default {DEFAULT_MAX_SPEED_MPS})')
    parser.add_argument('--pos-tol', type=float, default=0.5,
                        help='Along-track goal window meters (default 0.5)')
    parser.add_argument('--no-pursuit-log', action='store_true',
                        help='Disable per-cycle CSV logging (on by default in live mode)')
    args = parser.parse_args()

    pp = PurePursuit(
        target_ip=args.ip,
        target_port=args.port,
        rate_hz=args.rate,
        max_speed_mps=args.max_speed,
        pos_tol=args.pos_tol,
    )

    if args.mode in ('timed', 'live') and not args.mission_file:
        print(f"Error: --mode {args.mode} requires mission_file.")
        sys.exit(1)

    if args.mission_file:
        pp.load_path(args.mission_file)

    gps_receiver = None
    logger = None

    if args.mode == 'live':
        min_fix = None if args.min_fix == 'any' else args.min_fix
        gps_receiver = GPSReceiver(port=args.gps_port, min_fix=min_fix,
                                   require_head_valid=not args.allow_head_invalid)
        if not args.no_pursuit_log:
            logger = PursuitLogger()

    def signal_handler(sig, frame):
        pp.cleanup()
        if gps_receiver:
            gps_receiver.stop()
        if logger:
            logger.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        if args.mode == 'interactive':
            pp.run_interactive()
        elif args.mode == 'timed':
            pp.run_timed()
        else:
            pp.run_live(gps_receiver, logger=logger)
            gps_receiver.stop()
    finally:
        if logger:
            logger.close()


if __name__ == "__main__":
    main()
