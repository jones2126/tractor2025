"""
pure_pursuit_controller_20260714.py -- Pure Pursuit controller with live F9P/GPS input.

Lineage: pure_pursuit_grok_20251112.py -> pure_pursuit_live_20260713.py -> this file.

Change log vs pure_pursuit_live_20260713.py:

  CHANGED linear_x is now commanded in METERS PER SECOND, POSITIVE = FORWARD
        (was: normalized -1..0 with negative = forward). The Teensy owns the
        m/s -> transmission-actuator mapping (lookup table from speed
        calibration); this layer holds no speed calibration.
        !! REQUIRES the matching Teensy firmware change -- old firmware
        interpreting m/s values on the old normalized scale would read
        +0.5 m/s as a REVERSE command.
  REMOVED speed_to_linear_x() and the max-speed-as-calibration-scale concept.
  NEW   clamp_speed() -- safety cap only: commanded speed clamped to
        [0, max_speed_mps]. --max-speed default 1.5 m/s.
  CHANGED gps_offset_x/y set to 0.0. The position F9P antenna is mounted
        directly over the rear axle center (= base_link), so lat/lon on UDP
        6002 is already the base_link position. The heading F9P (~36 in ahead)
        only feeds RELPOSNED heading; it contributes no position offset.
        (The 0.3048/0.1524 values in the original Grok script were unverified.)

Carried over from pure_pursuit_live_20260713.py:

  GPSReceiver -- background thread on UDP 6002 (rtcm_server broadcast), parses
        {lat, lon, heading_deg, fix_quality, headValid, carrier, fatal_error, ...},
        converts compass heading_deg -> math-frame radians (CCW from east) via
        radians(90 - heading_deg), exposes latest pose with an "age".
  run_live() -- field-test mode: refuses to drive (sends stop) on stale pose,
        fix below --min-fix, headValid False, or rtcm_server fatal_error.
  angle_to_normalized() -- steering sent NORMALIZED: +1.0 = full LEFT lock,
        -1.0 = full RIGHT lock, 0.0 = center. Teensy owns the pot calibration
        (197/447/815 on tractor01).
  cleanup() sends an explicit stop before closing the socket.
  compute_steering() end-of-path root selection uses copysign(sqrt_disc, v),
        matching pure_pursuit.cpp's copysign(D, v_).

Core Pure Pursuit math note: delta = atan2(2*yt*L, ld^2) is the textbook
delta = atan(2L*sin(alpha)/ld) with yt = ld*sin(alpha) substituted -- yt is the
lateral offset of the lookahead point in the vehicle frame. Speed plays no
role in the steering computation; the mission file's speed column is passed
through as the commanded speed.

cmd_vel JSON sent on UDP 6004:
    {"linear_x": <m/s, POSITIVE = FORWARD, 0 = stop>,
     "angular_z": <-1..+1, +1 = full left, 0 = center>,
     "timestamp": <epoch>}
"""

import math
import socket
import json
import time
import argparse
import signal
import sys
import threading

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# UDP port conventions already in use on this project.
GPS_UDP_PORT = 6002          # rtcm_server position/heading broadcast (10-20 Hz)
CMD_VEL_UDP_PORT = 6004      # teensy_serial_bridge command listener

# How stale a GPS pose can be before we refuse to drive on it.
# rtcm_server publishes at 10-20 Hz, so 0.5 s is several missed packets, not one.
GPS_STALE_TIMEOUT_S = 0.5

# Default safety cap on commanded speed (m/s). A bad waypoint or file edit
# cannot command more than this.
DEFAULT_MAX_SPEED_MPS = 1.5

# Fix-quality ranking used for --min-fix gating.
_FIX_RANK = {
    None: 0, "Unknown": 0, "Invalid": 0,
    "GPS Fix": 1, "DGPS": 1,
    "RTK Float": 2,
    "RTK Fixed": 3,
}


def _fix_ok(fix_quality, min_fix):
    if min_fix is None:  # "any" -- gating disabled, bench-test use only
        return True
    return _FIX_RANK.get(fix_quality, 0) >= _FIX_RANK.get(min_fix, 3)


# ---------------------------------------------------------------------------
# Live GPS listener
# ---------------------------------------------------------------------------

class GPSReceiver:
    """Background UDP listener for rtcm_server's broadcast on port 6002.

    SO_REUSEPORT lets this run alongside other consumers already bound to the
    same port (field_test_logger, etc.) -- required per project notes, since a
    plain bind() would otherwise fail or steal packets.
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
            return  # no fix yet -- don't overwrite the last good pose with garbage

        # rtcm_server publishes heading_deg as compass degrees from true north.
        # The Pure Pursuit math (inherited from pure_pursuit.cpp) assumes a
        # math-frame heading: radians, CCW-positive, 0 = pointing east.
        heading_rad = math.radians(90.0 - heading_deg)

        with self._lock:
            self._latest = {
                'lat': lat,
                'lon': lon,
                'heading_rad': heading_rad,
                'fix_quality': msg.get('fix_quality'),
                'headValid': bool(msg.get('headValid')),
                'carrier': msg.get('carrier'),
                'fatal_error': bool(msg.get('fatal_error', False)),
                'fatal_base_reason': msg.get('fatal_base_reason'),
                'fatal_heading_reason': msg.get('fatal_heading_reason'),
            }
            self._last_update = time.time()

    def get_pose(self):
        """Latest pose dict with an 'age' field in seconds, or None if nothing
        has been received yet."""
        with self._lock:
            if self._latest is None:
                return None
            pose = dict(self._latest)
            pose['age'] = time.time() - self._last_update
            return pose

    def is_drivable(self, pose):
        """True if pose is fresh enough and meets the fix-quality/headValid gate."""
        if pose is None or pose['age'] > GPS_STALE_TIMEOUT_S:
            return False, "stale or no GPS"
        if pose['fatal_error']:
            return False, (f"rtcm_server fatal_error (base={pose['fatal_base_reason']}, "
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
    def __init__(self, wheelbase=1.27, max_steer=0.623, pos_tol=0.1,
                 target_ip='127.0.0.1', target_port=CMD_VEL_UDP_PORT, rate_hz=20.0,
                 max_speed_mps=DEFAULT_MAX_SPEED_MPS):
        """
        :param wheelbase: Vehicle wheelbase in meters (default: 1.27m).
        :param max_steer: Maximum steering angle in radians (default: 0.623 rad
               ~35.7 deg, derived from measured turning circle). This is the
               only steering-geometry value the navigation layer needs; pot
               calibration lives on the Teensy.
        :param pos_tol: Position tolerance for goal reaching in meters.
        :param target_ip: IP for UDP sending (default: localhost).
        :param target_port: UDP port for cmd_vel (default: 6004).
        :param rate_hz: Send rate in Hz for timed/live modes.
        :param max_speed_mps: Safety CAP on commanded speed in m/s (default
               1.5). Not a calibration -- the Teensy owns the m/s -> actuator
               mapping. This only clamps what the mission file can command.
        """
        self.L = wheelbase
        self.delta_max = max_steer
        self.pos_tol = pos_tol
        self.max_speed_mps = max_speed_mps

        # GPS antenna -> base_link offsets, in meters, vehicle frame
        # (+x forward, +y left). The position F9P antenna is mounted directly
        # over the rear axle center = base_link, so both are zero on tractor01.
        # The heading F9P (~36 in ahead) only feeds RELPOSNED heading and
        # needs no position offset. If a future machine mounts the position
        # antenna elsewhere, measure antenna-to-rear-axle-center and set these.
        self.gps_offset_x = 0.0
        self.gps_offset_y = 0.0

        self.path = []  # list of (x, y, yaw_rad, ld, v) in local frame
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

    # -- path loading ---------------------------------------------------------

    def load_path(self, filename):
        """Load mission path from TXT file: lat lon yaw_rad lookahead_m speed_mps"""
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
            v_max_in_file = max(p[4] for p in self.path)
            if v_max_in_file > self.max_speed_mps:
                print(f"NOTE: mission file commands up to {v_max_in_file:.2f} m/s; "
                      f"will be clamped to --max-speed {self.max_speed_mps:.2f} m/s.")

    def latlon_to_xy(self, lat, lon):
        """lat/lon -> local x/y meters (equirectangular projection).
        111320/110540 are meters-per-DEGREE, so they multiply degree deltas."""
        lat0_rad = math.radians(self.ref_lat)
        dlat_deg = lat - self.ref_lat
        dlon_deg = lon - self.ref_lon
        x = 111320.0 * dlon_deg * math.cos(lat0_rad)
        y = 110540.0 * dlat_deg
        return x, y

    def gps_to_base(self, gps_lat, gps_lon, heading_rad):
        """Transform GPS antenna position to base_link (rear axle center).
        With both offsets 0.0 (tractor01: antenna over the rear axle), this
        passes lat/lon through unchanged; kept for machines where it isn't."""
        gps_x, gps_y = self.latlon_to_xy(gps_lat, gps_lon)
        c = math.cos(heading_rad)
        s = math.sin(heading_rad)
        delta_x = c * self.gps_offset_x - s * self.gps_offset_y
        delta_y = s * self.gps_offset_x + c * self.gps_offset_y
        base_x = gps_x - delta_x
        base_y = gps_y - delta_y
        return base_x, base_y, heading_rad

    # -- core Pure Pursuit ------------------------------------------------------

    def compute_steering(self, curr_x, curr_y, curr_h):
        """Compute steering angle (rad) and speed (m/s) using Pure Pursuit.

        delta = atan2(2*yt*L, ld^2) is the textbook pure-pursuit law
        delta = atan(2L*sin(alpha)/ld) with yt = ld*sin(alpha) substituted,
        where yt is the lateral offset of the lookahead point in the vehicle
        frame. Speed does not enter the steering computation; the mission
        file's speed column is simply passed through.
        """
        if self.goal_reached or not self.path:
            return 0.0, 0.0

        found = False
        for i in range(self.idx, len(self.path)):
            px, py, _, p_ld, p_v = self.path[i]
            dist = math.hypot(px - curr_x, py - curr_y)
            if dist > p_ld:
                self.idx = i
                ld = p_ld
                v = p_v
                found = True
                break

        if not found:
            self.idx = len(self.path)
            if self.idx == 0:
                return 0.0, 0.0
            px, py, pyaw, ld, v = self.path[-1]
            dx = px - curr_x
            dy = py - curr_y
            rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
            rel_y = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
            if abs(rel_x) <= self.pos_tol:
                self.goal_reached = True
                return 0.0, 0.0
            rel_yaw = pyaw - curr_h
            k_end = math.tan(rel_yaw)
            x_end = rel_x
            y_end = rel_y
            l_end = y_end - k_end * x_end
            a = 1.0 + k_end ** 2
            b = 2.0 * k_end * l_end
            c = l_end ** 2 - ld ** 2
            disc = b ** 2 - 4.0 * a * c
            if disc < 0:
                dist_end = math.hypot(rel_x, rel_y)
                if dist_end > 0:
                    scale = ld / dist_end
                    yt = rel_y * scale
                else:
                    yt = rel_y
            else:
                sqrt_disc = math.sqrt(disc)
                # Sign of v (direction of travel), matching pure_pursuit.cpp's
                # copysign(D, v_).
                x_ld = (-b + math.copysign(sqrt_disc, v)) / (2.0 * a)
                y_ld = k_end * x_ld + l_end
                yt = y_ld
        else:
            px, py, _, _, _ = self.path[self.idx]
            dx = px - curr_x
            dy = py - curr_y
            rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
            rel_y = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
            yt = rel_y
            ld = self.path[self.idx][3]
            v = self.path[self.idx][4]

        ld2 = ld ** 2
        delta = math.atan2(2.0 * yt * self.L, ld2)
        delta = max(min(delta, self.delta_max), -self.delta_max)
        return delta, v

    # -- unit conversions to bridge units ---------------------------------------

    def angle_to_normalized(self, delta):
        """Steering angle (rad) -> normalized command for the Teensy.

        Convention: +1.0 = full LEFT lock, -1.0 = full RIGHT lock, 0.0 = center
        (matches pure-pursuit sign convention: positive delta = turn left).
        The Teensy firmware (auto mode 2) maps this to its machine-specific
        pot calibration -- no pot counts live in this navigation layer.
        """
        delta = max(min(delta, self.delta_max), -self.delta_max)
        return delta / self.delta_max

    def clamp_speed(self, v_mps):
        """Commanded speed in m/s, POSITIVE = FORWARD. Clamped to
        [0, max_speed_mps] as a safety limit. The Teensy owns the mapping
        from m/s to transmission actuator position."""
        return max(0.0, min(self.max_speed_mps, v_mps))

    # -- UDP send / stats ---------------------------------------------------------

    def send_cmd_vel(self, linear_x_mps, angular_z_normalized):
        cmd = {
            'linear_x': linear_x_mps,            # m/s, positive = forward
            'angular_z': angular_z_normalized,   # -1..+1, +1 = full left
            'timestamp': time.time()
        }
        message = json.dumps(cmd).encode('utf-8')
        self.sock.sendto(message, (self.target_ip, self.target_port))
        self.messages_sent += 1

    def _send_stop(self):
        """Explicit zero speed + centered steering."""
        try:
            self.send_cmd_vel(0.0, 0.0)
        except OSError:
            pass

    def print_statistics(self):
        if self.start_time is None:
            return
        current_time = time.time()
        if self.last_stats_time is None or current_time - self.last_stats_time >= 5.0:
            elapsed = current_time - self.last_stats_time if self.last_stats_time else current_time - self.start_time
            interval_messages = self.messages_sent - (self.last_stats_count or 0)
            actual_rate = interval_messages / elapsed if elapsed > 0 else 0
            total_elapsed = current_time - self.start_time
            overall_rate = self.messages_sent / total_elapsed if total_elapsed > 0 else 0
            print(f"\n=== SENDER STATISTICS ===")
            print(f"Messages sent: {self.messages_sent}")
            print(f"Target rate: {self.rate_hz:.1f} Hz")
            print(f"Actual rate (5s): {actual_rate:.1f} Hz")
            print(f"Overall rate: {overall_rate:.1f} Hz")
            print(f"Running time: {total_elapsed:.1f} seconds")
            print(f"=========================\n")
            self.last_stats_time = current_time
            self.last_stats_count = self.messages_sent

    # -- run modes ------------------------------------------------------------

    def run_interactive(self):
        """Type each GPS fix by hand. Heading input here is math-frame degrees
        (CCW from east), NOT compass -- see README example."""
        print("Pure Pursuit ready (interactive mode). Enter GPS data to compute and send UDP.")
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

                print(f"Base: x={self.last_x:.3f}m, y={self.last_y:.3f}m, h={math.degrees(self.last_h):.1f}deg")
                print(f"Target idx={self.idx}/{len(self.path)}, delta={math.degrees(delta):.2f}deg, "
                      f"steer={steer_cmd:+.2f}, speed={speed_cmd:.2f} m/s")
                if self.goal_reached:
                    print("Goal reached! Stopping.")
                    break
                self.print_statistics()
            except ValueError:
                print("Invalid numeric input.")
            except Exception as e:
                print(f"Error: {e}")
        self.cleanup()

    def run_timed(self):
        """SIMULATION ONLY -- sends at a fixed rate using the last pose set
        manually. Does NOT read live GPS. Use run_live() for a field test."""
        if not self.path:
            print("No path loaded. Load a mission file first.")
            return
        print(f"[SIMULATION] Starting timed Pure Pursuit sender at {self.rate_hz} Hz")
        print(f"Target UDP: {self.target_ip}:{self.target_port}")
        print("This mode does NOT read live GPS -- it replays the last pose only.")
        print("Press Ctrl+C to stop\n")

        self.start_time = time.time()
        self.last_stats_time = self.start_time
        next_send_time = self.start_time

        try:
            while self.running:
                current_time = time.time()
                if current_time >= next_send_time:
                    delta, v = self.compute_steering(self.last_x, self.last_y, self.last_h)
                    steer_cmd = self.angle_to_normalized(delta)
                    speed_cmd = self.clamp_speed(v)
                    self.send_cmd_vel(speed_cmd, steer_cmd)
                    if self.messages_sent % 100 == 0:
                        print(f"Sent #{self.messages_sent}: speed={speed_cmd:.2f} m/s, "
                              f"steer={steer_cmd:+.2f} (delta={math.degrees(delta):.1f}deg), idx={self.idx}")
                    next_send_time += self.period
                    if next_send_time < current_time:
                        next_send_time = current_time + self.period
                self.print_statistics()
                sleep_time = next_send_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.cleanup()

    def run_live(self, gps_receiver):
        """Real field-test mode. Reads live pose from GPSReceiver every cycle,
        refuses to drive on stale/low-quality/fatal-error data, and stops
        automatically at end of path."""
        if not self.path:
            print("No path loaded. Load a mission file first.")
            return

        print(f"[LIVE] Reading pose from UDP {gps_receiver.port}, "
              f"sending cmd_vel to {self.target_ip}:{self.target_port} at {self.rate_hz} Hz")
        print(f"Gate: min-fix={gps_receiver.min_fix!r}, require headValid={gps_receiver.require_head_valid}")
        print(f"Speed cap: {self.max_speed_mps:.2f} m/s")
        print("Press Ctrl+C to stop\n")

        self.start_time = time.time()
        self.last_stats_time = self.start_time
        loop_count = 0

        try:
            while self.running:
                loop_start = time.time()
                pose = gps_receiver.get_pose()
                ok, reason = gps_receiver.is_drivable(pose)

                if not ok:
                    self._send_stop()
                    if loop_count % 20 == 0:
                        print(f"[WAIT] not driving: {reason}")
                else:
                    self.last_x, self.last_y, self.last_h = self.gps_to_base(
                        pose['lat'], pose['lon'], pose['heading_rad'])
                    delta, v = self.compute_steering(self.last_x, self.last_y, self.last_h)
                    steer_cmd = self.angle_to_normalized(delta)
                    speed_cmd = self.clamp_speed(v)
                    self.send_cmd_vel(speed_cmd, steer_cmd)

                    if loop_count % 20 == 0:
                        print(f"idx={self.idx}/{len(self.path)} x={self.last_x:.2f} y={self.last_y:.2f} "
                              f"h={math.degrees(self.last_h):.1f}deg delta={math.degrees(delta):.1f}deg "
                              f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f}m/s "
                              f"fix={pose['fix_quality']}")

                    if self.goal_reached:
                        print("Goal reached -- sending stop.")
                        self._send_stop()
                        break

                self.print_statistics()
                loop_count += 1
                sleep_time = self.period - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nInterrupted -- stopping.")
        finally:
            self.cleanup()

    def cleanup(self):
        """Sends an explicit stop command before closing the socket, in
        addition to the bridge's own cmd_vel timeout."""
        self.running = False
        self._send_stop()
        if self.start_time:
            total_elapsed = time.time() - self.start_time
            overall_rate = self.messages_sent / total_elapsed if total_elapsed > 0 else 0
            print(f"\n=== FINAL STATISTICS ===")
            print(f"Total messages sent: {self.messages_sent}")
            print(f"Total time: {total_elapsed:.1f} seconds")
            print(f"Average rate: {overall_rate:.1f} Hz")
            print(f"========================")
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description='Pure Pursuit with UDP CMD Vel Sender')
    parser.add_argument('mission_file', nargs='?', help='Mission file path (required for timed/live modes)')
    parser.add_argument('--ip', default='127.0.0.1', help='Target IP for cmd_vel (default: localhost)')
    parser.add_argument('--port', type=int, default=CMD_VEL_UDP_PORT, help='UDP port for cmd_vel (default: 6004)')
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate Hz (default: 20.0)')
    parser.add_argument('--mode', choices=['interactive', 'timed', 'live'], default='interactive',
                        help='interactive (type GPS by hand), timed (simulation replay), live (real GPS via UDP 6002)')
    parser.add_argument('--gps-port', type=int, default=GPS_UDP_PORT, help='UDP port to read GPS from (default: 6002)')
    parser.add_argument('--min-fix', choices=['RTK Fixed', 'RTK Float', 'any'], default='RTK Fixed',
                        help='Minimum fix quality required to drive in live mode (default: RTK Fixed)')
    parser.add_argument('--allow-head-invalid', action='store_true',
                        help='Allow driving even if headValid=False (NOT recommended -- bench test only)')
    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help=f'Safety cap on commanded speed in m/s (default: {DEFAULT_MAX_SPEED_MPS})')
    args = parser.parse_args()

    pp = PurePursuit(
        target_ip=args.ip,
        target_port=args.port,
        rate_hz=args.rate,
        max_speed_mps=args.max_speed,
    )

    if args.mode in ('timed', 'live') and not args.mission_file:
        print(f"Error: --mode {args.mode} requires mission_file argument.")
        sys.exit(1)

    if args.mission_file:
        pp.load_path(args.mission_file)

    gps_receiver = None
    if args.mode == 'live':
        min_fix = None if args.min_fix == 'any' else args.min_fix
        gps_receiver = GPSReceiver(port=args.gps_port, min_fix=min_fix,
                                    require_head_valid=not args.allow_head_invalid)

    def signal_handler(sig, frame):
        pp.cleanup()
        if gps_receiver:
            gps_receiver.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    if args.mode == 'interactive':
        pp.run_interactive()
    elif args.mode == 'timed':
        pp.run_timed()
    else:
        pp.run_live(gps_receiver)
        gps_receiver.stop()


if __name__ == "__main__":
    main()
