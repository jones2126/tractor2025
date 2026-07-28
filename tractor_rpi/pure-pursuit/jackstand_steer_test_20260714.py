#!/usr/bin/env python3
"""
jackstand_steer_test_20260714.py -- end-to-end steering verification on jack stands.

v2 (same date): adds RC MODE MONITORING via UDP 6003.
  NEW   ModeMonitor class -- listens on UDP 6003 (teensy bridge status relay),
        extracts the control mode from STEER/TRANS/RADIO status lines
        (handles both raw text lines like "1,12345,STEER,m=2,sp=447,..."
        and JSON-wrapped variants). Modes: 0=Pause, 1=Manual, 2=Auto,
        9=NO RADIO SIGNAL (firmware safety state).
  NEW   Phase A now displays the current mode next to the GPS fix.
  NEW   Each test case is GATED on mode 2 (Auto): if the switch is in
        Manual/Pause, the script tells you and waits for you to flip it
        instead of silently streaming commands nothing is listening to.
  NEW   If the mode changes away from Auto MID-case, a warning prints
        immediately and steering command centers until Auto returns.
  If no 6003 data arrives at all, the script says so (bridge not running /
  different port?) and lets you proceed with mode UNKNOWN after confirming.

Exercises the FULL live chain using the real controller code:
  UDP 6002 (rtcm_server) -> GPSReceiver -> compass->math heading -> latlon->xy
  -> PurePursuit.compute_steering() -> angle_to_normalized() -> UDP 6004
  -> teensy_serial_bridge -> Teensy mode 2 -> mapNormalizedSteer() -> wheels

Requires pure_pursuit_controller_20260714.py in the same directory (it is
imported, not copied -- so this tests the code that will fly the mission).

Safety notes:
  - linear_x is ALWAYS 0.0 in this script; only steering is commanded.
  - Firmware cmd_vel timeout is 500 ms; this script streams continuously
    so the setpoint holds. Killing the script -> Teensy holds position,
    transmission stays neutral.

Usage (on tractor01, rtcm-server + teensy-bridge services running):
    python3 jackstand_steer_test_20260714.py
    python3 jackstand_steer_test_20260714.py --min-fix "RTK Float"
    python3 jackstand_steer_test_20260714.py --status-port 6003
"""

import math
import re
import json
import time
import socket
import threading
import argparse
import sys
import importlib.util
import os

CONTROLLER_FILE = "pure_pursuit_controller_20260714.py"

# Teensy pot calibration -- FOR DISPLAY ONLY (predicting the sp= value you
# should see on the Teensy STEER status line). The authoritative map lives
# in teensy_main_20260728.cpp::mapNormalizedSteer().
POT_RIGHT, POT_CENTER, POT_LEFT = 197, 447, 815

MODE_NAMES = {0: "AUTO", 1: "MANUAL", 2: "PAUSE", 9: "NO RADIO SIGNAL"}

# How stale a mode report can be before we call it unknown. The Teensy
# prints STEER/TRANS status continuously, so a few seconds of silence
# means the bridge relay (or the Teensy) has gone quiet.
MODE_STALE_S = 3.0

# Test cases: (name, bearing offset deg relative to heading (+ = RIGHT/clockwise),
#              distance m, waypoint lookahead m)
TEST_CASES = [
    ("1. DEAD AHEAD (wheels should center)",        0.0, 6.0, 3.0),
    ("2. SLIGHT RIGHT (+8 deg off heading)",        8.0, 6.0, 3.0),
    ("3. HARD RIGHT (+45 deg -- expect full lock)", 45.0, 6.0, 3.0),
    ("4. SLIGHT LEFT (-8 deg off heading)",         -8.0, 6.0, 3.0),
    ("5. HARD LEFT (-45 deg -- expect full lock)",  -45.0, 6.0, 3.0),
]

# Matches m=<digits> on Teensy status lines (STEER/TRANS/RADIO all carry it).
_MODE_TEXT_RE = re.compile(r'\b(?:STEER|TRANS|RADIO)\b.*?\bm=(\d+)')


class ModeMonitor:
    """Listens on UDP 6003 for teensy bridge status and tracks the RC mode.

    Tolerant parser: the bridge may relay raw Teensy text lines
    ("1,12345,STEER,m=2,sp=447,...") or wrap status in JSON. Handles both:
      - text: regex m=<n> on lines mentioning STEER/TRANS/RADIO
      - JSON: recursive search for keys 'm', 'mode', 'control_mode' (int),
              plus regex over any string values (relayed raw lines).
    """

    def __init__(self, port=6003):
        self.port = port
        self._lock = threading.Lock()
        self._mode = None
        self._mode_time = 0.0
        self._packets = 0
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
                data, _addr = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            self._packets += 1
            try:
                text = data.decode('utf-8', errors='replace')
            except Exception:
                continue
            mode = self._extract_mode(text)
            if mode is not None:
                with self._lock:
                    self._mode = mode
                    self._mode_time = time.time()

    def _extract_mode(self, text):
        # Try JSON first (bridge may wrap status in JSON)
        stripped = text.strip()
        if stripped.startswith('{') or stripped.startswith('['):
            try:
                obj = json.loads(stripped)
                m = self._search_json(obj)
                if m is not None:
                    return m
            except ValueError:
                pass
        # Raw / relayed text lines
        match = _MODE_TEXT_RE.search(text)
        if match:
            try:
                return int(match.group(1))
            except ValueError:
                return None
        return None

    def _search_json(self, obj):
        """Recursive search for a mode value in a JSON structure."""
        if isinstance(obj, dict):
            for key in ('control_mode', 'mode', 'm'):
                v = obj.get(key)
                if isinstance(v, int) and v in MODE_NAMES:
                    return v
                if isinstance(v, str) and v.isdigit() and int(v) in MODE_NAMES:
                    return int(v)
            for v in obj.values():
                m = self._search_json(v)
                if m is not None:
                    return m
        elif isinstance(obj, list):
            for v in obj:
                m = self._search_json(v)
                if m is not None:
                    return m
        elif isinstance(obj, str):
            match = _MODE_TEXT_RE.search(obj)
            if match:
                return int(match.group(1))
        return None

    def get_mode(self):
        """(mode_int_or_None, description_string). None = unknown/stale."""
        with self._lock:
            mode, mtime, pkts = self._mode, self._mode_time, self._packets
        if mode is None:
            if pkts == 0:
                return None, f"UNKNOWN (no data on UDP {self.port} -- teensy-bridge running?)"
            return None, f"UNKNOWN (data on {self.port} but no mode found in it yet)"
        age = time.time() - mtime
        if age > MODE_STALE_S:
            return None, f"UNKNOWN (last mode {MODE_NAMES.get(mode, mode)} was {age:.0f}s ago -- stale)"
        return mode, MODE_NAMES.get(mode, f"mode {mode}?")

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


def load_controller():
    """Import PurePursuit + GPSReceiver from the real controller file."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), CONTROLLER_FILE)
    if not os.path.exists(path):
        print(f"ERROR: {CONTROLLER_FILE} not found next to this script.")
        sys.exit(1)
    spec = importlib.util.spec_from_file_location("pp_controller", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def offset_latlon(lat, lon, compass_bearing_deg, dist_m):
    """Point dist_m away at compass bearing (deg from true north)."""
    b = math.radians(compass_bearing_deg)
    north_m = dist_m * math.cos(b)
    east_m = dist_m * math.sin(b)
    dlat = north_m / 110540.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def expected_pot(steer_norm):
    """Mirror of firmware mapNormalizedSteer() -- display only."""
    n = max(-1.0, min(1.0, steer_norm))
    if n >= 0:
        return int(round(POT_CENTER + n * (POT_LEFT - POT_CENTER)))
    return int(round(POT_CENTER + n * (POT_CENTER - POT_RIGHT)))


def require_auto(modes, allow_unknown=False):
    """Block until the RC switch reads AUTO (mode 2). Returns True to proceed,
    False if the user chose to proceed with mode UNKNOWN (only when allowed)."""
    warned_unknown = False
    while True:
        mode, desc = modes.get_mode()
        if mode == 0:
            return True
        if mode is None and allow_unknown:
            if not warned_unknown:
                print(f"    RC mode: {desc}")
                ans = input("    Mode unknown -- proceed anyway? Commands only work "
                            "in AUTO. (y/N): ").strip().lower()
                if ans == 'y':
                    return False
                warned_unknown = True
            time.sleep(1.0)
            continue
        print(f"    RC mode is {desc} -- flip the remote switch to AUTO (mode 2). "
              f"Waiting...")
        # poll until it changes
        while True:
            time.sleep(1.0)
            m2, d2 = modes.get_mode()
            if m2 == 0:
                print("    RC mode: AUTO -- good.")
                return True
            if (m2, d2) != (mode, desc):
                mode, desc = m2, d2
                print(f"    RC mode now: {desc}")
                break  # re-evaluate in outer loop


def main():
    ap = argparse.ArgumentParser(description="Jack-stands steering verification")
    ap.add_argument('--ip', default='127.0.0.1', help='cmd_vel target IP (default localhost)')
    ap.add_argument('--port', type=int, default=6004, help='cmd_vel UDP port (default 6004)')
    ap.add_argument('--gps-port', type=int, default=6002, help='GPS UDP port (default 6002)')
    ap.add_argument('--status-port', type=int, default=6003,
                    help='Teensy bridge status UDP port for mode monitoring (default 6003)')
    ap.add_argument('--min-fix', choices=['RTK Fixed', 'RTK Float', 'any'], default='RTK Fixed',
                    help='Fix quality required before running cases (default: RTK Fixed)')
    args = ap.parse_args()

    mod = load_controller()
    min_fix = None if args.min_fix == 'any' else args.min_fix

    # Real controller objects -- same classes the mission will use.
    pp = mod.PurePursuit(target_ip=args.ip, target_port=args.port)
    gps = mod.GPSReceiver(port=args.gps_port, min_fix=min_fix, require_head_valid=True)
    modes = ModeMonitor(port=args.status_port)

    # --- Streaming thread: sends current (linear_x, steer) at 10 Hz --------
    current_cmd = {'steer': 0.0}
    streaming = {'on': True}

    def stream():
        while streaming['on']:
            pp.send_cmd_vel(0.0, current_cmd['steer'])   # linear_x ALWAYS 0.0
            time.sleep(0.1)

    t = threading.Thread(target=stream, daemon=True)
    t.start()

    try:
        # --- Phase A: show live 6002 + RC mode until fix is acceptable -------
        print("=" * 70)
        print("PHASE A: live UDP 6002 display -- waiting for acceptable fix")
        print(f"         (need {args.min_fix}, headValid=True, fresh <0.5 s)")
        print(f"         RC mode monitored on UDP {args.status_port} "
              f"(0=Pause 1=Manual 2=Auto 9=no radio)")
        print("=" * 70)
        while True:
            pose = gps.get_pose()
            ok, reason = gps.is_drivable(pose)
            mode, mode_desc = modes.get_mode()
            if pose is None:
                print(f"  no 6002 data yet ... is rtcm-server running?   "
                      f"| RC mode: {mode_desc}")
            else:
                hd_compass = (90.0 - math.degrees(pose['heading_rad'])) % 360.0
                print(f"  lat={pose['lat']:.8f} lon={pose['lon']:.8f} "
                      f"heading={hd_compass:.1f} deg(compass) fix={pose['fix_quality']} "
                      f"headValid={pose['headValid']} age={pose['age']:.2f}s "
                      f"{'OK' if ok else '-- ' + reason}  | RC mode: {mode_desc}")
            if ok:
                if mode != 2:
                    print(f"  NOTE: GPS is ready but RC mode is {mode_desc} -- "
                          f"steering commands only act in AUTO.")
                ans = input("Fix acceptable. Press Enter to snapshot pose and build "
                            "test targets (or 'w' + Enter to keep watching): ").strip().lower()
                if ans != 'w':
                    break
            else:
                time.sleep(1.0)

        # --- Phase B: snapshot + synthesize targets --------------------------
        pose = gps.get_pose()
        lat0, lon0 = pose['lat'], pose['lon']
        heading_compass = (90.0 - math.degrees(pose['heading_rad'])) % 360.0
        print()
        print("=" * 70)
        print(f"PHASE B: snapshot  lat={lat0:.8f}  lon={lon0:.8f}  "
              f"heading={heading_compass:.1f} deg compass")
        print("Targets are placed relative to THIS heading (not your estimate).")
        print("=" * 70)

        cases = []
        for name, off_deg, dist, ld in TEST_CASES:
            tlat, tlon = offset_latlon(lat0, lon0, heading_compass + off_deg, dist)
            cases.append((name, off_deg, dist, ld, tlat, tlon))
            print(f"  {name}")
            print(f"      target: {tlat:.8f}, {tlon:.8f}  ({dist:.0f} m at "
                  f"{(heading_compass + off_deg) % 360:.1f} deg compass)")
        print()
        print("Watch the front wheels AND the Teensy STEER line on UDP 6003")
        print("(sp= should match 'expect pot' below; c= should converge to it).")
        print()

        # --- Phase C: run cases ----------------------------------------------
        for name, off_deg, dist, ld, tlat, tlon in cases:
            # NEW: gate every case on AUTO mode
            require_auto(modes, allow_unknown=True)

            input(f"--> Press Enter to START: {name} ")

            # Build a one-waypoint path in the REAL controller. ref lat/lon =
            # current pose so the local frame is anchored here.
            pp.ref_lat, pp.ref_lon = lat0, lon0
            tx, ty = pp.latlon_to_xy(tlat, tlon)
            pp.path = [(tx, ty, 0.0, ld, 0.0)]   # speed 0 in the waypoint too
            pp.idx = 0
            pp.goal_reached = False

            print(f"    streaming at 10 Hz -- press Enter to stop this case")
            stop_evt = threading.Event()

            def wait_enter():
                input()
                stop_evt.set()

            w = threading.Thread(target=wait_enter, daemon=True)
            w.start()

            last_print = 0.0
            mode_warned = False
            while not stop_evt.is_set():
                live = gps.get_pose()
                ok, reason = gps.is_drivable(live)
                mode, mode_desc = modes.get_mode()

                # NEW: live mid-case mode check
                if mode is not None and mode != 0:
                    current_cmd['steer'] = 0.0
                    if not mode_warned or time.time() - last_print > 2.0:
                        print(f"    [MODE] RC switch is {mode_desc} -- commands "
                              f"ignored by Teensy. Steering centered until AUTO returns.")
                        last_print = time.time()
                        mode_warned = True
                elif not ok:
                    current_cmd['steer'] = 0.0
                    if time.time() - last_print > 1.0:
                        print(f"    [WAIT] {reason} -- steering centered")
                        last_print = time.time()
                else:
                    if mode_warned:
                        print("    [MODE] AUTO restored -- resuming case.")
                        mode_warned = False
                    bx, by, bh = pp.gps_to_base(live['lat'], live['lon'], live['heading_rad'])
                    delta, _v = pp.compute_steering(bx, by, bh)
                    steer = pp.angle_to_normalized(delta)
                    current_cmd['steer'] = steer
                    if time.time() - last_print > 1.0:
                        print(f"    delta={math.degrees(delta):+6.1f} deg  "
                              f"steer={steer:+.3f}  expect pot ~{expected_pot(steer)}  "
                              f"(447=center, 815=full L, 197=full R)")
                        last_print = time.time()
                time.sleep(0.1)

            current_cmd['steer'] = 0.0
            print("    case ended -- streaming CENTER. Confirm wheels return "
                  "to straight before the next case.\n")
            time.sleep(1.0)

        print("=" * 70)
        print("All cases done.")
        print("PASS = case 2/3 turned wheels RIGHT (pot toward 197),")
        print("       case 4/5 turned wheels LEFT (pot toward 815),")
        print("       case 3/5 reached ~full lock, case 1 stayed centered,")
        print("       wheels re-centered between every case.")
        print("If direction is REVERSED: stop. Fix the sign in ONE place")
        print("(firmware mapNormalizedSteer) and re-run. Never patch both sides.")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        current_cmd['steer'] = 0.0
        time.sleep(0.3)                 # let the stream thread send center
        streaming['on'] = False
        time.sleep(0.2)
        pp._send_stop()
        gps.stop()
        modes.stop()
        pp.sock.close()
        print("Stopped -- final center/stop sent.")


if __name__ == "__main__":
    main()
