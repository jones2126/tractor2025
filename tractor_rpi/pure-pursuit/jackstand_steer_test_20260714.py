#!/usr/bin/env python3
"""
jackstand_steer_test_20260714.py -- end-to-end steering verification on jack stands.

Exercises the FULL live chain using the real controller code:
  UDP 6002 (rtcm_server) -> GPSReceiver -> compass->math heading -> latlon->xy
  -> PurePursuit.compute_steering() -> angle_to_normalized() -> UDP 6004
  -> teensy_serial_bridge -> Teensy mode 2 -> mapNormalizedSteer() -> wheels

Requires pure_pursuit_controller_20260714.py in the same directory (it is
imported, not copied -- so this tests the code that will fly the mission).

What it does:
  1. Displays live UDP 6002 data (lat/lon/heading/fix) until you confirm.
  2. Snapshots the pose, synthesizes 5 single-waypoint test paths RELATIVE
     to the actual live heading: dead ahead, slight right/left (+-8 deg),
     hard right/left (+-45 deg), each 6 m out.
  3. For each case: prints the expected steer command and expected Teensy
     pot setpoint (197/447/815 map), then streams cmd_vel at 10 Hz
     (linear_x = 0.0 -- no wheel drive) until you press Enter. Pose is
     re-read live every cycle. Between cases it streams center (0.0).
  4. On exit / Ctrl+C: streams a stop, then quits.

Safety notes:
  - linear_x is ALWAYS 0.0 in this script; only steering is commanded.
  - RC mode switch must be in AUTO (mode 2) for commands to take effect.
  - Firmware cmd_vel timeout is 500 ms; this script streams continuously
    so the setpoint holds. Killing the script -> Teensy holds position,
    transmission stays neutral.

Usage (on tractor01, rtcm-server + teensy-bridge services running):
    python3 jackstand_steer_test_20260714.py
    python3 jackstand_steer_test_20260714.py --min-fix "RTK Float"   # if Fixed is slow today
"""

import math
import time
import threading
import argparse
import sys
import importlib.util
import os

CONTROLLER_FILE = "pure_pursuit_controller_20260714.py"

# Teensy pot calibration -- FOR DISPLAY ONLY (predicting the sp= value you
# should see on the Teensy STEER status line). The authoritative map lives
# in teensy_main_20260714.cpp::mapNormalizedSteer().
POT_RIGHT, POT_CENTER, POT_LEFT = 197, 447, 815

# Test cases: (name, bearing offset deg relative to heading (+ = RIGHT/clockwise),
#              distance m, waypoint lookahead m)
TEST_CASES = [
    ("1. DEAD AHEAD (wheels should center)",        0.0, 6.0, 3.0),
    ("2. SLIGHT RIGHT (+8 deg off heading)",        8.0, 6.0, 3.0),
    ("3. HARD RIGHT (+45 deg -- expect full lock)", 45.0, 6.0, 3.0),
    ("4. SLIGHT LEFT (-8 deg off heading)",         -8.0, 6.0, 3.0),
    ("5. HARD LEFT (-45 deg -- expect full lock)",  -45.0, 6.0, 3.0),
]


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


def main():
    ap = argparse.ArgumentParser(description="Jack-stands steering verification")
    ap.add_argument('--ip', default='127.0.0.1', help='cmd_vel target IP (default localhost)')
    ap.add_argument('--port', type=int, default=6004, help='cmd_vel UDP port (default 6004)')
    ap.add_argument('--gps-port', type=int, default=6002, help='GPS UDP port (default 6002)')
    ap.add_argument('--min-fix', choices=['RTK Fixed', 'RTK Float', 'any'], default='RTK Fixed',
                    help='Fix quality required before running cases (default: RTK Fixed)')
    args = ap.parse_args()

    mod = load_controller()
    min_fix = None if args.min_fix == 'any' else args.min_fix

    # Real controller objects -- same classes the mission will use.
    pp = mod.PurePursuit(target_ip=args.ip, target_port=args.port)
    gps = mod.GPSReceiver(port=args.gps_port, min_fix=min_fix, require_head_valid=True)

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
        # --- Phase A: show live 6002 until fix is acceptable ----------------
        print("=" * 70)
        print("PHASE A: live UDP 6002 display -- waiting for acceptable fix")
        print(f"         (need {args.min_fix}, headValid=True, fresh <0.5 s)")
        print("=" * 70)
        while True:
            pose = gps.get_pose()
            ok, reason = gps.is_drivable(pose)
            if pose is None:
                print("  no 6002 data yet ... is rtcm-server running?")
            else:
                hd_compass = (90.0 - math.degrees(pose['heading_rad'])) % 360.0
                print(f"  lat={pose['lat']:.8f} lon={pose['lon']:.8f} "
                      f"heading={hd_compass:.1f} deg(compass) fix={pose['fix_quality']} "
                      f"headValid={pose['headValid']} age={pose['age']:.2f}s "
                      f"{'OK' if ok else '-- ' + reason}")
            if ok:
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
            while not stop_evt.is_set():
                live = gps.get_pose()
                ok, reason = gps.is_drivable(live)
                if not ok:
                    current_cmd['steer'] = 0.0
                    if time.time() - last_print > 1.0:
                        print(f"    [WAIT] {reason} -- steering centered")
                        last_print = time.time()
                else:
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
        pp.sock.close()
        print("Stopped -- final center/stop sent.")


if __name__ == "__main__":
    main()
