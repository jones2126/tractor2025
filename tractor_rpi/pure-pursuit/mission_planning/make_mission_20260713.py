#!/usr/bin/env python3
"""
Generate a Pure Pursuit mission file: A -> B -> (teardrop turn) -> A.

A forward-only Ackermann vehicle cannot reverse course in place, so the
turnaround at B is a "teardrop" loop: the tractor drives to B, curves around a
circle of radius R off to one side, and rejoins the A-B line heading back to A.

Geometry (local frame at B, +f = A->B direction, +l = left of travel):
  - The loop circle is centered at C = (0, R) (turn side = left; flip for right).
  - The outbound A->B line is tangent to that circle exactly at B, so entry is
    smooth. The vehicle sweeps CCW around C by (360 - 2*theta_t) degrees, where
    theta_t = acos(R / dist(A,C)), exiting at the point Ex where the circle's
    tangent points straight at A. It then drives the straight Ex->A home.
  - Loop clearance: bulges ~2R to the turn side and ~R beyond B.

Output line format (consumed by pure_pursuit_grok_20251112.py):
    lat lon yaw_rad lookahead_m speed_mps
yaw_rad is in the script's math frame: CCW from EAST (not a compass heading).
"""
import math

# --- endpoints (decimal degrees) --------------------------------------------
# 40°29'08.0"N 80°19'57.1"W  (start / home = A)
A = (40.0 + 29.0 / 60.0 + 8.0 / 3600.0, -(80.0 + 19.0 / 60.0 + 57.1 / 3600.0))
# 40°29'06.9"N 80°19'56.5"W  (far point = B)
B = (40.0 + 29.0 / 60.0 + 6.9 / 3600.0, -(80.0 + 19.0 / 60.0 + 56.5 / 3600.0))

TURN_RADIUS_M = 3.0    # loop radius (min feasible ~1.77 m; 3.0 = comfortable)
TURN_SIDE = "left"     # "left" or "right" — which way the teardrop bulges
SPACING_M = 1.0        # waypoint spacing along straights and the arc

# per-waypoint lookahead / speed (arc gets tighter lookahead + slower speed)
LD_STRAIGHT, V_STRAIGHT = 3.0, 0.5
LD_ARC, V_ARC = 1.5, 0.3

M_PER_DEG_LAT = 110540.0
OUT_FILE = "mission_out_and_back_20260713.txt"


def m_per_deg_lon(lat_deg):
    return 111320.0 * math.cos(math.radians(lat_deg))


def main():
    latA, lonA = A
    latB, lonB = B
    mlat = M_PER_DEG_LAT
    mlon = m_per_deg_lon((latA + latB) / 2.0)

    # ENU meters (origin at A). B relative to A:
    Be = (lonB - lonA) * mlon
    Bn = (latB - latA) * mlat
    D = math.hypot(Be, Bn)                 # leg length A->B
    ue, un = Be / D, Bn / D                # unit vector A->B (forward)
    sgn = 1.0 if TURN_SIDE == "left" else -1.0
    ne, nn = -un * sgn, ue * sgn           # unit vector to the turn side

    R = TURN_RADIUS_M
    r_min = 1.27 / math.tan(0.623)         # steering-limited min radius

    # local (f, l) -> ENU east/north, referenced to B
    def fl_to_en(f, l):
        return Be + f * ue + l * ne, Bn + f * un + l * nn

    def en_to_latlon(e, n):
        return latA + n / mlat, lonA + e / mlon

    def yaw_of(de, dn):                     # ENU direction -> CCW-from-east
        return math.atan2(dn, de)

    rows = []  # (lat, lon, yaw, ld, v)

    def add(e, n, yaw, ld, v):
        lat, lon = en_to_latlon(e, n)
        rows.append((lat, lon, yaw, ld, v))

    def straight(p0e, p0n, p1e, p1n, ld, v, include_start):
        seg = math.hypot(p1e - p0e, p1n - p0n)
        yaw = yaw_of(p1e - p0e, p1n - p0n)
        steps = max(1, int(round(seg / SPACING_M)))
        for i in range(0 if include_start else 1, steps + 1):
            fr = i / steps
            add(p0e + (p1e - p0e) * fr, p0n + (p1n - p0n) * fr, yaw, ld, v)
        return seg

    # --- 1) outbound straight A -> B ---------------------------------------
    Ae, An = 0.0, 0.0
    d_out = straight(Ae, An, Be, Bn, LD_STRAIGHT, V_STRAIGHT, include_start=True)

    # --- 2) teardrop arc around C = B + R*n --------------------------------
    Ce, Cn = fl_to_en(0.0, R)              # loop center
    dCA = math.hypot(Ae - Ce, An - Cn)     # distance center -> A
    theta_t = math.acos(R / dCA)           # half-angle to tangent point
    sweep = 2.0 * math.pi - 2.0 * theta_t  # CCW sweep (radians), >180 deg
    arc_len = R * sweep
    steps = max(2, int(round(arc_len / SPACING_M)))
    for i in range(1, steps + 1):
        a = math.radians(-90.0) + sgn * sweep * (i / steps)  # sgn flips CCW/CW
        e, n = fl_to_en(R * math.cos(a), R * (1.0 + math.sin(a)))
        te = -R * math.sin(a) * sgn        # local tangent (d fl_f, d fl_l)
        tn = R * math.cos(a) * sgn
        de = te * ue + tn * ne             # rotate local tangent into ENU
        dn = te * un + tn * nn
        add(e, n, yaw_of(de, dn), LD_ARC, V_ARC)

    a_end = math.radians(-90.0) + sgn * sweep
    Ex_e, Ex_n = fl_to_en(R * math.cos(a_end), R * (1.0 + math.sin(a_end)))

    # --- 3) return straight Ex -> A ----------------------------------------
    d_ret = straight(Ex_e, Ex_n, Ae, An, LD_STRAIGHT, V_STRAIGHT, include_start=False)

    with open(OUT_FILE, "w") as f:
        for lat, lon, yaw, ld, v in rows:
            f.write(f"{lat:.8f} {lon:.8f} {yaw:.6f} {ld:.2f} {v:.2f}\n")

    # --- diagnostics --------------------------------------------------------
    final_lat, final_lon = rows[-1][0], rows[-1][1]
    err_m = math.hypot((final_lon - lonA) * mlon, (final_lat - latA) * mlat)
    print(f"Leg A->B      : {d_out:.1f} m")
    print(f"Teardrop      : R={R:.2f} m (min feasible {r_min:.2f} m), "
          f"sweep={math.degrees(sweep):.1f} deg, arc={arc_len:.1f} m, side={TURN_SIDE}")
    print(f"Return Ex->A  : {d_ret:.1f} m")
    print(f"Feasible turn : {'YES' if R >= r_min else 'NO — R < min radius!'}")
    print(f"End vs A error: {err_m*100:.1f} cm")
    print(f"Wrote {len(rows)} waypoints to {OUT_FILE}")


if __name__ == "__main__":
    main()
