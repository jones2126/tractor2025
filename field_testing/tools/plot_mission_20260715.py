#!/usr/bin/env python3
"""
plot_mission_20260715.py
=========================
Reads a Pure Pursuit mission file (lat lon yaw_rad lookahead_m speed_mps,
one waypoint per line) and produces an interactive HTML map of the PLANNED
path -- before ever driving it.

This is the "plan" counterpart to analyze_field_test_20260710.py, which plots
the ACTUAL path driven from a field_test_logger CSV. Same map style (Folium,
CartoDB Positron tiles, zoom_start=21, max_zoom=22) so the two are visually
comparable side by side.

Produces:
  mission_map.html   - interactive map: waypoints colored by commanded speed,
                        heading arrows at intervals, start/end markers

Usage:
  python3 plot_mission_20260715.py mission_out_and_back_20260713.txt
  python3 plot_mission_20260715.py mission_out_and_back_20260713.txt --out /tmp/results
  python3 plot_mission_20260715.py mission_out_and_back_20260713.txt --arrow-every 3
"""

import argparse
import math
import os
import sys

import folium


# ---------------------------------------------------------------------------
# Speed color mapping -- a red->amber->green ramp scaled to whatever speeds
# are actually in the file, rather than a fixed dict, since mission files
# can command any speed_mps value (not just the 0.5/0.3 straight/arc
# convention used so far).
# ---------------------------------------------------------------------------
def speed_color(v, v_min, v_max):
    """Green = fastest commanded speed in this file, red = slowest.
    Degenerate case (all one speed) returns green."""
    if v_max <= v_min:
        return '#00cc44'
    frac = (v - v_min) / (v_max - v_min)  # 0 = slowest, 1 = fastest
    if frac < 0.5:
        t = frac / 0.5
        r, g, b = 0xcc, int(0x00 + t * 0xaa), 0x00
    else:
        t = (frac - 0.5) / 0.5
        r, g, b = int(0xcc - t * 0xcc), int(0xaa + t * (0xcc - 0xaa)), int(0x00 + t * 0x44)
    return f'#{r:02x}{g:02x}{b:02x}'


# ---------------------------------------------------------------------------
# Load mission file
# ---------------------------------------------------------------------------
def load_mission(path):
    """Parse 'lat lon yaw_rad lookahead_m speed_mps' lines into a list of dicts.
    Mirrors PurePursuit.load_path()'s parsing/skip-invalid-line behavior so a
    plotted mission always matches what the controller would actually load."""
    waypoints = []
    with open(path, 'r') as f:
        lines = f.readlines()

    for line_num, line in enumerate(lines, 1):
        parts = line.strip().split()
        if len(parts) != 5:
            print(f"Warning: Skipping invalid line {line_num}: {line.strip()}")
            continue
        try:
            lat, lon, yaw, ld, v = map(float, parts)
        except ValueError:
            print(f"Warning: Skipping invalid numeric line {line_num}: {line.strip()}")
            continue
        waypoints.append({
            'idx': len(waypoints),
            'lat': lat,
            'lon': lon,
            'yaw_rad': yaw,
            'yaw_deg': math.degrees(yaw),
            'lookahead_m': ld,
            'speed_mps': v,
        })

    print(f"Loaded {len(waypoints)} waypoints from {os.path.basename(path)}")
    return waypoints


# ---------------------------------------------------------------------------
# Heading arrow helper -- projects a short segment from a waypoint in the
# direction of yaw_rad (math-frame: radians, CCW from east), using the same
# equirectangular approximation as PurePursuit.latlon_to_xy(), run in
# reverse to turn a meter offset back into a lat/lon delta.
# ---------------------------------------------------------------------------
def project_forward(lat, lon, yaw_rad, dist_m):
    dx = dist_m * math.cos(yaw_rad)   # +x = east
    dy = dist_m * math.sin(yaw_rad)   # +y = north
    lat0_rad = math.radians(lat)
    dlon_deg = dx / (111320.0 * math.cos(lat0_rad))
    dlat_deg = dy / 110540.0
    return lat + dlat_deg, lon + dlon_deg


# ---------------------------------------------------------------------------
# Build the map
# ---------------------------------------------------------------------------
def plot_mission(waypoints, out_dir, arrow_every=5, arrow_len_m=1.0):
    if not waypoints:
        print("No waypoints to plot.")
        return None

    lats = [w['lat'] for w in waypoints]
    lons = [w['lon'] for w in waypoints]
    speeds = [w['speed_mps'] for w in waypoints]
    v_min, v_max = min(speeds), max(speeds)

    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)

    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=21,
        tiles='CartoDB positron',
        max_zoom=22,
    )

    # Thin connecting line showing the planned path
    coords = list(zip(lats, lons))
    folium.PolyLine(coords, color='#333333', weight=1.5, opacity=0.5).add_to(m)

    # One FeatureGroup per rounded speed value so legs can be toggled
    # (e.g. hide the fast straight legs to inspect just a turn in isolation)
    speed_groups = {}
    for w in waypoints:
        key = round(w['speed_mps'], 2)
        if key not in speed_groups:
            fg = folium.FeatureGroup(name=f'{key:.2f} m/s')
            speed_groups[key] = fg
            m.add_child(fg)

    for w in waypoints:
        key = round(w['speed_mps'], 2)
        color = speed_color(w['speed_mps'], v_min, v_max)
        tooltip = (
            f"<b>Waypoint {w['idx']}</b><br>"
            f"Lat: {w['lat']:.8f}<br>"
            f"Lon: {w['lon']:.8f}<br>"
            f"Yaw: {w['yaw_deg']:.1f}&deg; (math frame)<br>"
            f"Lookahead: {w['lookahead_m']:.2f} m<br>"
            f"Speed: {w['speed_mps']:.2f} m/s"
        )
        folium.CircleMarker(
            location=[w['lat'], w['lon']],
            radius=3,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.9,
            tooltip=tooltip,
        ).add_to(speed_groups[key])

    # Heading arrows at intervals -- short segment in the yaw direction, so a
    # heading reversal or bad yaw value in the mission file is visible at a
    # glance rather than buried in a tooltip.
    arrow_group = folium.FeatureGroup(name=f'Heading (every {arrow_every})')
    m.add_child(arrow_group)
    for w in waypoints[::arrow_every]:
        end_lat, end_lon = project_forward(w['lat'], w['lon'], w['yaw_rad'], arrow_len_m)
        folium.PolyLine(
            [[w['lat'], w['lon']], [end_lat, end_lon]],
            color='#3366ff',
            weight=2,
            opacity=0.8,
        ).add_to(arrow_group)

    # Start / end markers
    folium.Marker(
        [waypoints[0]['lat'], waypoints[0]['lon']],
        popup='Start',
        icon=folium.Icon(color='green', icon='play'),
    ).add_to(m)
    folium.Marker(
        [waypoints[-1]['lat'], waypoints[-1]['lon']],
        popup='End',
        icon=folium.Icon(color='red', icon='stop'),
    ).add_to(m)

    # Legend
    legend_html = (
        '<div style="position:fixed;bottom:30px;left:30px;z-index:1000;'
        'background:rgba(0,0,0,0.75);padding:10px;border-radius:8px;'
        'color:white;font-size:12px;">'
        '<b>Commanded Speed</b><br>'
        f'<span style="display:inline-block;width:12px;height:12px;'
        f'background:{speed_color(v_max, v_min, v_max)};border-radius:50%;'
        f'margin-right:4px;vertical-align:middle;"></span>Fastest ({v_max:.2f} m/s)<br>'
        f'<span style="display:inline-block;width:12px;height:12px;'
        f'background:{speed_color(v_min, v_min, v_max)};border-radius:50%;'
        f'margin-right:4px;vertical-align:middle;"></span>Slowest ({v_min:.2f} m/s)<br>'
        '<span style="display:inline-block;width:12px;height:2px;'
        'background:#3366ff;margin-right:4px;vertical-align:middle;"></span>Heading arrow<br>'
        '</div>'
    )
    m.get_root().html.add_child(folium.Element(legend_html))

    folium.LayerControl().add_to(m)

    out_path = os.path.join(out_dir, 'mission_map.html')
    m.save(out_path)
    print(f"  Saved: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
def print_summary(waypoints, path):
    speeds = [w['speed_mps'] for w in waypoints]
    lookaheads = [w['lookahead_m'] for w in waypoints]
    lats = [w['lat'] for w in waypoints]
    lons = [w['lon'] for w in waypoints]
    print(f"\nMission Summary: {os.path.basename(path)}")
    print("=" * 50)
    print(f"Waypoints:        {len(waypoints)}")
    print(f"Speed range:      {min(speeds):.2f} - {max(speeds):.2f} m/s")
    print(f"Lookahead range:  {min(lookaheads):.2f} - {max(lookaheads):.2f} m")
    print(f"Start:            {waypoints[0]['lat']:.8f}, {waypoints[0]['lon']:.8f}")
    print(f"End:              {waypoints[-1]['lat']:.8f}, {waypoints[-1]['lon']:.8f}")
    print(f"Lat range:        {min(lats):.8f} -> {max(lats):.8f}")
    print(f"Lon range:        {min(lons):.8f} -> {max(lons):.8f}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Plot a Pure Pursuit mission file on an interactive map')
    parser.add_argument('mission_file', help='Path to mission .txt file (lat lon yaw_rad lookahead_m speed_mps)')
    parser.add_argument('--out', '-o', default=None, help='Output directory (default: same dir as mission file)')
    parser.add_argument('--arrow-every', type=int, default=5, help='Draw a heading arrow every N waypoints (default: 5)')
    parser.add_argument('--arrow-len', type=float, default=1.0, help='Heading arrow length in meters (default: 1.0)')
    args = parser.parse_args()

    if not os.path.exists(args.mission_file):
        print(f"Error: file not found: {args.mission_file}")
        sys.exit(1)

    out_dir = args.out or os.path.dirname(os.path.abspath(args.mission_file))
    os.makedirs(out_dir, exist_ok=True)

    waypoints = load_mission(args.mission_file)
    if not waypoints:
        print("No valid waypoints found -- nothing to plot.")
        sys.exit(1)

    print_summary(waypoints, args.mission_file)
    print("\nGenerating map...")
    plot_mission(waypoints, out_dir, arrow_every=args.arrow_every, arrow_len_m=args.arrow_len)
    print(f"\nDone. Output file in: {out_dir}")
    print(f"  mission_map.html - interactive map of the planned mission")


if __name__ == '__main__':
    main()
