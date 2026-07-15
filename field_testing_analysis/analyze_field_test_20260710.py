#!/usr/bin/env python3
"""
analyze_field_test.py
=====================
Analyze and visualize field_test_*.csv logs from field_test_logger_20260710.py

Produces:
  1. gps_track.png          - matplotlib GPS track colored by fix quality
  2. speed_over_time.png    - speed (m/s) vs elapsed time            NEW
  3. gps_track_map.html     - interactive map colored by GPS fix quality
  4. rpi_wifi_map.html      - interactive map colored by RPi wlan0 RSSI (RPi→Mofi)  NEW
  5. router_wifi_map.html   - interactive map colored by GL router RSSI (router→hotspot)  RENAMED
  6. summary.txt            - fix quality breakdown, speed, heading, WiFi stats

Usage:
  python3 analyze_field_test.py field_test_20260710_143022.csv
  python3 analyze_field_test.py field_test_20260710_143022.csv --out /tmp/results
"""

import argparse
import os
import sys
import warnings
warnings.filterwarnings('ignore')

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import to_hex
import folium
from folium.plugins import AntPath


# ---------------------------------------------------------------------------
# Fix quality color mapping
# ---------------------------------------------------------------------------
FIX_COLORS = {
    'RTK Fixed':  '#00cc44',   # green
    'RTK Float':  '#ffaa00',   # amber
    'DGPS':       '#3399ff',   # blue
    'GPS':        '#ff6600',   # orange
    'No Fix':     '#cc0000',   # red
}
DEFAULT_COLOR = '#888888'

def fix_color(fix_str):
    for k, v in FIX_COLORS.items():
        if k.lower() in str(fix_str).lower():
            return v
    return DEFAULT_COLOR


# ---------------------------------------------------------------------------
# WiFi RSSI color mapping (router upstream signal strength)
# Thresholds: > -60 dBm = strong, -60 to -75 = medium, < -75 = weak
# ---------------------------------------------------------------------------
WIFI_COLORS = {
    'strong':  '#00cc44',  # green
    'medium':  '#ffaa00',  # amber
    'weak':    '#cc0000',  # red
    'unknown': '#888888',  # grey
}

def wifi_rssi_color(rssi_val, label_val):
    """Return a color based on router_wifi_rssi_dbm; fall back to signal label."""
    # NEW: prefer numeric RSSI for precise bucketing
    try:
        rssi = float(rssi_val)
        if rssi > -60:
            return WIFI_COLORS['strong']
        elif rssi > -75:
            return WIFI_COLORS['medium']
        else:
            return WIFI_COLORS['weak']
    except (TypeError, ValueError):
        pass
    # Fall back to label string if RSSI column is empty
    label = str(label_val).lower()
    return WIFI_COLORS.get(label, WIFI_COLORS['unknown'])


# ---------------------------------------------------------------------------
# Load and clean
# ---------------------------------------------------------------------------
def load_csv(path):
    df = pd.read_csv(path, parse_dates=['time'])
    # Drop rows with no GPS fix at all
    df = df.dropna(subset=['lat', 'lon'])
    df = df[(df['lat'] != 0) & (df['lon'] != 0)]
    df = df.reset_index(drop=True)
    print(f"Loaded {len(df)} rows with valid GPS from {os.path.basename(path)}")
    return df


# ---------------------------------------------------------------------------
# 1. Static matplotlib plot
# ---------------------------------------------------------------------------
def plot_static(df, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Field Test GPS Track', fontsize=13, fontweight='bold')

    # --- Left: GPS track colored by fix quality ---
    ax = axes[0]
    for fix_val in df['fix_quality'].unique():
        mask = df['fix_quality'] == fix_val
        color = fix_color(fix_val)
        ax.scatter(df.loc[mask, 'lon'], df.loc[mask, 'lat'],
                   c=color, s=8, label=str(fix_val), zorder=3, alpha=0.85)

    # Draw a thin connecting line to show path
    ax.plot(df['lon'], df['lat'], 'k-', linewidth=0.4, alpha=0.3, zorder=2)

    # Mark start and end
    ax.plot(df['lon'].iloc[0],  df['lat'].iloc[0],  'k^', ms=8, zorder=5, label='Start')
    ax.plot(df['lon'].iloc[-1], df['lat'].iloc[-1], 'ks', ms=8, zorder=5, label='End')

    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Track — color = fix quality')
    ax.legend(loc='best', fontsize=7, markerscale=1.5)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)

    # --- Right: heading and speed over time ---
    ax2 = axes[1]
    elapsed = df['elapsed_sec']

    color_hdg = '#1f77b4'
    color_spd = '#d62728'

    ax2.plot(elapsed, df['heading_deg'], color=color_hdg, lw=0.8, label='Heading (°)')
    ax2.set_xlabel('Elapsed time (s)')
    ax2.set_ylabel('Heading (°)', color=color_hdg)
    ax2.tick_params(axis='y', labelcolor=color_hdg)

    if df['speed_mps'].notna().any() and (df['speed_mps'] != '').any():
        try:
            speeds = pd.to_numeric(df['speed_mps'], errors='coerce')
            ax2b = ax2.twinx()
            ax2b.plot(elapsed, speeds, color=color_spd, lw=0.8, alpha=0.7, label='Speed (m/s)')
            ax2b.set_ylabel('Speed (m/s)', color=color_spd)
            ax2b.tick_params(axis='y', labelcolor=color_spd)
        except Exception:
            pass

    ax2.set_title('Heading & Speed vs. Time')
    ax2.grid(True, alpha=0.3)

    # Fix quality strip at bottom of time plot
    ymin, ymax = ax2.get_ylim()
    strip_h = (ymax - ymin) * 0.04
    for i in range(len(df) - 1):
        ax2.barh(ymin, elapsed.iloc[i+1] - elapsed.iloc[i],
                 left=elapsed.iloc[i], height=strip_h,
                 color=fix_color(df['fix_quality'].iloc[i]), alpha=0.7)

    plt.tight_layout()
    out_path = os.path.join(out_dir, 'gps_track.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# 2. Speed over time plot  NEW
# ---------------------------------------------------------------------------
def plot_speed(df, out_dir):
    """Dedicated speed (m/s) vs elapsed time plot with fix quality color strip."""
    speeds = pd.to_numeric(df['speed_mps'], errors='coerce')
    has_speed = speeds.notna().any()

    fig, ax = plt.subplots(figsize=(12, 4))
    fig.suptitle('Speed over Time', fontsize=13, fontweight='bold')
    elapsed = df['elapsed_sec']

    if has_speed:
        # NEW: plot speed line with shaded fill
        ax.plot(elapsed, speeds, color='#1f77b4', lw=1.2, label='Speed (m/s)')
        ax.fill_between(elapsed, speeds, alpha=0.15, color='#1f77b4')
        ax.set_ylabel('Speed (m/s)')
        ax.set_ylim(bottom=0)
    else:
        # NEW: graceful message if speed column is empty
        ax.text(0.5, 0.5, 'No speed data in this file\n(speed_mps column is empty)',
                ha='center', va='center', transform=ax.transAxes,
                fontsize=11, color='#888888')

    ax.set_xlabel('Elapsed time (s)')
    ax.grid(True, alpha=0.3)

    # NEW: fix quality color strip along the bottom
    ymin, ymax = ax.get_ylim()
    strip_h = (ymax - ymin) * 0.05
    for i in range(len(df) - 1):
        ax.barh(ymin, elapsed.iloc[i+1] - elapsed.iloc[i],
                left=elapsed.iloc[i], height=strip_h,
                color=fix_color(df['fix_quality'].iloc[i]), alpha=0.8)

    # NEW: fix quality legend for the strip
    handles = [mpatches.Patch(color=c, label=n) for n, c in FIX_COLORS.items()]
    ax.legend(handles=handles, title='GPS Fix (strip)', loc='upper right',
              fontsize=7, title_fontsize=7)

    plt.tight_layout()
    out_path = os.path.join(out_dir, 'speed_over_time.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {out_path}')
    return out_path


# ---------------------------------------------------------------------------
# 3. Interactive Folium map
# ---------------------------------------------------------------------------
def plot_folium(df, out_dir):
    center_lat = df['lat'].mean()
    center_lon = df['lon'].mean()

    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=21,
        tiles='CartoDB positron',
        max_zoom=22,
    )

    # Thin grey path
    coords = list(zip(df['lat'], df['lon']))
    folium.PolyLine(coords, color='white', weight=1, opacity=0.4).add_to(m)

    # Colored dots per fix type
    fix_groups = {}
    for fix_val in df['fix_quality'].unique():
        fg = folium.FeatureGroup(name=str(fix_val))
        fix_groups[fix_val] = fg
        m.add_child(fg)

    for _, row in df.iterrows():
        color = fix_color(row['fix_quality'])
        # CHANGED: tooltip now includes both WiFi signal paths
        rpi_rssi    = row.get('wifi_rssi_dbm', '')
        router_rssi = row.get('router_wifi_rssi_dbm', '')
        rpi_ssid    = row.get('wifi_ssid', '')
        router_ssid = row.get('router_wifi_ssid', '')
        tooltip = (
            f"<b>{row['fix_quality']}</b><br>"
            f"t={row['elapsed_sec']:.1f}s<br>"
            f"Hdg: {row['heading_deg']:.1f}°<br>"
            f"Lat: {row['lat']:.8f}<br>"
            f"Lon: {row['lon']:.8f}<br>"
            f"<hr style='margin:3px 0'>"
            f"RPi WiFi: {rpi_ssid} {rpi_rssi} dBm<br>"
            f"Router WiFi: {router_ssid} {router_rssi} dBm"
        )
        folium.CircleMarker(
            location=[row['lat'], row['lon']],
            radius=4,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.9,
            tooltip=tooltip,
        ).add_to(fix_groups[row['fix_quality']])

    # Start / end markers
    folium.Marker(
        [df['lat'].iloc[0], df['lon'].iloc[0]],
        popup='Start',
        icon=folium.Icon(color='green', icon='play')
    ).add_to(m)
    folium.Marker(
        [df['lat'].iloc[-1], df['lon'].iloc[-1]],
        popup='End',
        icon=folium.Icon(color='red', icon='stop')
    ).add_to(m)

    # Legend HTML
    legend_html = '<div style="position:fixed;bottom:30px;left:30px;z-index:1000;background:rgba(0,0,0,0.7);padding:10px;border-radius:8px;color:white;font-size:12px;">'
    legend_html += '<b>Fix Quality</b><br>'
    for name, color in FIX_COLORS.items():
        legend_html += f'<span style="display:inline-block;width:12px;height:12px;background:{color};border-radius:50%;margin-right:4px;vertical-align:middle;"></span>{name}<br>'
    legend_html += '</div>'
    m.get_root().html.add_child(folium.Element(legend_html))

    folium.LayerControl().add_to(m)

    out_path = os.path.join(out_dir, 'gps_track_map.html')
    m.save(out_path)
    print(f"  Saved: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# 4. WiFi signal coverage maps  CHANGED: one function, two output files
# ---------------------------------------------------------------------------
def plot_wifi_map(df, out_dir, source='router'):
    # CHANGED: accepts source='router' or source='rpi' to select signal path.
    # Produces router_wifi_map.html or rpi_wifi_map.html respectively.
    # Call twice from main() to produce both files.
    if source == 'router':
        col_rssi       = 'router_wifi_rssi_dbm'
        col_label      = 'router_wifi_signal_label'
        col_ssid       = 'router_wifi_ssid'
        title          = 'GL Router -> Field Hotspot'
        out_name       = 'router_wifi_map.html'
        other_rssi_col = 'wifi_rssi_dbm'
        other_ssid_col = 'wifi_ssid'
        other_label    = 'RPi wlan0'
    else:
        col_rssi       = 'wifi_rssi_dbm'
        col_label      = 'wifi_signal_label'
        col_ssid       = 'wifi_ssid'
        title          = 'RPi wlan0 -> Mofi'
        out_name       = 'rpi_wifi_map.html'
        other_rssi_col = 'router_wifi_rssi_dbm'
        other_ssid_col = 'router_wifi_ssid'
        other_label    = 'GL Router'

    has_data = (
        col_rssi in df.columns and
        df[col_rssi].notna().any() and
        (df[col_rssi] != '').any()
    )

    center_lat = df['lat'].mean()
    center_lon = df['lon'].mean()

    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=21,
        tiles='CartoDB positron',
        max_zoom=22,
    )

    coords = list(zip(df['lat'], df['lon']))
    folium.PolyLine(coords, color='#333333', weight=1, opacity=0.3).add_to(m)

    # CHANGED: one FeatureGroup per signal bucket for layer toggle
    groups = {
        'strong':  folium.FeatureGroup(name='Strong (> -60 dBm)'),
        'medium':  folium.FeatureGroup(name='Medium (-60 to -75 dBm)'),
        'weak':    folium.FeatureGroup(name='Weak (< -75 dBm)'),
        'unknown': folium.FeatureGroup(name='No data'),
    }
    for g in groups.values():
        m.add_child(g)

    for _, row in df.iterrows():
        rssi_val  = row.get(col_rssi, '')
        label_val = row.get(col_label, '')
        ssid_val  = row.get(col_ssid, '')
        color = wifi_rssi_color(rssi_val, label_val)

        try:
            rssi = float(rssi_val)
            bucket = 'strong' if rssi > -60 else ('medium' if rssi > -75 else 'weak')
        except (TypeError, ValueError):
            bucket = 'unknown'

        other_rssi = row.get(other_rssi_col, '')
        other_ssid = row.get(other_ssid_col, '')
        fix = row.get('fix_quality', '')

        # CHANGED: tooltip header names the source clearly
        tooltip = (
            f"<b>{title}: {rssi_val} dBm</b><br>"
            f"SSID: {ssid_val}<br>"
            f"t={row['elapsed_sec']:.1f}s<br>"
            f"GPS fix: {fix}<br>"
            f"<hr style='margin:3px 0'>"
            f"{other_label}: {other_ssid} {other_rssi} dBm"
        )
        folium.CircleMarker(
            location=[row['lat'], row['lon']],
            radius=4,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.9,
            tooltip=tooltip,
        ).add_to(groups[bucket])

    folium.Marker(
        [df['lat'].iloc[0], df['lon'].iloc[0]],
        popup='Start',
        icon=folium.Icon(color='green', icon='play')
    ).add_to(m)
    folium.Marker(
        [df['lat'].iloc[-1], df['lon'].iloc[-1]],
        popup='End',
        icon=folium.Icon(color='red', icon='stop')
    ).add_to(m)

    notice = ''
    if not has_data:
        notice = f'<br><i style="color:#ffaa00">No {title} data in this file</i>'

    # CHANGED: legend title names the source so each file is self-describing
    legend_html = (
        f'<div style="position:fixed;bottom:30px;left:30px;z-index:1000;'
        f'background:rgba(0,0,0,0.75);padding:10px;border-radius:8px;'
        f'color:white;font-size:12px;">'
        f'<b>{title}</b><br>'
    )
    for lbl, clr in WIFI_COLORS.items():
        legend_html += (
            f'<span style="display:inline-block;width:12px;height:12px;'
            f'background:{clr};border-radius:50%;margin-right:4px;'
            f'vertical-align:middle;"></span>{lbl}<br>'
        )
    legend_html += notice + '</div>'
    m.get_root().html.add_child(folium.Element(legend_html))

    folium.LayerControl().add_to(m)

    out_path = os.path.join(out_dir, out_name)
    m.save(out_path)
    print(f'  Saved: {out_path}')
    return out_path



# ---------------------------------------------------------------------------
# 5. Summary stats
# ---------------------------------------------------------------------------
def write_summary(df, input_path, out_dir):
    lines = []
    lines.append(f"Field Test Summary: {os.path.basename(input_path)}")
    lines.append("=" * 55)

    duration = df['elapsed_sec'].max()
    lines.append(f"Duration:       {duration:.1f} s  ({duration/60:.1f} min)")
    lines.append(f"Total rows:     {len(df)}")
    lines.append(f"Time range:     {df['time'].iloc[0]}  →  {df['time'].iloc[-1]}")

    lines.append("\n--- GPS Fix Quality ---")
    fix_counts = df['fix_quality'].value_counts()
    for fix, count in fix_counts.items():
        pct = 100 * count / len(df)
        lines.append(f"  {fix:<20} {count:5d} rows  ({pct:.1f}%)")

    lines.append("\n--- Heading (deg) ---")
    lines.append(f"  Min: {df['heading_deg'].min():.2f}")
    lines.append(f"  Max: {df['heading_deg'].max():.2f}")
    lines.append(f"  Mean: {df['heading_deg'].mean():.2f}")

    speeds = pd.to_numeric(df['speed_mps'], errors='coerce').dropna()
    if len(speeds) > 0:
        lines.append("\n--- Speed (m/s) ---")
        lines.append(f"  Min: {speeds.min():.3f}")
        lines.append(f"  Max: {speeds.max():.3f}")
        lines.append(f"  Mean: {speeds.mean():.3f}")

    if 'radio_signal' in df.columns:
        lines.append("\n--- Radio Signal (NRF24) ---")
        sig_counts = df['radio_signal'].value_counts()
        for sig, count in sig_counts.items():
            lines.append(f"  {str(sig):<12} {count:5d} rows")
        lines.append("  (Note: this is NRF24 radio, not WiFi)")

    # NEW 20260710: WiFi signal stats
    for col_rssi, col_label, label in [
        ('wifi_rssi_dbm',        'wifi_signal_label',        'RPi wlan0 → Mofi'),
        ('router_wifi_rssi_dbm', 'router_wifi_signal_label', 'GL Router → Hotspot'),
    ]:
        if col_rssi in df.columns:
            vals = pd.to_numeric(df[col_rssi], errors='coerce').dropna()
            if len(vals) > 0:
                lines.append(f"\n--- WiFi Signal: {label} ---")
                lines.append(f"  Rows with data: {len(vals)} / {len(df)}")
                lines.append(f"  Min:  {vals.min():.0f} dBm")
                lines.append(f"  Max:  {vals.max():.0f} dBm")
                lines.append(f"  Mean: {vals.mean():.0f} dBm")
                if col_label in df.columns:
                    label_counts = df[col_label].value_counts()
                    for lbl, cnt in label_counts.items():
                        pct = 100 * cnt / len(df)
                        lines.append(f"  {str(lbl):<10} {cnt:5d} rows  ({pct:.1f}%)")
            else:
                lines.append(f"\n--- WiFi Signal: {label} --- (no data in this file)")

    lines.append("\n--- GPS Bounding Box ---")
    lines.append(f"  Lat:  {df['lat'].min():.8f}  →  {df['lat'].max():.8f}")
    lines.append(f"  Lon:  {df['lon'].min():.8f}  →  {df['lon'].max():.8f}")

    out_path = os.path.join(out_dir, 'summary.txt')
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"  Saved: {out_path}")
    print()
    print('\n'.join(lines))
    return out_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Analyze field test CSV logs')
    parser.add_argument('csv_file', help='Path to field_test_*.csv')
    parser.add_argument('--out', '-o', default=None,
                        help='Output directory (default: same dir as CSV)')
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print(f"Error: file not found: {args.csv_file}")
        sys.exit(1)

    out_dir = args.out or os.path.dirname(os.path.abspath(args.csv_file))
    os.makedirs(out_dir, exist_ok=True)

    df = load_csv(args.csv_file)
    if len(df) == 0:
        print("No valid GPS rows found — nothing to plot.")
        sys.exit(1)

    print("\nGenerating outputs...")
    plot_static(df, out_dir)
    plot_speed(df, out_dir)                          # NEW: dedicated speed plot
    plot_folium(df, out_dir)
    plot_wifi_map(df, out_dir, source='rpi')         # CHANGED: RPi wlan0 -> Mofi
    plot_wifi_map(df, out_dir, source='router')      # CHANGED: GL router -> hotspot
    write_summary(df, args.csv_file, out_dir)

    print(f"\nDone. Output files in: {out_dir}")
    print(f"  gps_track.png        - GPS track colored by fix quality")
    print(f"  speed_over_time.png  - speed (m/s) vs elapsed time")
    print(f"  gps_track_map.html   - interactive map: GPS fix quality")
    print(f"  rpi_wifi_map.html    - interactive map: RPi wlan0 -> Mofi signal")
    print(f"  router_wifi_map.html - interactive map: GL router -> hotspot signal")
    print(f"  summary.txt          - stats including WiFi signal breakdown")


if __name__ == '__main__':
    main()
