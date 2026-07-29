#!/usr/bin/env python3
"""
analyze_speed.py
================
Calculate and plot speed from lat/lon GPS data using the haversine formula.
Use when speed_mps column is empty but lat/lon data is present.

Produces:
  speed_calculated.png  - speed (m/s) vs elapsed time with fix quality strip
                          and a smoothed rolling average overlay

Usage:
  python3 analyze_speed.py field_test_20260629_165953.csv
  python3 analyze_speed.py field_test_20260629_165953.csv --smooth 5
  python3 analyze_speed.py field_test_20260629_165953.csv --min_elapsed 60
"""

import argparse
import math
import os
import sys

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ---------------------------------------------------------------------------
# Fix quality color mapping (matches analyze_field_test.py)
# ---------------------------------------------------------------------------
FIX_COLORS = {
    'RTK Fixed':  '#00cc44',
    'RTK Float':  '#ffaa00',
    'DGPS':       '#3399ff',
    'GPS':        '#ff6600',
    'No Fix':     '#cc0000',
}

def fix_color(fix_str):
    for k, v in FIX_COLORS.items():
        if k.lower() in str(fix_str).lower():
            return v
    return '#888888'

# ---------------------------------------------------------------------------
# Haversine distance between two lat/lon points -> meters
# ---------------------------------------------------------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.asin(math.sqrt(a))

# ---------------------------------------------------------------------------
# Load CSV and calculate speed
# ---------------------------------------------------------------------------
def load_and_calc(path, min_elapsed=0.0):
    df = pd.read_csv(path, parse_dates=['time'])

    # Drop rows with no GPS position
    df = df.dropna(subset=['lat', 'lon'])
    df = df[(df['lat'] != 0) & (df['lon'] != 0)].reset_index(drop=True)
    print(f"Loaded {len(df)} rows with valid GPS")

    # Optionally skip initial stationary period
    if min_elapsed > 0:
        df = df[df['elapsed_sec'] >= min_elapsed].reset_index(drop=True)
        print(f"Trimmed to {len(df)} rows (elapsed >= {min_elapsed}s)")

    if len(df) < 2:
        print("Not enough rows to calculate speed.")
        sys.exit(1)

    # Calculate distance and time delta between consecutive rows
    speeds = [0.0]  # first row has no previous point
    for i in range(1, len(df)):
        dist_m = haversine_m(
            df['lat'].iloc[i-1], df['lon'].iloc[i-1],
            df['lat'].iloc[i],   df['lon'].iloc[i]
        )
        dt = df['elapsed_sec'].iloc[i] - df['elapsed_sec'].iloc[i-1]
        if dt > 0:
            speeds.append(dist_m / dt)
        else:
            speeds.append(0.0)

    df['speed_calc_mps'] = speeds

    # Cap outliers: GPS noise can produce spikes on stationary data.
    # 10 m/s (~22 mph) is well above tractor max speed — anything above is noise.
    MAX_SPEED = 10.0
    outliers = (df['speed_calc_mps'] > MAX_SPEED).sum()
    if outliers:
        print(f"Capped {outliers} outlier speed values above {MAX_SPEED} m/s")
    df['speed_calc_mps'] = df['speed_calc_mps'].clip(upper=MAX_SPEED)

    return df

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def plot_speed(df, out_dir, smooth_window):
    elapsed = df['elapsed_sec']
    speed   = df['speed_calc_mps']

    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle('Calculated Speed over Time (from lat/lon haversine)',
                 fontsize=13, fontweight='bold')

    # Raw speed — thin and semi-transparent
    ax.plot(elapsed, speed, color='#1f77b4', lw=0.7, alpha=0.5, label='Raw speed')

    # Rolling average overlay
    if smooth_window > 1:
        smoothed = speed.rolling(window=smooth_window, center=True, min_periods=1).mean()
        ax.plot(elapsed, smoothed, color='#d62728', lw=1.5,
                label=f'Rolling avg ({smooth_window} samples)')

    ax.fill_between(elapsed, speed, alpha=0.10, color='#1f77b4')
    ax.set_xlabel('Elapsed time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)

    # Stats annotation box
    moving = speed[speed > 0.05]
    stats_text = (
        f"All rows:  max={speed.max():.2f}  mean={speed.mean():.2f} m/s\n"
        f"Moving (>0.05 m/s):  n={len(moving)}  mean={moving.mean():.2f}  max={moving.max():.2f} m/s"
    )
    ax.text(0.01, 0.97, stats_text, transform=ax.transAxes,
            fontsize=8, va='top', ha='left',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Fix quality color strip along the bottom
    ymin, ymax = ax.get_ylim()
    strip_h = (ymax - ymin) * 0.04
    for i in range(len(df) - 1):
        ax.barh(ymin,
                elapsed.iloc[i+1] - elapsed.iloc[i],
                left=elapsed.iloc[i],
                height=strip_h,
                color=fix_color(df['fix_quality'].iloc[i]),
                alpha=0.85)

    # Legend: speed lines + fix quality strip
    speed_handles = [
        mpatches.Patch(color='#1f77b4', alpha=0.5, label='Raw speed'),
    ]
    if smooth_window > 1:
        speed_handles.append(
            mpatches.Patch(color='#d62728', label=f'Rolling avg ({smooth_window} samples)')
        )
    fix_handles = [
        mpatches.Patch(color=c, label=n) for n, c in FIX_COLORS.items()
    ]
    ax.legend(handles=speed_handles + fix_handles,
              title='— Speed  |  GPS Fix (strip) —',
              loc='upper right', fontsize=7, title_fontsize=7)

    plt.tight_layout()
    out_path = os.path.join(out_dir, 'speed_calculated.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {out_path}")

    # Print quick summary to console
    print(f"\nSpeed summary (haversine from lat/lon):")
    print(f"  Rows calculated: {len(df)}")
    print(f"  Max speed:  {speed.max():.3f} m/s  ({speed.max()*3.6:.2f} km/h)")
    print(f"  Mean speed: {speed.mean():.3f} m/s  ({speed.mean()*3.6:.2f} km/h)")
    moving = speed[speed > 0.05]
    if len(moving):
        print(f"  Mean while moving (>0.05 m/s): {moving.mean():.3f} m/s  ({moving.mean()*3.6:.2f} km/h)")
        print(f"  Time moving: {len(moving) * (elapsed.iloc[-1]-elapsed.iloc[0]) / len(df):.0f}s estimate")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Calculate speed from lat/lon GPS data')
    parser.add_argument('csv_file', help='Path to field_test_*.csv')
    parser.add_argument('--out', '-o', default=None,
                        help='Output directory (default: same dir as CSV)')
    parser.add_argument('--smooth', type=int, default=10,
                        help='Rolling average window in samples (default: 10, set 1 to disable)')
    parser.add_argument('--min_elapsed', type=float, default=0.0,
                        help='Skip rows before this elapsed time in seconds (e.g. 60 to skip startup)')
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print(f"Error: file not found: {args.csv_file}")
        sys.exit(1)

    out_dir = args.out or os.path.dirname(os.path.abspath(args.csv_file))
    os.makedirs(out_dir, exist_ok=True)

    df = load_and_calc(args.csv_file, min_elapsed=args.min_elapsed)
    plot_speed(df, out_dir, smooth_window=args.smooth)


if __name__ == '__main__':
    main()
