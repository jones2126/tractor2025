#!/usr/bin/env python3
"""
analyze_wifi_rssi.py
====================
Visualize WiFi signal strength from field_test_*.csv logs.

Produces two HTML maps and one PNG:

  wifi_comparison_map.html  - dots colored by WHICH signal is stronger at
                              each GPS point (RPi wlan0 vs GL router)
  wifi_rssi_map.html        - side-by-side view: left layer = RPi dBm,
                              right layer = router dBm (toggle via layer control)
  wifi_rssi_plot.png        - time-series of both RSSI values with the
                              stronger signal highlighted

Usage:
  python3 analyze_wifi_rssi.py field_test_20260710_115255.csv
  python3 analyze_wifi_rssi.py field_test_20260710_115255.csv --out /tmp/results
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
import folium

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------
COLOR_RPI    = '#1f77b4'   # blue  — RPi wlan0 is stronger
COLOR_ROUTER = '#d62728'   # red   — GL router is stronger
COLOR_EQUAL  = '#888888'   # grey  — within margin / only one present

# RSSI strength buckets (for individual signal maps)
def rssi_color(val):
    try:
        v = float(val)
        if v > -60:  return '#00cc44'   # green  strong
        if v > -70:  return '#aacc00'   # yellow-green
        if v > -75:  return '#ffaa00'   # amber
        if v > -80:  return '#ff6600'   # orange
        return              '#cc0000'   # red    weak
    except (TypeError, ValueError):
        return '#888888'

# ---------------------------------------------------------------------------
# Load
# ---------------------------------------------------------------------------
def load(path):
    df = pd.read_csv(path, parse_dates=['time'])
    df = df.dropna(subset=['lat', 'lon'])
    df = df[(df['lat'] != 0) & (df['lon'] != 0)].reset_index(drop=True)
    df['wifi_rssi_dbm']        = pd.to_numeric(df['wifi_rssi_dbm'],        errors='coerce')
    df['router_wifi_rssi_dbm'] = pd.to_numeric(df['router_wifi_rssi_dbm'], errors='coerce')
    print(f"Loaded {len(df)} rows with valid GPS")
    print(f"  RPi wlan0   : {df['wifi_rssi_dbm'].notna().sum()} rows  "
          f"range {df['wifi_rssi_dbm'].min():.0f} to {df['wifi_rssi_dbm'].max():.0f} dBm")
    print(f"  GL Router   : {df['router_wifi_rssi_dbm'].notna().sum()} rows  "
          f"range {df['router_wifi_rssi_dbm'].min():.0f} to {df['router_wifi_rssi_dbm'].max():.0f} dBm")
    return df

# ---------------------------------------------------------------------------
# Helper: base folium map
# ---------------------------------------------------------------------------
def base_map(df):
    m = folium.Map(
        location=[df['lat'].mean(), df['lon'].mean()],
        zoom_start=21,
        tiles='CartoDB positron',
        max_zoom=22,
    )
    folium.PolyLine(list(zip(df['lat'], df['lon'])),
                    color='#333333', weight=1, opacity=0.25).add_to(m)
    folium.Marker([df['lat'].iloc[0],  df['lon'].iloc[0]],
                  popup='Start', icon=folium.Icon(color='green', icon='play')).add_to(m)
    folium.Marker([df['lat'].iloc[-1], df['lon'].iloc[-1]],
                  popup='End',   icon=folium.Icon(color='red',   icon='stop')).add_to(m)
    return m

def add_legend(m, html):
    m.get_root().html.add_child(folium.Element(html))

def legend_div(title, items):
    """items = list of (color, label)"""
    html = (
        '<div style="position:fixed;bottom:30px;left:30px;z-index:1000;'
        'background:rgba(0,0,0,0.78);padding:12px;border-radius:8px;'
        'color:white;font-size:12px;line-height:1.6;">'
        f'<b>{title}</b><br>'
    )
    for color, label in items:
        html += (
            f'<span style="display:inline-block;width:13px;height:13px;'
            f'background:{color};border-radius:50%;margin-right:5px;'
            f'vertical-align:middle;"></span>{label}<br>'
        )
    html += '</div>'
    return html

# ---------------------------------------------------------------------------
# Map 1: comparison — which signal is stronger at each point
# ---------------------------------------------------------------------------
def map_comparison(df, out_dir):
    MARGIN_DBM = 3   # treat signals within 3 dBm as equal

    m = base_map(df)

    fg_rpi    = folium.FeatureGroup(name='RPi wlan0 stronger')
    fg_router = folium.FeatureGroup(name='GL Router stronger')
    fg_equal  = folium.FeatureGroup(name='Within 3 dBm / single signal')
    m.add_child(fg_rpi)
    m.add_child(fg_router)
    m.add_child(fg_equal)

    for _, row in df.iterrows():
        rpi    = row['wifi_rssi_dbm']
        router = row['router_wifi_rssi_dbm']
        rpi_ssid    = row.get('wifi_ssid', '')
        router_ssid = row.get('router_wifi_ssid', '')
        fix    = row.get('fix_quality', '')

        rpi_ok    = pd.notna(rpi)
        router_ok = pd.notna(router)

        if rpi_ok and router_ok:
            diff = rpi - router   # positive = RPi stronger (less negative)
            if diff > MARGIN_DBM:
                color = COLOR_RPI
                winner = f'RPi wlan0 stronger by {diff:.0f} dBm'
                fg = fg_rpi
            elif diff < -MARGIN_DBM:
                color = COLOR_ROUTER
                winner = f'GL Router stronger by {-diff:.0f} dBm'
                fg = fg_router
            else:
                color = COLOR_EQUAL
                winner = f'Within {MARGIN_DBM} dBm (approx equal)'
                fg = fg_equal
        elif rpi_ok:
            color, winner, fg = COLOR_RPI, 'RPi only (no router data)', fg_rpi
        elif router_ok:
            color, winner, fg = COLOR_ROUTER, 'Router only (no RPi data)', fg_router
        else:
            continue

        tooltip = (
            f"<b>{winner}</b><br>"
            f"RPi wlan0: {rpi_ssid} {rpi:.0f} dBm<br>" if rpi_ok else
            f"<b>{winner}</b><br>RPi wlan0: no data<br>"
        )
        tooltip += (
            f"GL Router: {router_ssid} {router:.0f} dBm<br>" if router_ok
            else "GL Router: no data<br>"
        )
        tooltip += f"GPS fix: {fix}<br>t={row['elapsed_sec']:.1f}s"

        folium.CircleMarker(
            location=[row['lat'], row['lon']],
            radius=4, color=color, fill=True,
            fill_color=color, fill_opacity=0.9,
            tooltip=tooltip,
        ).add_to(fg)

    add_legend(m, legend_div(
        'Which WiFi signal is stronger?',
        [
            (COLOR_RPI,    'RPi wlan0 stronger (> 3 dBm)'),
            (COLOR_ROUTER, 'GL Router stronger (> 3 dBm)'),
            (COLOR_EQUAL,  'Within 3 dBm / single signal'),
        ]
    ))
    folium.LayerControl().add_to(m)

    out = os.path.join(out_dir, 'wifi_comparison_map.html')
    m.save(out)
    print(f"  Saved: {out}")
    return out

# ---------------------------------------------------------------------------
# Map 2: individual RSSI levels — one layer per source, toggleable
# ---------------------------------------------------------------------------
def map_rssi_levels(df, out_dir):
    m = base_map(df)

    fg_rpi    = folium.FeatureGroup(name='RPi wlan0 RSSI level')
    fg_router = folium.FeatureGroup(name='GL Router RSSI level')
    m.add_child(fg_rpi)
    m.add_child(fg_router)

    for _, row in df.iterrows():
        rpi    = row['wifi_rssi_dbm']
        router = row['router_wifi_rssi_dbm']
        fix    = row.get('fix_quality', '')
        t      = row['elapsed_sec']

        if pd.notna(rpi):
            tooltip = (
                f"<b>RPi wlan0: {rpi:.0f} dBm</b><br>"
                f"SSID: {row.get('wifi_ssid','')}<br>"
                f"GPS fix: {fix}<br>t={t:.1f}s"
            )
            folium.CircleMarker(
                location=[row['lat'], row['lon']],
                radius=4, color=rssi_color(rpi), fill=True,
                fill_color=rssi_color(rpi), fill_opacity=0.9,
                tooltip=tooltip,
            ).add_to(fg_rpi)

        if pd.notna(router):
            tooltip = (
                f"<b>GL Router: {router:.0f} dBm</b><br>"
                f"SSID: {row.get('router_wifi_ssid','')}<br>"
                f"GPS fix: {fix}<br>t={t:.1f}s"
            )
            folium.CircleMarker(
                location=[row['lat'], row['lon']],
                radius=4, color=rssi_color(router), fill=True,
                fill_color=rssi_color(router), fill_opacity=0.9,
                tooltip=tooltip,
            ).add_to(fg_router)

    add_legend(m, legend_div(
        'WiFi RSSI Level (toggle layers above)',
        [
            ('#00cc44', '> -60 dBm  strong'),
            ('#aacc00', '-60 to -70 dBm  good'),
            ('#ffaa00', '-70 to -75 dBm  marginal'),
            ('#ff6600', '-75 to -80 dBm  weak'),
            ('#cc0000', '< -80 dBm  very weak'),
        ]
    ))
    folium.LayerControl().add_to(m)

    out = os.path.join(out_dir, 'wifi_rssi_map.html')
    m.save(out)
    print(f"  Saved: {out}")
    return out

# ---------------------------------------------------------------------------
# Plot: time-series of both RSSI values
# ---------------------------------------------------------------------------
def plot_rssi_time(df, out_dir):
    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle('WiFi RSSI over Time', fontsize=13, fontweight='bold')

    elapsed = df['elapsed_sec']
    rpi    = df['wifi_rssi_dbm']
    router = df['router_wifi_rssi_dbm']

    ax.plot(elapsed, rpi,    color=COLOR_RPI,    lw=0.8, alpha=0.8, label='RPi wlan0')
    ax.plot(elapsed, router, color=COLOR_ROUTER,  lw=0.8, alpha=0.8, label='GL Router')

    # Shade the region where each is stronger
    both = df['wifi_rssi_dbm'].notna() & df['router_wifi_rssi_dbm'].notna()
    rpi_vals    = rpi.where(both)
    router_vals = router.where(both)
    ax.fill_between(elapsed, rpi_vals, router_vals,
                    where=(rpi_vals > router_vals),
                    alpha=0.15, color=COLOR_RPI,
                    label='RPi stronger zone')
    ax.fill_between(elapsed, rpi_vals, router_vals,
                    where=(router_vals >= rpi_vals),
                    alpha=0.15, color=COLOR_ROUTER,
                    label='Router stronger zone')

    # Threshold lines
    for dbm, label in [(-60, '-60 strong'), (-75, '-75 marginal'), (-80, '-80 weak')]:
        ax.axhline(dbm, color='#aaaaaa', lw=0.7, linestyle='--')
        ax.text(elapsed.iloc[-1], dbm + 0.5, label, fontsize=7,
                color='#666666', ha='right', va='bottom')

    ax.set_xlabel('Elapsed time (s)')
    ax.set_ylabel('RSSI (dBm)')
    ax.set_ylim(-95, -25)
    ax.invert_yaxis()   # more negative = worse = bottom of chart
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)

    # Fix quality color strip at top
    ymin, ymax = ax.get_ylim()
    strip_h = abs(ymax - ymin) * 0.03
    FIX_COLORS = {
        'RTK Fixed': '#00cc44', 'RTK Float': '#ffaa00',
        'DGPS': '#3399ff', 'GPS': '#ff6600',
        'Invalid': '#cc0000',
    }
    def fix_color(s):
        for k, v in FIX_COLORS.items():
            if k.lower() in str(s).lower(): return v
        return '#888888'

    for i in range(len(df) - 1):
        ax.barh(ymin, elapsed.iloc[i+1] - elapsed.iloc[i],
                left=elapsed.iloc[i], height=strip_h,
                color=fix_color(df['fix_quality'].iloc[i]), alpha=0.85)

    fix_handles = [mpatches.Patch(color=c, label=n) for n, c in FIX_COLORS.items()]
    ax.legend(
        handles=ax.get_legend_handles_labels()[0] + fix_handles,
        labels=ax.get_legend_handles_labels()[1] + list(FIX_COLORS.keys()),
        loc='lower right', fontsize=7, ncol=2
    )

    plt.tight_layout()
    out = os.path.join(out_dir, 'wifi_rssi_plot.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {out}")
    return out

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Analyze WiFi RSSI from field test CSV')
    parser.add_argument('csv_file')
    parser.add_argument('--out', '-o', default=None)
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print(f"Error: {args.csv_file} not found"); sys.exit(1)

    out_dir = args.out or os.path.dirname(os.path.abspath(args.csv_file))
    os.makedirs(out_dir, exist_ok=True)

    df = load(args.csv_file)

    print("\nGenerating outputs...")
    map_comparison(df, out_dir)
    map_rssi_levels(df, out_dir)
    plot_rssi_time(df, out_dir)

    print(f"\nDone. Output files in: {out_dir}")
    print(f"  wifi_comparison_map.html - which signal is stronger at each GPS point")
    print(f"  wifi_rssi_map.html       - individual dBm levels, toggle RPi vs router layer")
    print(f"  wifi_rssi_plot.png       - time-series of both signals with GPS fix quality strip")

if __name__ == '__main__':
    main()
