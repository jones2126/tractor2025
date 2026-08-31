#!/usr/bin/env python3
"""Plot one manually selected perimeter segment by RTK/carrier state."""

import argparse
import csv
import html
import math
from pathlib import Path


EARTH_RADIUS_M = 6371008.8


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--start-elapsed", required=True, type=float)
    parser.add_argument("--end-elapsed", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--html-output", type=Path)
    parser.add_argument("--title", default="Manual perimeter run")
    args = parser.parse_args()

    with args.input.open(newline="", encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))

    valid = [
        row for row in rows
        if number(row.get("lat")) is not None and number(row.get("lon")) is not None
    ]
    if not valid:
        raise SystemExit("No valid latitude/longitude rows")

    lat0 = math.radians(number(valid[0]["lat"]))
    lon0 = math.radians(number(valid[0]["lon"]))

    def xy(row):
        lat = math.radians(number(row["lat"]))
        lon = math.radians(number(row["lon"]))
        return (
            EARTH_RADIUS_M * (lon - lon0) * math.cos((lat + lat0) / 2),
            EARTH_RADIUS_M * (lat - lat0),
        )

    # One point per distinct F9P position avoids plotting repeated 20 Hz rows.
    points = []
    previous = None
    for row in valid:
        position = (row["lat"], row["lon"])
        if position == previous:
            continue
        elapsed = number(row.get("elapsed_sec"))
        x, y = xy(row)
        points.append({"row": row, "elapsed": elapsed, "x": x, "y": y})
        previous = position

    selected = [
        point for point in points
        if point["elapsed"] is not None
        and args.start_elapsed <= point["elapsed"] <= args.end_elapsed
    ]
    if not selected:
        raise SystemExit("No position rows in selected elapsed-time interval")

    fixed = []
    floating = []
    other = []
    for point in selected:
        row = point["row"]
        if row.get("fix_quality") == "RTK Fixed" and row.get("carrier") == "fixed":
            fixed.append(point)
        elif row.get("fix_quality") == "RTK Fixed" and row.get("carrier") == "float":
            floating.append(point)
        else:
            other.append(point)

    width, height = 1200, 900
    left, right, top, bottom = 100, 45, 105, 80
    plot_width = width - left - right
    plot_height = height - top - bottom
    xs = [point["x"] for point in points]
    ys = [point["y"] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad = 3.0
    min_x, max_x = min_x - pad, max_x + pad
    min_y, max_y = min_y - pad, max_y + pad
    scale = min(plot_width / (max_x - min_x), plot_height / (max_y - min_y))
    used_width = (max_x - min_x) * scale
    used_height = (max_y - min_y) * scale
    x_offset = left + (plot_width - used_width) / 2
    y_offset = top + (plot_height - used_height) / 2

    def screen(point):
        return (
            x_offset + (point["x"] - min_x) * scale,
            y_offset + (max_y - point["y"]) * scale,
        )

    def polyline(group):
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in map(screen, group))

    def nice_step(span):
        raw = span / 8
        magnitude = 10 ** math.floor(math.log10(raw))
        fraction = raw / magnitude
        factor = 1 if fraction <= 1 else 2 if fraction <= 2 else 5 if fraction <= 5 else 10
        return factor * magnitude

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#222}.tick{font-size:13px}.legend{font-size:13px}</style>',
        f'<text x="{width/2}" y="31" text-anchor="middle" font-size="18">{html.escape(args.title)}</text>',
        f'<text x="{width/2}" y="55" text-anchor="middle" font-size="15">Highlighted points where RTK was Fixed and heading carrier was Float</text>',
    ]

    x_step = nice_step(max_x - min_x)
    x_tick = math.ceil(min_x / x_step) * x_step
    while x_tick <= max_x:
        sx = x_offset + (x_tick - min_x) * scale
        lines.append(f'<line x1="{sx:.1f}" y1="{y_offset:.1f}" x2="{sx:.1f}" y2="{y_offset+used_height:.1f}" stroke="#dddddd" stroke-width="1"/>')
        lines.append(f'<text class="tick" x="{sx:.1f}" y="{y_offset+used_height+22:.1f}" text-anchor="middle">{x_tick:g}</text>')
        x_tick += x_step

    y_step = nice_step(max_y - min_y)
    y_tick = math.ceil(min_y / y_step) * y_step
    while y_tick <= max_y:
        sy = y_offset + (max_y - y_tick) * scale
        lines.append(f'<line x1="{x_offset:.1f}" y1="{sy:.1f}" x2="{x_offset+used_width:.1f}" y2="{sy:.1f}" stroke="#dddddd" stroke-width="1"/>')
        lines.append(f'<text class="tick" x="{x_offset-12:.1f}" y="{sy+5:.1f}" text-anchor="end">{y_tick:g}</text>')
        y_tick += y_step

    lines.extend([
        f'<rect x="{x_offset:.1f}" y="{y_offset:.1f}" width="{used_width:.1f}" height="{used_height:.1f}" fill="none" stroke="#444" stroke-width="1.2"/>',
        f'<polyline points="{polyline(points)}" fill="none" stroke="#9ecae1" stroke-width="1.2" opacity="0.5"/>',
        f'<polyline points="{polyline(selected)}" fill="none" stroke="#ff7f0e" stroke-width="3"/>',
    ])

    def circles(group, color, radius):
        for point in group:
            sx, sy = screen(point)
            lines.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{radius}" fill="{color}"/>')

    circles(other, "#1f77b4", 2.0)
    circles(fixed, "#ff7f0e", 2.2)
    circles(floating, "#2ca02c", 2.4)

    start = selected[0]
    end = selected[-1]
    start_x, start_y = screen(start)
    end_x, end_y = screen(end)
    lines.extend([
        f'<path d="M {start_x-8:.1f} {start_y-8:.1f} L {start_x+8:.1f} {start_y+8:.1f} M {start_x+8:.1f} {start_y-8:.1f} L {start_x-8:.1f} {start_y+8:.1f}" stroke="#d62728" stroke-width="3" fill="none"/>',
        f'<polygon points="{end_x:.1f},{end_y-8:.1f} {end_x+8:.1f},{end_y:.1f} {end_x:.1f},{end_y+8:.1f} {end_x-8:.1f},{end_y:.1f}" fill="#9467bd"/>',
        f'<text x="{x_offset+used_width/2:.1f}" y="{height-24}" text-anchor="middle" font-size="15">Local X (m)</text>',
        f'<text x="25" y="{y_offset+used_height/2:.1f}" text-anchor="middle" font-size="15" transform="rotate(-90 25 {y_offset+used_height/2:.1f})">Local Y (m)</text>',
    ])

    legend_x, legend_y = x_offset + 14, y_offset + 14
    legend_items = [
        ("line", "#9ecae1", "Full logged path"),
        ("line", "#ff7f0e", "Analyzed perimeter segment"),
        ("circle", "#1f77b4", "Other states"),
        ("circle", "#ff7f0e", "RTK Fixed + heading carrier fixed"),
        ("circle", "#2ca02c", "RTK Fixed + heading carrier float"),
        ("x", "#d62728", f"Perimeter start ({args.start_elapsed:.1f} s)"),
        ("diamond", "#9467bd", f"Perimeter end / pause start ({args.end_elapsed:.1f} s)"),
    ]
    legend_height = 24 * len(legend_items) + 16
    lines.append(f'<rect x="{legend_x}" y="{legend_y}" width="330" height="{legend_height}" fill="white" fill-opacity="0.9" stroke="#cccccc"/>')
    for index, (kind, color, label) in enumerate(legend_items):
        y = legend_y + 21 + index * 24
        if kind == "line":
            lines.append(f'<line x1="{legend_x+12}" y1="{y}" x2="{legend_x+34}" y2="{y}" stroke="{color}" stroke-width="3"/>')
        elif kind == "circle":
            lines.append(f'<circle cx="{legend_x+23}" cy="{y}" r="4" fill="{color}"/>')
        elif kind == "x":
            lines.append(f'<path d="M {legend_x+17} {y-6} L {legend_x+29} {y+6} M {legend_x+29} {y-6} L {legend_x+17} {y+6}" stroke="{color}" stroke-width="2"/>')
        else:
            lines.append(f'<polygon points="{legend_x+23},{y-6} {legend_x+29},{y} {legend_x+23},{y+6} {legend_x+17},{y}" fill="{color}"/>')
        lines.append(f'<text class="legend" x="{legend_x+43}" y="{y+5}">{html.escape(label)}</text>')

    lines.append('</svg>')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines), encoding="utf-8")
    print(args.output.resolve())

    if args.html_output:
        themed_svg = "\n".join(lines)
        replacements = {
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">':
                f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img" aria-label="Manual perimeter path colored by RTK and heading carrier state">',
            '<rect width="100%" height="100%" fill="white"/>':
                '<rect width="100%" height="100%" fill="var(--background)" fill-opacity="0"/>',
            '<style>text{font-family:Arial,sans-serif;fill:#222}.tick{font-size:13px}.legend{font-size:13px}</style>':
                '<style>text{font-family:inherit;fill:var(--foreground)}.tick,.legend{font-size:13px}</style>',
            '#dddddd': 'var(--border)',
            '#444': 'var(--foreground)',
            '#9ecae1': 'var(--viz-series-4)',
            '#ff7f0e': 'var(--viz-series-2)',
            '#1f77b4': 'var(--viz-series-1)',
            '#2ca02c': 'var(--viz-series-3)',
            '#d62728': 'var(--viz-series-5)',
            '#9467bd': 'var(--viz-series-6)',
            'fill="white" fill-opacity="0.9" stroke="#cccccc"':
                'fill="var(--popover)" fill-opacity="0.94" stroke="var(--border)"',
        }
        for old, new in replacements.items():
            themed_svg = themed_svg.replace(old, new)
        fragment = "\n".join([
            '<div id="heading-carrier-map-20260829" style="width:100%;color:var(--foreground)">',
            '  <style>',
            '    #heading-carrier-map-20260829 svg { display:block; width:100%; height:auto; }',
            '  </style>',
            themed_svg,
            '</div>',
        ])
        args.html_output.parent.mkdir(parents=True, exist_ok=True)
        args.html_output.write_text(fragment, encoding="utf-8")
        print(args.html_output.resolve())


if __name__ == "__main__":
    main()
