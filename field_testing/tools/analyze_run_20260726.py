from __future__ import annotations

import argparse
import json
import re
import sys
import webbrowser
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent
AUTO_MODE = 0
MODE_ALIGNMENT_TOLERANCE_S = 0.15
THRESHOLDS_M = (0.05, 0.10, 0.15, 0.25, 0.50, 1.00)
MAP_DATA_MARKER = "/*__MAP_DATA__*/"
RUN_ID_PATTERN = re.compile(r"^\d{8}_\d{6}$")
SITE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


@dataclass(frozen=True)
class RunPaths:
    site_name: str
    run_id: str
    run_dir: Path
    pursuit_path: Path
    field_path: Path
    mission_path: Path
    audit_path: Path
    boundary_path: Path
    template_path: Path
    output_dir: Path
    analysis_path: Path
    map_data_path: Path
    map_path: Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze one Pure Pursuit field run and build a standalone "
            "interactive target-versus-actual HTML map."
        )
    )
    parser.add_argument(
        "--site-name",
        required=True,
        help="site archive/directory name, for example 62_Collins_Dr",
    )
    parser.add_argument(
        "--run-id",
        required=True,
        help="shared log timestamp in YYYYMMDD_HHMMSS form",
    )
    parser.add_argument(
        "--field-plans-root",
        type=Path,
        default=Path.home() / "Documents" / "field_plans",
        help="root containing <site>/runs/<run-id>",
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        help="override the derived <field-plans-root>/<site>/runs/<run-id>",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="output directory; defaults to the selected run directory",
    )
    parser.add_argument(
        "--template",
        type=Path,
        default=SCRIPT_DIR / "tractor-path-map-template.html",
        help="standalone HTML template containing the map-data marker",
    )
    parser.add_argument(
        "--open-map",
        action="store_true",
        help="open the generated HTML map in the default browser",
    )
    return parser


def resolve_paths(args: argparse.Namespace) -> RunPaths:
    site_name = args.site_name.strip()
    run_id = args.run_id.strip()
    if not SITE_NAME_PATTERN.fullmatch(site_name):
        raise ValueError(
            "site name may contain only letters, digits, underscore, dot, "
            "and hyphen"
        )
    if not RUN_ID_PATTERN.fullmatch(run_id):
        raise ValueError("run ID must use YYYYMMDD_HHMMSS")

    field_plans_root = args.field_plans_root.expanduser().resolve()
    run_dir = (
        args.run_dir.expanduser().resolve()
        if args.run_dir
        else field_plans_root / site_name / "runs" / run_id
    )
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else run_dir
    )
    mission_dir = run_dir / site_name
    output_stem = f"{site_name}_run"

    paths = RunPaths(
        site_name=site_name,
        run_id=run_id,
        run_dir=run_dir,
        pursuit_path=run_dir / f"pursuit_log_{run_id}.csv",
        field_path=run_dir / f"field_test_{run_id}.csv",
        mission_path=mission_dir / f"{site_name}_mission.txt",
        audit_path=mission_dir / f"{site_name}_mission_audit.csv",
        boundary_path=mission_dir / "01_boundary_final.csv",
        template_path=args.template.expanduser().resolve(),
        output_dir=output_dir,
        analysis_path=output_dir / f"{output_stem}_analysis_{run_id}.json",
        map_data_path=output_dir / f"{output_stem}_map_data_{run_id}.json",
        map_path=output_dir / f"{output_stem}_map_{run_id}.html",
    )
    required = (
        paths.pursuit_path,
        paths.field_path,
        paths.mission_path,
        paths.audit_path,
        paths.boundary_path,
        paths.template_path,
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Required analysis input(s) not found:\n  " + "\n  ".join(missing)
        )
    return paths


def require_columns(
    frame: pd.DataFrame, columns: tuple[str, ...], source_name: str
) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise ValueError(
            f"{source_name} is missing required column(s): "
            + ", ".join(missing)
        )


def weighted_percent_le(
    values: pd.Series, weights: pd.Series, threshold: float
) -> float:
    valid = values.notna() & weights.gt(0)
    denominator = float(weights[valid].sum())
    if denominator == 0:
        return float("nan")
    numerator = float(weights[valid & values.le(threshold)].sum())
    return 100.0 * numerator / denominator


def weighted_true_percent(values: pd.Series, weights: pd.Series) -> float:
    valid = values.notna() & weights.gt(0)
    denominator = float(weights[valid].sum())
    if denominator == 0:
        return float("nan")
    numerator = float(weights[valid & values.astype(bool)].sum())
    return 100.0 * numerator / denominator


def weighted_mean(values: pd.Series, weights: pd.Series) -> float:
    valid = values.notna() & weights.gt(0)
    if not valid.any():
        return float("nan")
    return float(np.average(values[valid], weights=weights[valid]))


def summarize_distance(
    group: pd.DataFrame, value_column: str
) -> dict[str, float | int]:
    weights = group["time_weight_s"]
    values = group[value_column]
    result: dict[str, float | int] = {
        "samples": int(values.notna().sum()),
        "duration_s": float(weights[values.notna()].sum()),
        "mean_m": weighted_mean(values, weights),
        "median_m": float(values.median()),
        "p90_m": float(values.quantile(0.90)),
        "p95_m": float(values.quantile(0.95)),
        "p99_m": float(values.quantile(0.99)),
        "max_m": float(values.max()),
        "rms_m": float(np.sqrt(weighted_mean(values.pow(2), weights))),
    }
    for threshold in THRESHOLDS_M:
        result[f"within_{threshold:.2f}m_pct_time"] = weighted_percent_le(
            values, weights, threshold
        )
    return result


def summarize_controller_error(
    group: pd.DataFrame,
) -> dict[str, float | int]:
    weights = group["time_weight_s"]
    cte = group["cte_m"]
    signed = group["signed_cte_m"]
    result: dict[str, float | int] = {
        "samples": int(len(group)),
        "duration_s": float(weights.sum()),
        "mean_abs_cte_m": weighted_mean(cte, weights),
        "median_abs_cte_m": float(cte.median()),
        "p90_abs_cte_m": float(cte.quantile(0.90)),
        "p95_abs_cte_m": float(cte.quantile(0.95)),
        "p99_abs_cte_m": float(cte.quantile(0.99)),
        "max_abs_cte_m": float(cte.max()),
        "rms_cte_m": float(np.sqrt(weighted_mean(cte.pow(2), weights))),
        "signed_bias_m": weighted_mean(signed, weights),
    }
    for threshold in THRESHOLDS_M:
        result[f"within_{threshold:.2f}m_pct_time"] = weighted_percent_le(
            cte, weights, threshold
        )
        result[f"within_{threshold:.2f}m_pct_samples"] = (
            100.0 * float(cte.le(threshold).mean())
        )
    return result


def calculate_geometric_cte(
    auto: pd.DataFrame, audit: pd.DataFrame
) -> list[float]:
    mission_xy = audit.sort_values("waypoint_idx")[
        ["east_m", "north_m"]
    ].to_numpy(dtype=float)
    if len(mission_xy) < 2:
        raise ValueError("mission audit must contain at least two waypoints")

    # The controller origin is mission waypoint 0. The audit coordinates use
    # the planning frame, so translate the path into the controller frame.
    mission_xy = mission_xy - mission_xy[0]
    geometric_cte: list[float] = []
    for row in auto.itertuples():
        point = np.array([float(row.pos_x_m), float(row.pos_y_m)])
        waypoint_index = int(row.waypoint_idx)
        low = max(0, waypoint_index - 40)
        high = min(len(mission_xy) - 1, waypoint_index + 10)
        if high <= low:
            raise ValueError(
                f"cannot form a mission segment near waypoint {waypoint_index}"
            )
        segment_start = mission_xy[low:high]
        segment_end = mission_xy[low + 1 : high + 1]
        segment = segment_end - segment_start
        denominator = np.einsum("ij,ij->i", segment, segment)
        fraction = np.clip(
            np.einsum("ij,ij->i", point - segment_start, segment)
            / np.where(denominator > 0, denominator, 1),
            0,
            1,
        )
        projection = segment_start + fraction[:, None] * segment
        geometric_cte.append(
            float(np.sqrt(((projection - point) ** 2).sum(axis=1)).min())
        )
    return geometric_cte


def analyze_run(paths: RunPaths) -> tuple[dict[str, object], dict[str, object]]:
    pursuit = pd.read_csv(paths.pursuit_path, low_memory=False)
    require_columns(
        pursuit,
        (
            "timestamp",
            "elapsed_s",
            "lat",
            "lon",
            "waypoint_idx",
            "speed_cmd_mps",
            "cross_track_err_m",
            "yt_m",
            "driving",
            "head_valid",
            "fix_quality",
            "pos_x_m",
            "pos_y_m",
        ),
        paths.pursuit_path.name,
    )
    pursuit["timestamp"] = pd.to_numeric(
        pursuit["timestamp"], errors="coerce"
    )
    pursuit = pursuit.dropna(subset=["timestamp"]).copy()
    for column in (
        "elapsed_s",
        "lat",
        "lon",
        "waypoint_idx",
        "speed_cmd_mps",
        "cross_track_err_m",
        "yt_m",
        "pos_x_m",
        "pos_y_m",
    ):
        pursuit[column] = pd.to_numeric(pursuit[column], errors="coerce")
    pursuit["driving_bool"] = (
        pursuit["driving"].astype(str).str.strip().str.lower().eq("true")
    )
    pursuit["head_valid_bool"] = (
        pursuit["head_valid"].astype(str).str.strip().str.lower().eq("true")
    )
    pursuit = pursuit.sort_values("timestamp").reset_index(drop=True)

    field = pd.read_csv(paths.field_path, low_memory=False)
    require_columns(
        field,
        ("time", "trans_mode", "speed_mps"),
        paths.field_path.name,
    )
    field["field_timestamp"] = pd.to_datetime(
        field["time"], utc=True, errors="coerce"
    ).map(lambda value: value.timestamp() if pd.notna(value) else np.nan)
    for column in ("trans_mode", "speed_mps"):
        field[column] = pd.to_numeric(field[column], errors="coerce")
    field = (
        field.dropna(subset=["field_timestamp", "trans_mode"])
        .sort_values("field_timestamp")
        .reset_index(drop=True)
    )

    aligned = pd.merge_asof(
        pursuit,
        field[["field_timestamp", "trans_mode", "speed_mps"]].rename(
            columns={"speed_mps": "actual_speed_mps"}
        ),
        left_on="timestamp",
        right_on="field_timestamp",
        direction="nearest",
        tolerance=MODE_ALIGNMENT_TOLERANCE_S,
    )

    audit = pd.read_csv(paths.audit_path)
    require_columns(
        audit,
        ("waypoint", "kind", "east_m", "north_m"),
        paths.audit_path.name,
    )
    for column in ("waypoint", "east_m", "north_m"):
        audit[column] = pd.to_numeric(audit[column], errors="coerce")
    audit = audit.dropna(subset=["waypoint", "east_m", "north_m"]).copy()
    audit["waypoint_idx"] = audit["waypoint"] - 1
    aligned = aligned.merge(
        audit[["waypoint_idx", "kind"]],
        how="left",
        on="waypoint_idx",
    )

    aligned["cte_m"] = aligned["cross_track_err_m"].abs()
    aligned["signed_cte_m"] = aligned["yt_m"]
    aligned["is_auto"] = aligned["trans_mode"].eq(AUTO_MODE)
    aligned["analysis_mask"] = (
        aligned["is_auto"]
        & aligned["driving_bool"]
        & aligned["cte_m"].notna()
        & aligned["lat"].notna()
        & aligned["lon"].notna()
        & aligned["pos_x_m"].notna()
        & aligned["pos_y_m"].notna()
    )

    next_timestamp = aligned["timestamp"].shift(-1)
    dt = next_timestamp - aligned["timestamp"]
    valid_dt = dt.between(0, 0.20, inclusive="both")
    aligned["time_weight_s"] = np.where(
        aligned["analysis_mask"] & valid_dt, dt, 0.0
    )
    aligned["auto_time_weight_s"] = np.where(
        aligned["is_auto"] & valid_dt, dt, 0.0
    )

    auto = aligned[aligned["analysis_mask"]].copy()
    if auto.empty:
        raise ValueError(
            "no Auto-mode driving samples with valid position and CTE were "
            "found"
        )
    auto["geometric_cte_m"] = calculate_geometric_cte(auto, audit)

    controller_lateral = summarize_controller_error(auto)
    geometric = summarize_distance(auto, "geometric_cte_m")
    by_kind: dict[str, dict[str, object]] = {}
    for kind, group in auto.groupby("kind", dropna=False):
        key = str(kind) if pd.notna(kind) else "unclassified"
        by_kind[key] = {
            "controller_lateral_error": summarize_controller_error(group),
            "geometric_path_error": summarize_distance(
                group, "geometric_cte_m"
            ),
        }

    field_auto = field[field["trans_mode"].eq(AUTO_MODE)].copy()
    mode_counts = {
        str(int(mode)): int(count)
        for mode, count in field["trans_mode"].value_counts().items()
    }
    field["mode_group"] = field["trans_mode"].ne(
        field["trans_mode"].shift()
    ).cumsum()
    mode_intervals: list[dict[str, float | int]] = []
    for _, group in field.groupby("mode_group"):
        start = float(group["field_timestamp"].iloc[0])
        end = float(group["field_timestamp"].iloc[-1])
        mode_intervals.append(
            {
                "mode": int(group["trans_mode"].iloc[0]),
                "start_epoch_s": start,
                "end_epoch_s": end,
                "duration_s": max(0.0, end - start),
                "samples": int(len(group)),
            }
        )

    worst_columns = [
        "elapsed_s",
        "lat",
        "lon",
        "waypoint_idx",
        "kind",
        "cte_m",
        "signed_cte_m",
        "speed_cmd_mps",
    ]
    worst = (
        auto.nlargest(10, "cte_m")[worst_columns]
        .replace({np.nan: None})
        .to_dict(orient="records")
    )

    auto_duration = float(aligned["auto_time_weight_s"].sum())
    driving_duration = float(aligned["time_weight_s"].sum())
    summary: dict[str, object] = {
        "site_name": paths.site_name,
        "run_id": paths.run_id,
        "run_directory": str(paths.run_dir),
        "definitions": {
            "auto_mode_value": AUTO_MODE,
            "auto_mode_source": (
                "field_test trans_mode; current tractor firmware defines "
                "mode 0 as Auto"
            ),
            "primary_population": (
                "Pure Pursuit cycles with valid CTE, position, "
                "controller driving=True, and nearest field logger "
                "trans_mode=0 within 0.15 s"
            ),
            "controller_cte": "cross_track_err_m = abs(yt_m)",
            "geometric_cte": (
                "minimum local XY distance from actual position to nearby "
                "mission polyline segments"
            ),
            "percent_time_method": (
                "Each 20 Hz controller sample is weighted by time to the "
                "next sample; gaps over 0.20 s receive zero weight"
            ),
        },
        "source_rows": {
            "pursuit_rows": int(len(pursuit)),
            "field_rows": int(len(field)),
            "aligned_without_mode": int(
                aligned["trans_mode"].isna().sum()
            ),
            "auto_driving_cte_samples": int(len(auto)),
        },
        "timing": {
            "pursuit_start_epoch_s": float(pursuit["timestamp"].min()),
            "pursuit_end_epoch_s": float(pursuit["timestamp"].max()),
            "pursuit_elapsed_s": float(
                pursuit["timestamp"].max() - pursuit["timestamp"].min()
            ),
            "analyzed_auto_driving_duration_s": controller_lateral[
                "duration_s"
            ],
            "aligned_auto_mode_duration_s": auto_duration,
            "auto_driving_availability_pct": (
                100.0 * driving_duration / auto_duration
                if auto_duration > 0
                else float("nan")
            ),
            "auto_wait_head_invalid_s": float(
                aligned.loc[
                    aligned["is_auto"] & ~aligned["driving_bool"],
                    "auto_time_weight_s",
                ].sum()
            ),
            "mode_intervals": mode_intervals,
        },
        "mode_counts_field_samples": mode_counts,
        "controller_logged_lateral_error_auto_driving": controller_lateral,
        "geometric_path_error_auto_driving": geometric,
        "by_path_kind": by_kind,
        "quality": {
            "rtk_fixed_pct_time": weighted_true_percent(
                auto["fix_quality"].eq("RTK Fixed"),
                auto["time_weight_s"],
            ),
            "valid_heading_pct_time": weighted_true_percent(
                auto["head_valid_bool"],
                auto["time_weight_s"],
            ),
            "commanded_speed_counts": {
                f"{float(speed):.2f}": int(count)
                for speed, count in auto["speed_cmd_mps"]
                .value_counts()
                .items()
            },
            "actual_speed_mps_auto_field": {
                "mean": float(field_auto["speed_mps"].mean()),
                "median": float(field_auto["speed_mps"].median()),
                "p95": float(field_auto["speed_mps"].quantile(0.95)),
                "max": float(field_auto["speed_mps"].max()),
            },
        },
        "worst_10_auto_driving": worst,
    }

    mission = pd.read_csv(
        paths.mission_path,
        sep=r"\s+",
        header=None,
        names=["lat", "lon", "yaw_rad", "lookahead_m", "speed_mps"],
    )
    require_columns(
        mission, ("lat", "lon"), paths.mission_path.name
    )
    boundary = pd.read_csv(paths.boundary_path)
    require_columns(
        boundary,
        ("sequence", "lat", "lon"),
        paths.boundary_path.name,
    )
    boundary = boundary.sort_values("sequence")

    actual_map = auto.iloc[::4].copy()
    if actual_map.index[-1] != auto.index[-1]:
        actual_map = pd.concat([actual_map, auto.tail(1)])

    map_data: dict[str, object] = {
        "metadata": {
            "site_name": paths.site_name,
            "run_id": paths.run_id,
        },
        "summary": {
            "controller_within_010_pct_time": controller_lateral[
                "within_0.10m_pct_time"
            ],
            "geometric_within_010_pct_time": geometric[
                "within_0.10m_pct_time"
            ],
            "geometric_within_025_pct_time": geometric[
                "within_0.25m_pct_time"
            ],
            "geometric_within_050_pct_time": geometric[
                "within_0.50m_pct_time"
            ],
            "geometric_mean_m": geometric["mean_m"],
            "geometric_p95_m": geometric["p95_m"],
            "geometric_max_m": geometric["max_m"],
            "duration_s": controller_lateral["duration_s"],
        },
        "target": [
            [round(float(row.lat), 9), round(float(row.lon), 9)]
            for row in mission.itertuples()
        ],
        "boundary": [
            [round(float(row.lat), 9), round(float(row.lon), 9)]
            for row in boundary.itertuples()
        ],
        "actual": [
            [
                round(float(row.lat), 9),
                round(float(row.lon), 9),
                round(float(row.cte_m), 3),
                round(float(row.geometric_cte_m), 3),
                round(float(row.elapsed_s), 2),
                int(row.waypoint_idx),
            ]
            for row in actual_map.itertuples()
        ],
    }
    return summary, map_data


def write_outputs(
    paths: RunPaths,
    summary: dict[str, object],
    map_data: dict[str, object],
) -> None:
    paths.output_dir.mkdir(parents=True, exist_ok=True)
    summary_json = json.dumps(summary, indent=2, allow_nan=False)
    map_json = json.dumps(map_data, separators=(",", ":"), allow_nan=False)
    paths.analysis_path.write_text(summary_json + "\n", encoding="utf-8")
    paths.map_data_path.write_text(map_json + "\n", encoding="utf-8")

    template = paths.template_path.read_text(encoding="utf-8")
    marker_count = template.count(MAP_DATA_MARKER)
    if marker_count != 1:
        raise ValueError(
            "map template must contain exactly one "
            f"{MAP_DATA_MARKER!r} marker; found {marker_count}"
        )
    paths.map_path.write_text(
        template.replace(MAP_DATA_MARKER, map_json),
        encoding="utf-8",
    )


def main() -> int:
    args = build_parser().parse_args()
    paths = resolve_paths(args)
    summary, map_data = analyze_run(paths)
    write_outputs(paths, summary, map_data)

    controller = summary[
        "controller_logged_lateral_error_auto_driving"
    ]
    geometric = summary["geometric_path_error_auto_driving"]
    print("Run analysis complete.")
    print(f"Site / run              : {paths.site_name} / {paths.run_id}")
    print(
        "Auto-driving duration  : "
        f"{float(controller['duration_s']):.1f} s"
    )
    print(
        "Controller CTE <= 0.10 : "
        f"{float(controller['within_0.10m_pct_time']):.1f}%"
    )
    print(
        "Geometric CTE <= 0.10  : "
        f"{float(geometric['within_0.10m_pct_time']):.1f}%"
    )
    print(f"Analysis JSON           : {paths.analysis_path}")
    print(f"Map data JSON           : {paths.map_data_path}")
    print(f"Interactive map         : {paths.map_path}")

    if args.open_map:
        webbrowser.open(paths.map_path.as_uri())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
