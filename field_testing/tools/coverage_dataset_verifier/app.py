#!/usr/bin/env python3
"""Local server for the coverage source-dataset verification tool.

The server intentionally uses only the Python standard library. It reads files
inside the repository, reports their metadata to the browser, and writes review
records only inside the user-selected repository-relative output directory.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import mimetypes
import os
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterable
from urllib.parse import parse_qs, unquote, urlparse
from uuid import uuid4


TOOL_DIR = Path(__file__).resolve().parent
STATIC_DIR = TOOL_DIR / "static"
DEFAULT_PROJECT_PATH = TOOL_DIR / "config" / "62_collins_sources.json"
REVIEW_STATUS = "REVIEW ONLY — NOT APPROVED FOR FIELD USE"
REQUIRED_PURPOSES = {
    "boundary_evidence",
    "obstacle_evidence",
    "transition_evidence",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_repo_path(repo_root: Path, relative_path: str) -> Path:
    if not isinstance(relative_path, str) or not relative_path.strip():
        raise ValueError("A repository-relative path is required.")
    normalized = relative_path.replace("\\", "/").lstrip("/")
    candidate = (repo_root / normalized).resolve()
    try:
        candidate.relative_to(repo_root)
    except ValueError as exc:
        raise ValueError("The path must stay inside the repository.") from exc
    return candidate


def number_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def update_range(current: list[float | None], value: Any) -> None:
    number = number_or_none(value)
    if number is None:
        return
    current[0] = number if current[0] is None else min(current[0], number)
    current[1] = number if current[1] is None else max(current[1], number)


def inspect_csv(path: Path) -> dict[str, Any]:
    row_count = 0
    columns: list[str] = []
    lat_range: list[float | None] = [None, None]
    lon_range: list[float | None] = [None, None]
    time_range: list[str | None] = [None, None]
    first_coordinate: dict[str, Any] | None = None
    lat_names = ("lat", "latitude")
    lon_names = ("lon", "lng", "longitude")
    time_names = ("time", "time_utc", "timestamp", "datetime")

    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = list(reader.fieldnames or [])
        lookup = {name.lower(): name for name in columns}
        lat_column = next((lookup[name] for name in lat_names if name in lookup), None)
        lon_column = next((lookup[name] for name in lon_names if name in lookup), None)
        time_column = next((lookup[name] for name in time_names if name in lookup), None)
        for row in reader:
            row_count += 1
            if lat_column:
                update_range(lat_range, row.get(lat_column))
            if lon_column:
                update_range(lon_range, row.get(lon_column))
            if time_column:
                value = (row.get(time_column) or "").strip()
                if value:
                    time_range[0] = value if time_range[0] is None else min(time_range[0], value)
                    time_range[1] = value if time_range[1] is None else max(time_range[1], value)
            lat = number_or_none(row.get(lat_column)) if lat_column else None
            lon = number_or_none(row.get(lon_column)) if lon_column else None
            if first_coordinate is None and lat is not None and lon is not None:
                first_coordinate = {
                    "dataRow": row_count,
                    "latitude": lat,
                    "longitude": lon,
                    "time": (row.get(time_column) or "").strip() if time_column else None,
                }

    return {
        "format": "csv",
        "columns": columns,
        "rowCount": row_count,
        "latitudeRange": lat_range if lat_range[0] is not None else None,
        "longitudeRange": lon_range if lon_range[0] is not None else None,
        "timeRange": time_range if time_range[0] is not None else None,
        "firstCoordinate": first_coordinate,
    }


def iter_geojson_coordinates(value: Any) -> Iterable[tuple[float, float]]:
    if not isinstance(value, list) or not value:
        return
    if len(value) >= 2 and number_or_none(value[0]) is not None and number_or_none(value[1]) is not None:
        yield float(value[0]), float(value[1])
        return
    for child in value:
        yield from iter_geojson_coordinates(child)


def inspect_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as stream:
        payload = json.load(stream)
    result: dict[str, Any] = {"format": "json"}
    if isinstance(payload, dict) and payload.get("type") == "FeatureCollection":
        features = payload.get("features") or []
        geometry_types: set[str] = set()
        lon_range: list[float | None] = [None, None]
        lat_range: list[float | None] = [None, None]
        for feature in features:
            geometry = (feature or {}).get("geometry") or {}
            if geometry.get("type"):
                geometry_types.add(str(geometry["type"]))
            for lon, lat in iter_geojson_coordinates(geometry.get("coordinates")):
                update_range(lon_range, lon)
                update_range(lat_range, lat)
        result.update(
            {
                "format": "geojson",
                "featureCount": len(features),
                "geometryTypes": sorted(geometry_types),
                "latitudeRange": lat_range if lat_range[0] is not None else None,
                "longitudeRange": lon_range if lon_range[0] is not None else None,
            }
        )
    elif isinstance(payload, dict):
        result["topLevelKeys"] = sorted(payload.keys())
    elif isinstance(payload, list):
        result["itemCount"] = len(payload)
    return result


def thin_points(points: list[Any], limit: int = 600) -> list[Any]:
    if len(points) <= limit:
        return points
    stride = max(1, math.ceil(len(points) / limit))
    thinned = points[::stride]
    if points[-1] != thinned[-1]:
        thinned.append(points[-1])
    return thinned


def simplify_geometry(geometry: dict[str, Any]) -> dict[str, Any]:
    geometry_type = geometry.get("type")
    coordinates = geometry.get("coordinates")
    if geometry_type in {"Point", "MultiPoint"}:
        simplified = coordinates
    elif geometry_type == "LineString":
        simplified = thin_points(coordinates or [])
    elif geometry_type == "MultiLineString":
        simplified = [thin_points(line or []) for line in coordinates or []]
    elif geometry_type == "Polygon":
        simplified = [thin_points(ring or []) for ring in coordinates or []]
    elif geometry_type == "MultiPolygon":
        simplified = [[thin_points(ring or []) for ring in polygon or []] for polygon in coordinates or []]
    else:
        simplified = coordinates
    return {"type": geometry_type, "coordinates": simplified}


def preview_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(payload, dict) or payload.get("type") != "FeatureCollection":
        raise ValueError("JSON preview currently requires a GeoJSON FeatureCollection.")
    features = []
    for index, feature in enumerate(payload.get("features") or []):
        properties = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        feature_id = str(
            feature.get("id")
            or properties.get("asset_id")
            or properties.get("feature_id")
            or properties.get("id")
            or f"feature-{index + 1}"
        )
        name = str(properties.get("asset_name") or properties.get("name") or feature_id)
        kind = str(properties.get("geometry_role") or properties.get("role") or properties.get("kind") or "unclassified")
        features.append(
            {
                "id": feature_id,
                "name": name,
                "kind": kind,
                "geometry": simplify_geometry(geometry),
                "reviewOnly": bool(properties.get("review_only", True)),
            }
        )
    return {"format": "geojson", "features": features}


def preview_csv(path: Path, max_points: int = 5000) -> dict[str, Any]:
    summary = inspect_csv(path)
    columns = summary.get("columns") or []
    lookup = {name.lower(): name for name in columns}
    lat_column = next((lookup[name] for name in ("lat", "latitude") if name in lookup), None)
    lon_column = next((lookup[name] for name in ("lon", "lng", "longitude") if name in lookup), None)
    time_column = next((lookup[name] for name in ("time", "time_utc", "timestamp", "datetime") if name in lookup), None)
    if not lat_column or not lon_column:
        raise ValueError("CSV preview requires latitude and longitude columns.")
    row_count = int(summary.get("rowCount") or 0)
    stride = max(1, math.ceil(row_count / max_points))
    coordinates: list[list[float]] = []
    samples: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as stream:
        reader = csv.DictReader(stream)
        for data_row, row in enumerate(reader, 1):
            if (data_row - 1) % stride:
                continue
            lat = number_or_none(row.get(lat_column))
            lon = number_or_none(row.get(lon_column))
            if lat is None or lon is None:
                continue
            coordinates.append([lon, lat])
            samples.append({"dataRow": data_row, "time": row.get(time_column) if time_column else None})
    return {
        "format": "csv",
        "sampleStride": stride,
        "originalRowCount": row_count,
        "features": [
            {
                "id": "recorded-path",
                "name": "Recorded latitude/longitude path",
                "kind": "raw_recorded_path",
                "geometry": {"type": "LineString", "coordinates": coordinates},
                "reviewOnly": True,
            }
        ],
        "samples": samples,
    }


def preview_source(repo_root: Path, relative_path: str) -> dict[str, Any]:
    path = safe_repo_path(repo_root, relative_path)
    if not path.is_file():
        raise ValueError("Preview source was not found.")
    if path.suffix.lower() in {".json", ".geojson"}:
        result = preview_json(path)
    elif path.suffix.lower() in {".csv", ".tsv"}:
        result = preview_csv(path)
    else:
        raise ValueError("Preview is available for CSV, JSON, and GeoJSON sources.")
    return {"ok": True, "path": relative_path.replace("\\", "/"), **result}


def source_purposes(source: dict[str, Any]) -> set[str]:
    purposes = source.get("purposes")
    if isinstance(purposes, list):
        return {str(value) for value in purposes}
    legacy_role = source.get("role")
    mapping = {
        "boundary_evidence": "boundary_evidence",
        "raw_gps_log": "obstacle_evidence",
        "obstacle_inventory": "obstacle_evidence",
        "transition_path": "transition_evidence",
        "supporting_evidence": "supporting_reference",
    }
    return {mapping[legacy_role]} if legacy_role in mapping else set()


def inspect_source(repo_root: Path, relative_path: str, expected_sha256: str = "") -> dict[str, Any]:
    path = safe_repo_path(repo_root, relative_path)
    if not path.exists():
        return {"ok": False, "path": relative_path, "errors": ["File not found."]}
    if not path.is_file():
        return {"ok": False, "path": relative_path, "errors": ["Path is not a file."]}

    digest = sha256_file(path)
    errors: list[str] = []
    warnings: list[str] = []
    expected = (expected_sha256 or "").lower().strip()
    if expected and expected != digest:
        errors.append("SHA-256 does not match the recorded expected hash.")
    if not expected:
        warnings.append("No expected hash was supplied; this hash will become the recorded baseline.")

    details: dict[str, Any] = {}
    suffix = path.suffix.lower()
    try:
        if suffix in {".csv", ".tsv"}:
            details = inspect_csv(path)
        elif suffix in {".json", ".geojson"}:
            details = inspect_json(path)
        else:
            details = {"format": suffix.lstrip(".") or "unknown"}
    except (csv.Error, UnicodeError, json.JSONDecodeError, OSError) as exc:
        errors.append(f"Content inspection failed: {exc}")

    return {
        "ok": not errors,
        "path": relative_path.replace("\\", "/"),
        "sha256": digest,
        "expectedSha256": expected or None,
        "hashMatches": not expected or expected == digest,
        "bytes": path.stat().st_size,
        "modifiedUtc": datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "errors": errors,
        "warnings": warnings,
        **details,
    }


def validate_project(project: dict[str, Any], repo_root: Path, require_approval: bool) -> tuple[list[str], list[dict[str, Any]]]:
    errors: list[str] = []
    if project.get("status") != REVIEW_STATUS:
        errors.append(f"Project status must remain '{REVIEW_STATUS}'.")
    if require_approval and not str(project.get("siteId", "")).strip():
        errors.append("Site ID is required.")
    if require_approval and not str(project.get("siteName", "")).strip():
        errors.append("Friendly site name is required.")
    if require_approval and number_or_none((project.get("equipment") or {}).get("deckWidthM")) is None:
        errors.append("Deck width must be a number.")

    sources = project.get("sources") or []
    purposes = set().union(*(source_purposes(source) for source in sources if source.get("required", True))) if sources else set()
    if require_approval:
        for purpose in sorted(REQUIRED_PURPOSES - purposes):
            errors.append(f"At least one required source used for '{purpose}' is needed.")
        if not any(source.get("required", True) and source.get("sourceType") == "raw_recording" for source in sources):
            errors.append("At least one required raw recording is needed.")

    inspected: list[dict[str, Any]] = []
    source_ids: set[str] = set()
    for source in sources:
        source_id = str(source.get("id", "")).strip()
        if not source_id or source_id in source_ids:
            errors.append("Every source must have a unique, non-empty ID.")
            continue
        source_ids.add(source_id)
        if str(source.get("path", "")).strip():
            result = inspect_source(repo_root, str(source.get("path", "")), str(source.get("expectedSha256", "")))
            inspected.append({"sourceId": source_id, **result})
            if require_approval and source.get("required", True) and not result["ok"]:
                errors.append(f"Required source '{source_id}' did not pass verification.")
        elif require_approval and source.get("required", True):
            errors.append(f"Required source '{source_id}' needs a file path.")
        if require_approval and source.get("required", True) and source.get("sourceDecision") != "use_this_source":
            errors.append(f"Required source '{source_id}' must have the decision 'use_this_source'.")

    frame_id = str((project.get("coordinateFrame") or {}).get("sourceId", "")).strip()
    if require_approval and (not frame_id or frame_id not in source_ids):
        errors.append("Coordinate-frame source must identify one of the listed sources.")
    elif require_approval and not any(item["sourceId"] == frame_id and item["ok"] for item in inspected):
        errors.append("Coordinate-frame source must pass verification.")

    if require_approval:
        approval = project.get("sourceApproval") or {}
        if not str(approval.get("reviewer", "")).strip():
            errors.append("Reviewer name is required to approve Tab 1.")
        if not str(approval.get("reviewDate", "")).strip():
            errors.append("Review date is required to approve Tab 1.")
        if not approval.get("attestation"):
            errors.append("The source-verification attestation must be checked.")
    return errors, inspected


def append_audit_event(ledger_path: Path, event_type: str, reviewer: str, details: dict[str, Any]) -> dict[str, Any]:
    if ledger_path.exists():
        ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    else:
        ledger = {"schemaVersion": 1, "status": REVIEW_STATUS, "events": []}
    events = ledger.setdefault("events", [])
    previous_hash = events[-1]["eventHash"] if events else None
    event = {
        "sequence": len(events) + 1,
        "eventId": str(uuid4()),
        "eventType": event_type,
        "timestampUtc": utc_now(),
        "reviewer": reviewer or None,
        "previousEventHash": previous_hash,
        "details": details,
    }
    event["eventHash"] = hashlib.sha256(canonical_json(event)).hexdigest()
    events.append(event)
    atomic_write_json(ledger_path, ledger)
    return event


def atomic_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def save_project(repo_root: Path, project: dict[str, Any], approve: bool) -> dict[str, Any]:
    errors, inspected = validate_project(project, repo_root, require_approval=approve)
    if errors:
        return {"ok": False, "errors": errors, "inspection": inspected}

    output_dir = safe_repo_path(repo_root, str(project.get("outputDirectory", "")))
    project_copy = json.loads(json.dumps(project))
    project_copy["schemaVersion"] = 1
    project_copy["status"] = REVIEW_STATUS
    project_copy["updatedUtc"] = utc_now()
    project_copy["sourceVerification"] = {
        "state": "approved_as_source_inputs" if approve else "draft",
        "verifiedUtc": utc_now(),
        "sources": inspected,
    }
    project_copy["gates"] = {
        "sources": "complete" if approve else "draft",
        "boundaries": "locked_not_implemented",
        "obstacles": "locked_not_implemented",
        "transitions": "locked_not_implemented",
        "datasetApproval": "blocked",
    }
    project_copy["fieldValidationState"] = "not_approved"
    project_copy["missionApprovalState"] = "not_approved"
    project_copy["coverageGenerationAllowed"] = False
    project_copy["executableMissionGenerated"] = False

    output_dir.mkdir(parents=True, exist_ok=True)
    project_path = output_dir / "site-project.json"
    ledger_path = output_dir / "approval-ledger.json"
    previous_revision = 0
    if project_path.exists():
        try:
            previous_revision = int(json.loads(project_path.read_text(encoding="utf-8")).get("projectRevision", 0))
        except (ValueError, TypeError, json.JSONDecodeError):
            previous_revision = 0
    project_copy["projectRevision"] = previous_revision + 1
    frame_id = str((project_copy.get("coordinateFrame") or {}).get("sourceId", ""))
    frame_inspection = next((item for item in inspected if item.get("sourceId") == frame_id), None)
    if frame_inspection and frame_inspection.get("firstCoordinate"):
        project_copy["coordinateFrame"]["resolvedOrigin"] = frame_inspection["firstCoordinate"]
    event = append_audit_event(
        ledger_path,
        "sources_approved" if approve else "sources_draft_saved",
        str((project.get("sourceApproval") or {}).get("reviewer", "")).strip(),
        {
            "siteId": project_copy["siteId"],
            "projectRevision": project_copy["projectRevision"],
            "sourceCount": len(project_copy.get("sources") or []),
            "requiredSourceCount": sum(1 for source in project_copy.get("sources") or [] if source.get("required", True)),
            "projectSha256": hashlib.sha256(canonical_json(project_copy)).hexdigest(),
            "comments": str((project.get("sourceApproval") or {}).get("comments", "")).strip() or None,
        },
    )
    project_copy["latestAuditEvent"] = {"sequence": event["sequence"], "eventHash": event["eventHash"]}
    atomic_write_json(project_path, project_copy)
    return {
        "ok": True,
        "approved": approve,
        "projectPath": project_path.relative_to(repo_root).as_posix(),
        "ledgerPath": ledger_path.relative_to(repo_root).as_posix(),
        "event": event,
        "project": project_copy,
    }


class CoverageReviewHandler(SimpleHTTPRequestHandler):
    repo_root: Path

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"[{self.log_date_time_string()}] {format_string % args}")

    def send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(encoded)

    def read_json(self) -> Any:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length > 5_000_000:
            raise ValueError("Request is too large.")
        return json.loads(self.rfile.read(content_length).decode("utf-8"))

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/default-project":
            self.send_json(json.loads(DEFAULT_PROJECT_PATH.read_text(encoding="utf-8")))
            return
        if parsed.path == "/api/health":
            self.send_json({"ok": True, "status": REVIEW_STATUS})
            return
        if parsed.path == "/repo-file":
            try:
                query = parse_qs(parsed.query)
                path = safe_repo_path(self.repo_root, unquote((query.get("path") or [""])[0]))
                if not path.is_file():
                    raise ValueError("Review file was not found.")
                payload = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", mimetypes.guess_type(path.name)[0] or "application/octet-stream")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
            except (ValueError, OSError) as exc:
                self.send_json({"ok": False, "errors": [str(exc)]}, HTTPStatus.NOT_FOUND)
            return
        super().do_GET()

    def do_POST(self) -> None:
        try:
            payload = self.read_json()
            if self.path == "/api/inspect":
                result = inspect_source(self.repo_root, payload.get("path", ""), payload.get("expectedSha256", ""))
                self.send_json(result, HTTPStatus.OK if result["ok"] else HTTPStatus.UNPROCESSABLE_ENTITY)
                return
            if self.path == "/api/preview":
                result = preview_source(self.repo_root, payload.get("path", ""))
                self.send_json(result)
                return
            if self.path in {"/api/save-draft", "/api/approve-sources"}:
                result = save_project(self.repo_root, payload, approve=self.path.endswith("approve-sources"))
                self.send_json(result, HTTPStatus.OK if result["ok"] else HTTPStatus.UNPROCESSABLE_ENTITY)
                return
            self.send_json({"ok": False, "errors": ["Unknown API endpoint."]}, HTTPStatus.NOT_FOUND)
        except (ValueError, json.JSONDecodeError) as exc:
            self.send_json({"ok": False, "errors": [str(exc)]}, HTTPStatus.BAD_REQUEST)
        except OSError as exc:
            self.send_json({"ok": False, "errors": [f"File operation failed: {exc}"]}, HTTPStatus.INTERNAL_SERVER_ERROR)


def find_repo_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / "AGENTS.md").exists() and (candidate / "field_testing").exists():
            return candidate.resolve()
    raise RuntimeError("Could not locate the tractor2025 repository root.")


def create_server(repo_root: Path, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("ConfiguredCoverageReviewHandler", (CoverageReviewHandler,), {"repo_root": repo_root.resolve()})
    return ThreadingHTTPServer((host, port), handler)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local coverage dataset verifier.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--repo-root", type=Path)
    args = parser.parse_args()
    repo_root = (args.repo_root or find_repo_root(TOOL_DIR)).resolve()
    server = create_server(repo_root, args.host, args.port)
    print(REVIEW_STATUS)
    print(f"Coverage source verifier: http://{args.host}:{args.port}")
    print(f"Repository root: {repo_root}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
