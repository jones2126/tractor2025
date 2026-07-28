#!/usr/bin/env node
/**
 * Build a reusable Excel workbook for straight-row Pure Pursuit diagnostics.
 *
 * Only samples whose active mission waypoint belongs to an audited `stripe`
 * block and whose tractor position projects onto that finite straight segment
 * are included. Headlands, connectors/keyholes, and terminal circling are
 * therefore excluded by mission geometry rather than elapsed-time guesses.
 *
 * Example:
 *   node build_straight_row_workbook_20260728.mjs \
 *     --run-dir C:\...\runs\20260727_144659 \
 *     --mission-dir C:\...\62_Collins_polygon_1 \
 *     --site-name 62_Collins_polygon_1 \
 *     --run-id 20260727_144659 \
 *     --output C:\...\straight_row_diagnostics_20260727_144659.xlsx
 */

import fs from "node:fs/promises";
import path from "node:path";
import process from "node:process";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const AUTO_MODE = 0;
const ALIGN_TOLERANCE_S = 0.15;
const WORKBOOK_SAMPLE_RATE_HZ = 5;
const WORKBOOK_SAMPLE_INTERVAL_S = 1 / WORKBOOK_SAMPLE_RATE_HZ;
const STEADY_END_EXCLUSION_M = 3.0;
const STEADY_HEADING_LIMIT_DEG = 15.0;
const STEER_CENTER = 447;
const STEER_RIGHT = 197;
const STEER_LEFT = 815;
const STEER_DEADBAND = 10;
const STEER_MIN_PWM = 150;
const STEER_MAX_PWM = 255;
const CURRENT_KP = 1.0;
const CURRENT_KI = 0.0;
const CURRENT_KD = 0.0;
const TEENSY_STATUS_INTERVAL_S = 2.0;

function parseArgs(argv) {
  const result = {};
  for (let i = 2; i < argv.length; i += 1) {
    const key = argv[i];
    if (!key.startsWith("--")) throw new Error(`Unexpected argument: ${key}`);
    if (key === "--help") {
      result.help = true;
      continue;
    }
    const value = argv[i + 1];
    if (value === undefined || value.startsWith("--")) {
      throw new Error(`Missing value for ${key}`);
    }
    result[key.slice(2)] = value;
    i += 1;
  }
  const required = ["run-dir", "mission-dir", "site-name", "run-id", "output"];
  if (!result.help) {
    for (const key of required) {
      if (!result[key]) throw new Error(`Required argument missing: --${key}`);
    }
  }
  return result;
}

function usage() {
  return [
    "Usage:",
    "  node build_straight_row_workbook_20260728.mjs \\",
    "    --run-dir <run directory> --mission-dir <mission directory> \\",
    "    --site-name <site> --run-id <YYYYMMDD_HHMMSS> --output <xlsx> \\",
    "    [--preview-dir <directory>]",
  ].join("\n");
}

function parseCsv(text) {
  const rows = [];
  let row = [];
  let field = "";
  let quoted = false;
  for (let i = 0; i < text.length; i += 1) {
    const char = text[i];
    if (quoted) {
      if (char === '"' && text[i + 1] === '"') {
        field += '"';
        i += 1;
      } else if (char === '"') {
        quoted = false;
      } else {
        field += char;
      }
    } else if (char === '"') {
      quoted = true;
    } else if (char === ",") {
      row.push(field);
      field = "";
    } else if (char === "\n") {
      row.push(field.replace(/\r$/, ""));
      rows.push(row);
      row = [];
      field = "";
    } else {
      field += char;
    }
  }
  if (field.length || row.length) {
    row.push(field.replace(/\r$/, ""));
    rows.push(row);
  }
  if (!rows.length) return [];
  const headers = rows[0];
  return rows.slice(1).filter((values) => values.some((v) => v !== "")).map(
    (values) => Object.fromEntries(headers.map((header, i) => [header, values[i] ?? ""])),
  );
}

function number(value) {
  if (value === null || value === undefined || value === "") return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function bool(value) {
  return String(value).trim().toLowerCase() === "true";
}

function wrapDegrees(value) {
  return ((value + 180) % 360 + 360) % 360 - 180;
}

function headingFromVector(dx, dy) {
  return ((90 - Math.atan2(dy, dx) * 180 / Math.PI) % 360 + 360) % 360;
}

function compassFromYaw(yawRad) {
  return ((90 - yawRad * 180 / Math.PI) % 360 + 360) % 360;
}

function quartileForRank(rank, count) {
  const fraction = (rank - 0.5) / count;
  if (fraction <= 0.25) return "South";
  if (fraction <= 0.50) return "South-mid";
  if (fraction <= 0.75) return "North-mid";
  return "North";
}

function nearestField(fieldRows, timestamp) {
  let low = 0;
  let high = fieldRows.length - 1;
  while (low <= high) {
    const middle = Math.floor((low + high) / 2);
    if (fieldRows[middle].fieldTimestamp < timestamp) low = middle + 1;
    else high = middle - 1;
  }
  const candidates = [];
  if (low < fieldRows.length) candidates.push(fieldRows[low]);
  if (low > 0) candidates.push(fieldRows[low - 1]);
  if (!candidates.length) return null;
  candidates.sort(
    (a, b) => Math.abs(a.fieldTimestamp - timestamp) - Math.abs(b.fieldTimestamp - timestamp),
  );
  return Math.abs(candidates[0].fieldTimestamp - timestamp) <= ALIGN_TOLERANCE_S
    ? candidates[0]
    : null;
}

function average(values) {
  const finite = values.filter(Number.isFinite);
  return finite.length ? finite.reduce((sum, value) => sum + value, 0) / finite.length : null;
}

function slope(xValues, yValues) {
  const pairs = xValues.map((x, i) => [x, yValues[i]]).filter(
    ([x, y]) => Number.isFinite(x) && Number.isFinite(y),
  );
  if (pairs.length < 3) return null;
  const meanX = average(pairs.map(([x]) => x));
  const meanY = average(pairs.map(([, y]) => y));
  let numerator = 0;
  let denominator = 0;
  for (const [x, y] of pairs) {
    numerator += (x - meanX) * (y - meanY);
    denominator += (x - meanX) ** 2;
  }
  return denominator ? numerator / denominator : null;
}

function actualSteerNormalized(sample) {
  if (!Number.isFinite(sample.steerCurrent)) return null;
  return sample.steerCurrent >= STEER_CENTER
    ? (sample.steerCurrent - STEER_CENTER) / (STEER_LEFT - STEER_CENTER)
    : (sample.steerCurrent - STEER_CENTER) / (STEER_CENTER - STEER_RIGHT);
}

function meanMetric(samples, selector) {
  return average(samples.map(selector));
}

function absOrNull(value) {
  return Number.isFinite(value) ? Math.abs(value) : null;
}

function summarizeSamples(samples) {
  const activePwm = samples.filter((sample) => Number.isFinite(sample.pwm) && sample.pwm > 0);
  return {
    count: samples.length,
    geomSigned: meanMetric(samples, (sample) => sample.geometricSigned),
    geomAbs: meanMetric(samples, (sample) => absOrNull(sample.geometricSigned)),
    ytSigned: meanMetric(samples, (sample) => sample.controllerYt),
    ytAbs: meanMetric(samples, (sample) => absOrNull(sample.controllerYt)),
    headingSigned: meanMetric(samples, (sample) => sample.headingError),
    headingAbs: meanMetric(samples, (sample) => absOrNull(sample.headingError)),
    steerMean: meanMetric(samples, (sample) => sample.steerNormalized),
    steerAbs: meanMetric(samples, (sample) => absOrNull(sample.steerNormalized)),
    actualSteerMean: meanMetric(samples, actualSteerNormalized),
    normalizedTracking: meanMetric(
      samples,
      (sample) => (
        Number.isFinite(sample.steerNormalized) && Number.isFinite(actualSteerNormalized(sample))
          ? sample.steerNormalized - actualSteerNormalized(sample)
          : null
      ),
    ),
    steerError: meanMetric(samples, (sample) => sample.steerError),
    steerErrorAbs: meanMetric(samples, (sample) => absOrNull(sample.steerError)),
    pwmActive: samples.length ? activePwm.length / samples.length : null,
    pwmAtMinimumWhenActive: activePwm.length
      ? activePwm.filter((sample) => sample.pwm === STEER_MIN_PWM).length / activePwm.length
      : 0,
    pwmSaturated: samples.length
      ? samples.filter((sample) => Number.isFinite(sample.pwm) && sample.pwm >= 250).length
        / samples.length
      : null,
    activePwmMean: meanMetric(activePwm, (sample) => sample.pwm),
    speedCommand: meanMetric(samples, (sample) => sample.speedCommand),
    actualSpeed: meanMetric(samples, (sample) => sample.actualSpeed),
    speedShortfall: meanMetric(
      samples,
      (sample) => (
        Number.isFinite(sample.speedCommand) && Number.isFinite(sample.actualSpeed)
          ? sample.speedCommand - sample.actualSpeed
          : null
      ),
    ),
  };
}

function columnName(index) {
  let value = index + 1;
  let result = "";
  while (value > 0) {
    const remainder = (value - 1) % 26;
    result = String.fromCharCode(65 + remainder) + result;
    value = Math.floor((value - 1) / 26);
  }
  return result;
}

function styleTitle(sheet, range, title) {
  sheet.getRange(range).merge();
  const cell = sheet.getRange(range.split(":")[0]);
  cell.values = [[title]];
  sheet.getRange(range).format = {
    fill: "#17365D",
    font: { bold: true, color: "#FFFFFF", size: 18 },
    verticalAlignment: "center",
  };
  sheet.getRange(range).format.rowHeight = 32;
}

function styleSection(range) {
  range.format = {
    fill: "#D9EAF7",
    font: { bold: true, color: "#17365D" },
    borders: { preset: "outside", style: "thin", color: "#9FBAD0" },
  };
}

function styleHeader(range) {
  range.format = {
    fill: "#4472C4",
    font: { bold: true, color: "#FFFFFF" },
    horizontalAlignment: "center",
    verticalAlignment: "center",
    wrapText: true,
    borders: { preset: "inside", style: "thin", color: "#B4C6E7" },
  };
  range.format.rowHeight = 42;
}

function setColumnWidths(sheet, widths) {
  for (const [column, width] of Object.entries(widths)) {
    sheet.getRange(`${column}:${column}`).format.columnWidth = width;
  }
}

async function loadAndAnalyze(args) {
  const runDir = path.resolve(args["run-dir"]);
  const missionDir = path.resolve(args["mission-dir"]);
  const site = args["site-name"];
  const runId = args["run-id"];
  const pursuitPath = path.join(runDir, `pursuit_log_${runId}.csv`);
  const fieldPath = path.join(runDir, `field_test_${runId}.csv`);
  const auditPath = path.join(missionDir, `${site}_supervised_test_mission_audit.csv`);

  const [pursuitText, fieldText, auditText] = await Promise.all([
    fs.readFile(pursuitPath, "utf8"),
    fs.readFile(fieldPath, "utf8"),
    fs.readFile(auditPath, "utf8"),
  ]);
  const pursuitRows = parseCsv(pursuitText).map((row) => ({
    ...row,
    timestampNumber: number(row.timestamp),
    elapsedNumber: number(row.elapsed_s),
    waypointNumber: number(row.waypoint_idx),
  })).filter((row) => Number.isFinite(row.timestampNumber));
  const fieldRows = parseCsv(fieldText).map((row) => ({
    ...row,
    fieldTimestamp: Date.parse(row.time) / 1000,
    modeNumber: number(row.trans_mode),
  })).filter((row) => Number.isFinite(row.fieldTimestamp)).sort(
    (a, b) => a.fieldTimestamp - b.fieldTimestamp,
  );
  const auditRows = parseCsv(auditText).map((row) => ({
    ...row,
    waypointNumber: number(row.waypoint),
    waypointIndex: number(row.waypoint) - 1,
    east: number(row.east_m),
    north: number(row.north_m),
    yaw: number(row.yaw_rad),
  })).filter((row) => Number.isFinite(row.waypointIndex));

  let stripeId = 0;
  let inStripe = false;
  const auditByIndex = new Map();
  for (const row of auditRows) {
    if (row.kind === "stripe") {
      if (!inStripe) stripeId += 1;
      row.stripeId = stripeId;
      inStripe = true;
    } else {
      row.stripeId = null;
      inStripe = false;
    }
    auditByIndex.set(row.waypointIndex, row);
  }

  const originEast = auditRows[0].east;
  const originNorth = auditRows[0].north;
  const stripeGroups = new Map();
  for (const row of auditRows.filter((item) => item.stripeId !== null)) {
    if (!stripeGroups.has(row.stripeId)) stripeGroups.set(row.stripeId, []);
    stripeGroups.get(row.stripeId).push(row);
  }
  const stripeMeta = [];
  for (const [id, rows] of stripeGroups) {
    const first = rows[0];
    const last = rows[rows.length - 1];
    const dx = last.east - first.east;
    const dy = last.north - first.north;
    const length = Math.hypot(dx, dy);
    stripeMeta.push({
      rowId: id,
      startWaypoint: first.waypointIndex,
      endWaypoint: last.waypointIndex,
      startX: first.east - originEast,
      startY: first.north - originNorth,
      endX: last.east - originEast,
      endY: last.north - originNorth,
      unitX: dx / length,
      unitY: dy / length,
      length,
      meanNorth: average(rows.map((row) => row.north)),
      targetHeading: headingFromVector(dx, dy),
      direction: dx >= 0 ? "Eastbound" : "Westbound",
    });
  }
  stripeMeta.sort((a, b) => a.meanNorth - b.meanNorth);
  const observedIds = new Set();
  const metaById = new Map(stripeMeta.map((meta) => [meta.rowId, meta]));
  const rawSamples = [];
  let lastKeptElapsed = -Infinity;
  let previousSnapshot = null;
  let snapshotId = 0;

  for (const pursuit of pursuitRows) {
    if (!bool(pursuit.driving)) continue;
    const audit = auditByIndex.get(pursuit.waypointNumber);
    if (!audit || audit.kind !== "stripe" || audit.stripeId === null) continue;
    const field = nearestField(fieldRows, pursuit.timestampNumber);
    if (!field || field.modeNumber !== AUTO_MODE) continue;
    const meta = metaById.get(audit.stripeId);
    const posX = number(pursuit.pos_x_m);
    const posY = number(pursuit.pos_y_m);
    const actualHeading = number(pursuit.heading_compass_deg);
    if (![posX, posY, actualHeading].every(Number.isFinite)) continue;
    const offsetX = posX - meta.startX;
    const offsetY = posY - meta.startY;
    const alongTrack = meta.unitX * offsetX + meta.unitY * offsetY;
    if (alongTrack < 0 || alongTrack > meta.length) continue;
    const geometricSigned = -meta.unitY * offsetX + meta.unitX * offsetY;
    const progress = alongTrack / meta.length;
    const headingError = wrapDegrees(actualHeading - meta.targetHeading);
    const phase = progress < 1 / 3 ? "Early" : progress < 2 / 3 ? "Middle" : "Late";
    const steadyState = (
      alongTrack >= STEADY_END_EXCLUSION_M
      && alongTrack <= meta.length - STEADY_END_EXCLUSION_M
      && Math.abs(headingError) <= STEADY_HEADING_LIMIT_DEG
    );
    const snapshot = [
      field.steer_setpoint,
      field.steer_current,
      field.steer_error,
      field.steer_pwm,
    ].join("|");
    if (snapshot !== previousSnapshot) {
      snapshotId += 1;
      previousSnapshot = snapshot;
    }
    const targetX = number(pursuit.target_x_m);
    const targetY = number(pursuit.target_y_m);
    const targetDistance = (
      Number.isFinite(targetX) && Number.isFinite(targetY)
        ? Math.hypot(targetX - posX, targetY - posY)
        : null
    );
    if (
      Number.isFinite(pursuit.elapsedNumber)
      && pursuit.elapsedNumber - lastKeptElapsed < WORKBOOK_SAMPLE_INTERVAL_S - 0.01
    ) {
      continue;
    }
    lastKeptElapsed = pursuit.elapsedNumber;
    rawSamples.push({
      sampleNumber: rawSamples.length + 1,
      timestamp: pursuit.timestampNumber,
      elapsed: pursuit.elapsedNumber,
      rowId: meta.rowId,
      northRank: null,
      quartile: null,
      direction: meta.direction,
      waypointIndex: pursuit.waypointNumber,
      pathLength: meta.length,
      alongTrack,
      progress,
      phase,
      steadyState,
      targetHeading: Number.isFinite(audit.yaw) ? compassFromYaw(audit.yaw) : meta.targetHeading,
      actualHeading,
      headingError,
      targetX,
      targetY,
      controllerYt: number(pursuit.yt_m),
      geometricSigned,
      lookahead: number(pursuit.lookahead_dist_m),
      alpha: number(pursuit.alpha_deg),
      delta: number(pursuit.delta_deg),
      steerNormalized: number(pursuit.steer_normalized),
      loggedSetpoint: number(field.steer_setpoint),
      steerCurrent: number(field.steer_current),
      steerError: number(field.steer_error),
      pwm: number(field.steer_pwm),
      speedCommand: number(pursuit.speed_cmd_mps),
      actualSpeed: number(field.actual_speed_mps ?? field.speed_mps),
      targetDistance,
      snapshotId,
      fixQuality: pursuit.fix_quality,
      headValid: bool(pursuit.head_valid),
    });
    observedIds.add(meta.rowId);
  }

  const observedMeta = stripeMeta.filter((meta) => observedIds.has(meta.rowId));
  observedMeta.sort((a, b) => a.meanNorth - b.meanNorth);
  observedMeta.forEach((meta, index) => {
    meta.northRank = index + 1;
    meta.quartile = quartileForRank(index + 1, observedMeta.length);
  });
  for (const sample of rawSamples) {
    const meta = metaById.get(sample.rowId);
    sample.northRank = meta.northRank;
    sample.quartile = meta.quartile;
  }

  const driftByRow = new Map();
  for (const meta of observedMeta) {
    const steady = rawSamples.filter(
      (sample) => sample.rowId === meta.rowId && sample.steadyState,
    );
    const drift = slope(
      steady.map((sample) => sample.alongTrack),
      steady.map((sample) => sample.geometricSigned),
    );
    driftByRow.set(meta.rowId, Number.isFinite(drift) ? drift * 10 : null);
  }

  return {
    site,
    runId,
    runDir,
    missionDir,
    pursuitPath,
    fieldPath,
    auditPath,
    sourceCounts: {
      pursuit: pursuitRows.length,
      field: fieldRows.length,
      audit: auditRows.length,
    },
    rawSamples,
    observedMeta,
    driftByRow,
  };
}

function rawRowValues(sample) {
  const expectedSetpoint = Number.isFinite(sample.steerNormalized)
    ? (
      sample.steerNormalized >= 0
        ? STEER_CENTER + sample.steerNormalized * (STEER_LEFT - STEER_CENTER)
        : STEER_CENTER + sample.steerNormalized * (STEER_CENTER - STEER_RIGHT)
    )
    : null;
  const actualSteerNormalized = Number.isFinite(sample.steerCurrent)
    ? (
      sample.steerCurrent >= STEER_CENTER
        ? (sample.steerCurrent - STEER_CENTER) / (STEER_LEFT - STEER_CENTER)
        : (sample.steerCurrent - STEER_CENTER) / (STEER_CENTER - STEER_RIGHT)
    )
    : null;
  const expectedPwm = Number.isFinite(sample.steerError)
    ? (
      Math.abs(sample.steerError) <= STEER_DEADBAND
        ? 0
        : Math.min(
          STEER_MAX_PWM,
          Math.max(STEER_MIN_PWM, Math.abs(sample.steerError) * CURRENT_KP),
        )
    )
    : null;
  return [
    sample.sampleNumber,
    sample.timestamp,
    sample.elapsed,
    sample.rowId,
    sample.northRank,
    sample.quartile,
    sample.direction,
    sample.waypointIndex,
    sample.pathLength,
    sample.alongTrack,
    sample.progress,
    sample.phase,
    sample.steadyState,
    sample.targetHeading,
    sample.actualHeading,
    sample.headingError,
    sample.targetX,
    sample.targetY,
    sample.controllerYt,
    Number.isFinite(sample.controllerYt) ? Math.abs(sample.controllerYt) : null,
    sample.geometricSigned,
    Number.isFinite(sample.geometricSigned) ? Math.abs(sample.geometricSigned) : null,
    sample.lookahead,
    sample.alpha,
    sample.delta,
    sample.steerNormalized,
    expectedSetpoint,
    sample.loggedSetpoint,
    sample.steerCurrent,
    sample.steerError,
    Number.isFinite(sample.steerError) ? Math.abs(sample.steerError) : null,
    actualSteerNormalized,
    (
      Number.isFinite(sample.steerNormalized) && Number.isFinite(actualSteerNormalized)
        ? sample.steerNormalized - actualSteerNormalized
        : null
    ),
    sample.pwm,
    Number.isFinite(sample.pwm) && Number.isFinite(sample.steerError)
      ? Math.sign(sample.steerError) * sample.pwm
      : null,
    Number.isFinite(sample.pwm) ? sample.pwm > 0 : null,
    Number.isFinite(sample.pwm) ? sample.pwm === STEER_MIN_PWM : null,
    Number.isFinite(sample.pwm) ? sample.pwm >= 250 : null,
    Number.isFinite(sample.steerError) ? Math.abs(sample.steerError) <= STEER_DEADBAND : null,
    expectedPwm,
    Number.isFinite(sample.pwm) && Number.isFinite(expectedPwm) ? sample.pwm - expectedPwm : null,
    sample.speedCommand,
    sample.actualSpeed,
    (
      Number.isFinite(sample.speedCommand) && Number.isFinite(sample.actualSpeed)
        ? sample.speedCommand - sample.actualSpeed
        : null
    ),
    sample.targetDistance,
    sample.snapshotId,
    sample.fixQuality,
    sample.headValid,
    Number.isFinite(sample.headingError) ? Math.abs(sample.headingError) : null,
    Number.isFinite(sample.steerNormalized) ? Math.abs(sample.steerNormalized) : null,
  ];
}

function averageIfFormula(rawColumn, rowIdCell, extraCriteria = []) {
  const last = "__LAST__";
  const pieces = [
    `=AVERAGEIFS('Straight Samples'!$${rawColumn}$2:$${rawColumn}$${last}`,
    `'Straight Samples'!$D$2:$D$${last},${rowIdCell}`,
  ];
  for (const [column, criterion] of extraCriteria) {
    pieces.push(`'Straight Samples'!$${column}$2:$${column}$${last},${criterion}`);
  }
  return pieces.join(",") + ")";
}

function countIfFormula(rowIdCell, extraCriteria = []) {
  const last = "__LAST__";
  const pieces = [
    `=COUNTIFS('Straight Samples'!$D$2:$D$${last},${rowIdCell}`,
  ];
  for (const [column, criterion] of extraCriteria) {
    pieces.push(`'Straight Samples'!$${column}$2:$${column}$${last},${criterion}`);
  }
  return pieces.join(",") + ")";
}

function averageCriteriaFormula(rawColumn, criteria) {
  const last = "__LAST__";
  const pieces = [`=AVERAGEIFS('Straight Samples'!$${rawColumn}$2:$${rawColumn}$${last}`];
  for (const [column, criterion] of criteria) {
    pieces.push(`'Straight Samples'!$${column}$2:$${column}$${last},${criterion}`);
  }
  return pieces.join(",") + ")";
}

function countCriteriaFormula(criteria) {
  const last = "__LAST__";
  const pairs = criteria.map(
    ([column, criterion]) =>
      `'Straight Samples'!$${column}$2:$${column}$${last},${criterion}`,
  );
  return `=COUNTIFS(${pairs.join(",")})`;
}

function replaceLast(formula, lastRawRow) {
  return formula.replaceAll("__LAST__", String(lastRawRow));
}

function writeMixedRows(sheet, startRowIndex, startColumnIndex, rows) {
  if (!rows.length) return;
  const values = rows.map((row) => row.map(
    (value) => typeof value === "string" && value.startsWith("=") ? null : value,
  ));
  sheet.getRangeByIndexes(
    startRowIndex,
    startColumnIndex,
    rows.length,
    rows[0].length,
  ).values = values;
  for (let column = 0; column < rows[0].length; column += 1) {
    if (!rows.some((row) => typeof row[column] === "string" && row[column].startsWith("="))) {
      continue;
    }
    const formulas = rows.map((row) => [
      typeof row[column] === "string" && row[column].startsWith("=") ? row[column] : null,
    ]);
    sheet.getRangeByIndexes(
      startRowIndex,
      startColumnIndex + column,
      rows.length,
      1,
    ).formulas = formulas;
  }
}

async function buildWorkbook(data, outputPath, previewDir) {
  console.log(`Building workbook for ${data.rawSamples.length} straight samples...`);
  const workbook = Workbook.create();
  const overview = workbook.worksheets.add("Overview");
  const rowSummary = workbook.worksheets.add("Row Summary");
  const rowSegments = workbook.worksheets.add("Row Segments");
  const northDirection = workbook.worksheets.add("North-Direction");
  const raw = workbook.worksheets.add("Straight Samples");
  const definitions = workbook.worksheets.add("Definitions & PID");
  for (const sheet of [overview, rowSummary, rowSegments, northDirection, raw, definitions]) {
    sheet.showGridLines = false;
  }

  const rawHeaders = [
    "Sample #", "Timestamp epoch (s)", "Elapsed (s)", "Row ID", "North rank",
    "Geographic quartile", "Direction", "Active waypoint index", "Path length (m)",
    "Along-track (m)", "Progress", "Phase", "Steady state", "Target heading (deg)",
    "Actual heading (deg)", "Heading error (deg)", "Target X (m)", "Target Y (m)",
    "Controller yt (m)", "|Controller yt| (m)", "Geometric signed CTE (m)",
    "|Geometric CTE| (m)", "Lookahead (m)", "PP alpha (deg)", "PP delta (deg)",
    "PP steer normalized", "Expected pot setpoint", "Logged pot setpoint",
    "Steer current", "Steer error (counts)", "|Steer error|", "Actual steer normalized",
    "PP command vs pot snapshot gap", "PWM magnitude", "Signed PWM", "PWM active",
    "PWM at minimum drive", "PWM saturated", "Within 10-count deadband",
    "Expected PWM (Kp=1)", "PWM model mismatch", "Speed command (m/s)",
    "Actual speed (m/s)", "Speed shortfall (m/s)", "Target distance (m)",
    "Steering snapshot ID", "Fix quality", "Heading valid", "|Heading error| (deg)",
    "|PP steer normalized|",
  ];
  raw.getRange(`A1:${columnName(rawHeaders.length - 1)}1`).values = [rawHeaders];
  styleHeader(raw.getRange(`A1:${columnName(rawHeaders.length - 1)}1`));
  const rawValues = data.rawSamples.map(rawRowValues);
  if (rawValues.some((row) => row.length !== rawHeaders.length)) {
    throw new Error(
      `Straight Samples column mismatch: ${rawHeaders.length} headers, `
      + `${rawValues[0]?.length ?? 0} values`,
    );
  }
  const lastRawRow = rawValues.length + 1;
  raw.getRangeByIndexes(1, 0, rawValues.length, rawHeaders.length).values = rawValues;
  console.log("Straight Samples values written.");
  raw.freezePanes.freezeRows(1);
  raw.freezePanes.freezeColumns(7);
  raw.tables.add(`A1:${columnName(rawHeaders.length - 1)}${lastRawRow}`, true, "StraightSamplesTable");
  raw.getRange(`B2:B${lastRawRow}`).format.numberFormat = "0.000";
  raw.getRange(`C2:C${lastRawRow}`).format.numberFormat = "0.000";
  raw.getRange(`I2:J${lastRawRow}`).format.numberFormat = "0.000";
  raw.getRange(`K2:K${lastRawRow}`).format.numberFormat = "0.0%";
  raw.getRange(`N2:P${lastRawRow}`).format.numberFormat = "0.00";
  raw.getRange(`Q2:Y${lastRawRow}`).format.numberFormat = "0.000";
  raw.getRange(`Z2:AG${lastRawRow}`).format.numberFormat = "0.000";
  raw.getRange(`AH2:AO${lastRawRow}`).format.numberFormat = "0";
  raw.getRange(`AP2:AS${lastRawRow}`).format.numberFormat = "0.000";
  raw.getRange(`AW2:AX${lastRawRow}`).format.numberFormat = "0.000";
  setColumnWidths(raw, {
    A: 10, B: 17, C: 12, D: 9, E: 10, F: 15, G: 12, H: 14,
    I: 12, J: 12, K: 10, L: 10, M: 11, N: 14, O: 14, P: 13,
    Q: 11, R: 11, S: 13, T: 13, U: 16, V: 16, W: 11, X: 11,
    Y: 11, Z: 13, AA: 14, AB: 14, AC: 12, AD: 14, AE: 12, AF: 14,
    AG: 15, AH: 12, AI: 11, AJ: 10, AK: 14, AL: 12, AM: 16, AN: 14,
    AO: 14, AP: 14, AQ: 13, AR: 14, AS: 13, AT: 13, AU: 13, AV: 12,
    AW: 14, AX: 16,
  });
  raw.getRange(`V2:V${lastRawRow}`).conditionalFormats.add("colorScale", {
    colors: ["#C6EFCE", "#FFEB9C", "#FFC7CE"],
    thresholds: ["min", "50%", "max"],
  });
  raw.getRange(`AE2:AE${lastRawRow}`).conditionalFormats.add("colorScale", {
    colors: ["#C6EFCE", "#FFEB9C", "#FFC7CE"],
    thresholds: ["min", "50%", "max"],
  });
  console.log("Straight Samples formatting complete.");

  const summaryHeaders = [
    "Row ID", "North rank", "Mean northing (m)", "Quartile", "Direction",
    "Target heading (deg)", "Length (m)", "Samples", "Duration (s)",
    "Steady samples", "All geom signed CTE (m)", "All geom MAE (m)",
    "Steady geom signed CTE (m)", "Steady geom MAE (m)", "CTE drift per 10 m",
    "All controller yt signed (m)", "Steady controller yt signed (m)",
    "Steady controller yt MAE (m)", "Steady heading error (deg)",
    "Steady heading MAE (deg)", "Steady PP steer mean", "Steady |PP steer|",
    "Steady actual steer normalized", "Steady PP-vs-pot snapshot gap",
    "Steady steer error (counts)", "Steady |steer error|", "PWM active",
    "PWM at 150 when active", "PWM saturated", "Mean active PWM",
    "Speed command (m/s)", "Actual speed (m/s)", "Speed shortfall (m/s)",
    "Early geom signed (m)", "Middle geom signed (m)", "Late geom signed (m)",
    "Early heading error (deg)", "Middle heading error (deg)", "Late heading error (deg)",
    "Diagnostic flag",
  ];
  styleTitle(rowSummary, "A1:AN1", "Straight-row diagnostic summary");
  rowSummary.getRange("A2:AN2").merge();
  rowSummary.getRange("A2").values = [[
    `Only audited stripe samples are included. "Steady" means ≥${STEADY_END_EXCLUSION_M.toFixed(0)} m from both row ends and |heading error| ≤${STEADY_HEADING_LIMIT_DEG.toFixed(0)}°.`,
  ]];
  rowSummary.getRange("A2:AN2").format = {
    fill: "#EAF2F8", font: { italic: true, color: "#44546A" }, wrapText: true,
  };
  rowSummary.getRange("A5:AN5").values = [summaryHeaders];
  styleHeader(rowSummary.getRange("A5:AN5"));
  const summaryStart = 6;
  const summaryRows = data.observedMeta.map((meta, index) => {
    const row = summaryStart + index;
    const allSamples = data.rawSamples.filter((sample) => sample.rowId === meta.rowId);
    const steadySamples = allSamples.filter((sample) => sample.steadyState);
    const all = summarizeSamples(allSamples);
    const steady = summarizeSamples(steadySamples);
    const early = summarizeSamples(allSamples.filter((sample) => sample.phase === "Early"));
    const middle = summarizeSamples(allSamples.filter((sample) => sample.phase === "Middle"));
    const late = summarizeSamples(allSamples.filter((sample) => sample.phase === "Late"));
    return [
      meta.rowId,
      meta.northRank,
      meta.meanNorth,
      meta.quartile,
      meta.direction,
      meta.targetHeading,
      meta.length,
      all.count,
      `=H${row}/${WORKBOOK_SAMPLE_RATE_HZ}`,
      steady.count,
      all.geomSigned,
      all.geomAbs,
      steady.geomSigned,
      steady.geomAbs,
      data.driftByRow.get(meta.rowId),
      all.ytSigned,
      steady.ytSigned,
      steady.ytAbs,
      steady.headingSigned,
      steady.headingAbs,
      steady.steerMean,
      steady.steerAbs,
      steady.actualSteerMean,
      steady.normalizedTracking,
      steady.steerError,
      steady.steerErrorAbs,
      steady.pwmActive,
      steady.pwmAtMinimumWhenActive,
      steady.pwmSaturated,
      steady.activePwmMean,
      steady.speedCommand,
      steady.actualSpeed,
      `=AE${row}-AF${row}`,
      early.geomSigned,
      middle.geomSigned,
      late.geomSigned,
      early.headingSigned,
      middle.headingSigned,
      late.headingSigned,
      `=IF(N${row}>=0.4,"High persistent CTE",IF(Z${row}>=15,"Steering tracking error",IF(AA${row}>=0.25,"Frequent PWM activity","Review")))`,
    ];
  });
  const summaryLast = summaryStart + summaryRows.length - 1;
  writeMixedRows(rowSummary, summaryStart - 1, 0, summaryRows);
  rowSummary.tables.add(`A5:AN${summaryLast}`, true, "RowSummaryTable");
  rowSummary.freezePanes.freezeRows(5);
  rowSummary.freezePanes.freezeColumns(7);
  rowSummary.getRange(`C${summaryStart}:C${summaryLast}`).format.numberFormat = "0.000";
  rowSummary.getRange(`F${summaryStart}:G${summaryLast}`).format.numberFormat = "0.00";
  rowSummary.getRange(`H${summaryStart}:J${summaryLast}`).format.numberFormat = "0";
  rowSummary.getRange(`K${summaryStart}:R${summaryLast}`).format.numberFormat = "0.000";
  rowSummary.getRange(`S${summaryStart}:T${summaryLast}`).format.numberFormat = "0.00";
  rowSummary.getRange(`U${summaryStart}:Z${summaryLast}`).format.numberFormat = "0.000";
  rowSummary.getRange(`AA${summaryStart}:AC${summaryLast}`).format.numberFormat = "0.0%";
  rowSummary.getRange(`AD${summaryStart}:AD${summaryLast}`).format.numberFormat = "0";
  rowSummary.getRange(`AE${summaryStart}:AM${summaryLast}`).format.numberFormat = "0.000";
  setColumnWidths(rowSummary, {
    A: 9, B: 10, C: 13, D: 13, E: 12, F: 14, G: 10, H: 10, I: 11,
    J: 12, K: 14, L: 13, M: 16, N: 14, O: 13, P: 16, Q: 16, R: 15,
    S: 15, T: 14, U: 13, V: 13, W: 15, X: 16, Y: 15, Z: 14, AA: 11,
    AB: 14, AC: 11, AD: 13, AE: 13, AF: 13, AG: 14, AH: 14, AI: 15,
    AJ: 14, AK: 15, AL: 16, AM: 14, AN: 22,
  });
  rowSummary.getRange(`N${summaryStart}:N${summaryLast}`).conditionalFormats.add("colorScale", {
    colors: ["#C6EFCE", "#FFEB9C", "#FFC7CE"], thresholds: ["min", "50%", "max"],
  });
  rowSummary.getRange(`Z${summaryStart}:Z${summaryLast}`).conditionalFormats.add("colorScale", {
    colors: ["#C6EFCE", "#FFEB9C", "#FFC7CE"], thresholds: ["min", "50%", "max"],
  });
  console.log("Row Summary complete.");

  styleTitle(rowSegments, "A1:N1", "Early / middle / late row comparison");
  const segmentHeaders = [
    "Row ID", "North rank", "Quartile", "Direction", "Segment", "Samples",
    "Geom signed CTE (m)", "Geom MAE (m)", "Controller yt signed (m)",
    "Heading error (deg)", "PP steer mean", "|Steer error| (counts)",
    "PWM active", "Actual speed (m/s)",
  ];
  rowSegments.getRange("A4:N4").values = [segmentHeaders];
  styleHeader(rowSegments.getRange("A4:N4"));
  const segmentRows = [];
  for (const meta of data.observedMeta) {
    for (const phase of ["Early", "Middle", "Late"]) {
      const samples = data.rawSamples.filter(
        (sample) => sample.rowId === meta.rowId && sample.phase === phase,
      );
      const summary = summarizeSamples(samples);
      segmentRows.push([
        meta.rowId, meta.northRank, meta.quartile, meta.direction, phase,
        summary.count,
        summary.geomSigned,
        summary.geomAbs,
        summary.ytSigned,
        summary.headingSigned,
        summary.steerMean,
        summary.steerErrorAbs,
        summary.pwmActive,
        summary.actualSpeed,
      ]);
    }
  }
  const segmentLast = 4 + segmentRows.length;
  writeMixedRows(rowSegments, 4, 0, segmentRows);
  rowSegments.tables.add(`A4:N${segmentLast}`, true, "RowSegmentsTable");
  rowSegments.freezePanes.freezeRows(4);
  rowSegments.getRange(`F5:F${segmentLast}`).format.numberFormat = "0";
  rowSegments.getRange(`G5:L${segmentLast}`).format.numberFormat = "0.000";
  rowSegments.getRange(`M5:M${segmentLast}`).format.numberFormat = "0.0%";
  rowSegments.getRange(`N5:N${segmentLast}`).format.numberFormat = "0.000";
  setColumnWidths(rowSegments, {
    A: 9, B: 10, C: 13, D: 12, E: 10, F: 10, G: 16, H: 13,
    I: 16, J: 15, K: 13, L: 17, M: 12, N: 14,
  });
  console.log("Row Segments complete.");

  styleTitle(northDirection, "A1:L1", "Northing and travel-direction comparison");
  northDirection.getRange("A2:L2").merge();
  northDirection.getRange("A2").values = [[
    "Steady-state samples only. This table separates geographic degradation from eastbound/westbound load effects.",
  ]];
  northDirection.getRange("A2:L2").format = {
    fill: "#EAF2F8", font: { italic: true, color: "#44546A" },
  };
  const ndHeaders = [
    "Quartile", "Direction", "Samples", "Geom signed CTE (m)", "Geom MAE (m)",
    "Controller yt signed (m)", "Heading error (deg)", "PP steer mean",
    "|Steer error| (counts)", "PWM active", "Actual speed (m/s)", "Speed shortfall (m/s)",
  ];
  northDirection.getRange("A5:L5").values = [ndHeaders];
  styleHeader(northDirection.getRange("A5:L5"));
  const quartiles = ["South", "South-mid", "North-mid", "North"];
  const directions = ["Eastbound", "Westbound"];
  const ndRows = [];
  for (const quartile of quartiles) {
    for (const direction of directions) {
      const samples = data.rawSamples.filter(
        (sample) => (
          sample.quartile === quartile
          && sample.direction === direction
          && sample.steadyState
        ),
      );
      const summary = summarizeSamples(samples);
      ndRows.push([
        quartile,
        direction,
        summary.count,
        summary.geomSigned,
        summary.geomAbs,
        summary.ytSigned,
        summary.headingSigned,
        summary.steerMean,
        summary.steerErrorAbs,
        summary.pwmActive,
        summary.actualSpeed,
        summary.speedShortfall,
      ]);
    }
  }
  const ndLast = 5 + ndRows.length;
  writeMixedRows(northDirection, 5, 0, ndRows);
  northDirection.tables.add(`A5:L${ndLast}`, true, "NorthDirectionTable");
  northDirection.getRange(`C6:C${ndLast}`).format.numberFormat = "0";
  northDirection.getRange(`D6:I${ndLast}`).format.numberFormat = "0.000";
  northDirection.getRange(`J6:J${ndLast}`).format.numberFormat = "0.0%";
  northDirection.getRange(`K6:L${ndLast}`).format.numberFormat = "0.000";
  setColumnWidths(northDirection, {
    A: 13, B: 13, C: 10, D: 16, E: 13, F: 17, G: 15, H: 13,
    I: 17, J: 12, K: 14, L: 16,
  });
  console.log("North-Direction complete.");

  styleTitle(definitions, "A1:F1", "Definitions, firmware context, and PID interpretation");
  definitions.getRange("A3:B3").values = [["Analysis setting", "Value"]];
  styleHeader(definitions.getRange("A3:B3"));
  const settingsRows = [
    ["Steady-state end exclusion", `${STEADY_END_EXCLUSION_M.toFixed(1)} m from each row end`],
    ["Steady-state heading gate", `|heading error| ≤ ${STEADY_HEADING_LIMIT_DEG.toFixed(0)}°`],
    ["Field-log alignment tolerance", `${ALIGN_TOLERANCE_S.toFixed(2)} s`],
    ["Workbook diagnostic sample rate", `${WORKBOOK_SAMPLE_RATE_HZ} Hz`],
    ["AUTO mode value", AUTO_MODE],
    ["Current firmware Kp / Ki / Kd", `${CURRENT_KP.toFixed(1)} / ${CURRENT_KI.toFixed(1)} / ${CURRENT_KD.toFixed(1)}`],
    ["Steering deadband", `${STEER_DEADBAND} pot counts`],
    ["Minimum non-zero PWM", STEER_MIN_PWM],
    ["Maximum PWM", STEER_MAX_PWM],
    ["Teensy steering status print interval", `${TEENSY_STATUS_INTERVAL_S.toFixed(1)} s`],
  ];
  definitions.getRange(`A4:B${3 + settingsRows.length}`).values = settingsRows;
  definitions.getRange("A15:F15").merge();
  definitions.getRange("A15").values = [["Metric definitions"]];
  styleSection(definitions.getRange("A15:F15"));
  const definitionRows = [
    ["Geometric signed CTE", "Tractor position relative to the finite straight line. Positive means tractor is left of travel direction.", "", "", "", ""],
    ["Controller yt", "Selected target lateral position in tractor coordinates. Positive means target is left of the tractor. Its sign is opposite geometric tractor offset during correction.", "", "", "", ""],
    ["Heading error", "Actual compass heading minus audited target compass heading, wrapped to ±180°.", "", "", "", ""],
    ["Steady state", `At least ${STEADY_END_EXCLUSION_M.toFixed(0)} m from both row ends and within ${STEADY_HEADING_LIMIT_DEG.toFixed(0)}° of target heading. Intended to remove keyhole acquisition and row-exit behavior.`, "", "", "", ""],
    ["CTE drift per 10 m", "Least-squares slope of steady geometric signed CTE versus along-track distance, multiplied by 10.", "", "", "", ""],
    ["Signed PWM", "PWM magnitude with sign inferred from steer_error: positive=left, negative=right. The logger does not record direction separately.", "", "", "", ""],
    ["Steering snapshot ID", `Changes when the repeated Teensy steering status tuple changes. Steering telemetry is printed only every ${TEENSY_STATUS_INTERVAL_S.toFixed(0)} s, so PWM is diagnostic snapshot data, not a 20 Hz waveform.`, "", "", "", ""],
    ["PP-vs-pot snapshot gap", "Current 5 Hz Pure Pursuit command minus the slower repeated pot snapshot. Useful for context, but not a synchronized low-level tracking error. Use logged steer_error for the internally consistent setpoint-current error.", "", "", "", ""],
  ];
  definitions.getRange(`A16:F${15 + definitionRows.length}`).values = definitionRows;
  for (let row = 16; row <= 15 + definitionRows.length; row += 1) {
    definitions.getRange(`B${row}:F${row}`).merge();
  }
  definitions.getRange("A25:F25").merge();
  definitions.getRange("A25").values = [["PID interpretation and safe test sequence"]];
  styleSection(definitions.getRange("A25:F25"));
  const pidRows = [
    ["1", "The current loop is P-only with a 10-count deadband and a 150-PWM minimum. That behaves more like deadband plus a strong pulse than a continuously proportional low-output controller.", "", "", "", ""],
    ["2", "If steady steer error stays mostly inside ±10 counts while CTE grows, changing PID gains is unlikely to fix the primary problem. Inspect mechanics, steering-center calibration, GPS-to-base offset, lateral slope, and Pure Pursuit behavior first.", "", "", "", ""],
    ["3", "If persistent setpoint-current error remains outside the deadband without PWM saturation, test a smaller deadband and/or calibrated minimum-PWM compensation before adding integral gain.", "", "", "", ""],
    ["4", "A small Ki may remove steady pot-count bias, but only after adding integral reset/anti-windup and testing on jack stands at low speed. Do not copy the historical Ki=0.02 directly into field operation without step testing.", "", "", "", ""],
    ["5", "Increase Kp only with measured steering step-response evidence. The present 150 minimum PWM can make a larger Kp more abrupt rather than more accurate.", "", "", "", ""],
    ["6", "Kd can damp overshoot but differentiates noisy pot feedback. Add filtering and measure oscillation before using it. Tune the low-level steering loop before changing Pure Pursuit lookahead.", "", "", "", ""],
  ];
  definitions.getRange(`A26:F${25 + pidRows.length}`).values = pidRows;
  for (let row = 26; row <= 25 + pidRows.length; row += 1) {
    definitions.getRange(`B${row}:F${row}`).merge();
  }
  definitions.getRange("A34:F34").merge();
  definitions.getRange("A34").values = [["Source files"]];
  styleSection(definitions.getRange("A34:F34"));
  definitions.getRange("A35:B37").values = [
    ["Pure Pursuit log", data.pursuitPath],
    ["Field logger", data.fieldPath],
    ["Mission audit", data.auditPath],
  ];
  definitions.getRange("A3:F40").format.wrapText = true;
  definitions.getRange("A3:F40").format.verticalAlignment = "top";
  setColumnWidths(definitions, { A: 30, B: 34, C: 18, D: 18, E: 18, F: 18 });
  definitions.getRange("A3:F40").format.autofitRows();
  console.log("Definitions & PID complete.");

  styleTitle(overview, "A1:H1", `Straight-row diagnostics — ${data.site} / ${data.runId}`);
  overview.getRange("A3:H3").merge();
  overview.getRange("A3").values = [[
    "Purpose: explain persistent north-versus-south accuracy differences using straight-row geometry, Pure Pursuit state, steering feedback, and PWM snapshots.",
  ]];
  overview.getRange("A3:H3").format = {
    fill: "#EAF2F8", font: { color: "#44546A" }, wrapText: true,
  };
  overview.getRange("A5:B5").values = [["Run scope", "Value"]];
  styleHeader(overview.getRange("A5:B5"));
  overview.getRange("A6:B13").values = [
    ["Straight samples", data.rawSamples.length],
    ["Workbook diagnostic rate", `${WORKBOOK_SAMPLE_RATE_HZ} Hz`],
    ["Straight rows observed", data.observedMeta.length],
    ["Source pursuit rows", data.sourceCounts.pursuit],
    ["Source field rows", data.sourceCounts.field],
    ["Excluded geometry", "Headlands, connectors/keyholes, terminal circling"],
    ["Steady-state rule", `≥${STEADY_END_EXCLUSION_M.toFixed(0)} m from ends; |heading error|≤${STEADY_HEADING_LIMIT_DEG.toFixed(0)}°`],
    ["Workbook generated", new Date()],
  ];
  overview.getRange("B6:B10").format.numberFormat = "#,##0";
  overview.getRange("B13").format.numberFormat = "yyyy-mm-dd hh:mm";
  overview.getRange("D5:H5").merge();
  overview.getRange("D5").values = [["How to read this workbook"]];
  styleSection(overview.getRange("D5:H5"));
  overview.getRange("D6:H12").merge();
  overview.getRange("D6").values = [[
    "Start with Row Summary, sorted south to north. Compare steady geometric CTE with controller yt: geometric CTE measures where the tractor actually drove, while yt measures where the selected lookahead target sits in tractor coordinates. Use Row Segments to see whether error enters with the turn, grows along the row, or remains persistent. Use North-Direction to separate location effects from eastbound/westbound load. Filter Straight Samples for 5 Hz diagnostic evidence; the source pursuit log remains 20 Hz, while PWM and pot feedback are repeated slower Teensy snapshots.",
  ]];
  overview.getRange("D6:H12").format = {
    fill: "#F2F2F2", wrapText: true, verticalAlignment: "top",
    borders: { preset: "outside", style: "thin", color: "#BFBFBF" },
  };
  overview.getRange("A15:H15").merge();
  overview.getRange("A15").values = [["Evidence to evaluate before changing PID gains"]];
  styleSection(overview.getRange("A15:H15"));
  overview.getRange("A16:H21").values = [
    ["Pattern", "Interpretation", "", "", "", "", "", ""],
    ["CTE grows northward in both directions", "Location/terrain/mechanical load remains more likely than a single uphill-direction explanation.", "", "", "", "", "", ""],
    ["Large geometric CTE but small steady steer error", "Low-level PID is following its setpoint; inspect Pure Pursuit target behavior, calibration, tire slip, slope, and mechanics.", "", "", "", "", "", ""],
    ["Persistent steer error outside ±10 counts", "Deadband/minimum-PWM behavior or insufficient low-level response may contribute.", "", "", "", "", "", ""],
    ["PWM frequently 150", "Controller is repeatedly using the minimum motor-moving pulse. A smaller calibrated deadband/minimum-PWM strategy may be more useful than simply increasing Kp.", "", "", "", "", "", ""],
    ["PWM near 255 with persistent error", "Actuator saturation/load would be supported. This run previously showed little saturation.", "", "", "", "", "", ""],
  ];
  for (let row = 16; row <= 21; row += 1) overview.getRange(`B${row}:H${row}`).merge();
  overview.getRange("A16:H16").format = {
    fill: "#D9EAD3", font: { bold: true, color: "#274E13" },
  };
  overview.getRange("A16:H21").format.wrapText = true;
  overview.getRange("A16:H21").format.verticalAlignment = "top";
  overview.getRange("A16:H21").format.autofitRows();
  setColumnWidths(overview, { A: 29, B: 16, C: 3, D: 16, E: 16, F: 16, G: 16, H: 16 });

  const accuracyChart = overview.charts.add("line", {
    chartType: "line",
    title: "Persistent straight-row error increases northward",
    hasLegend: true,
  });
  const geomSeries = accuracyChart.series.add("Steady geometric MAE (m)");
  geomSeries.categoryFormula = `'Row Summary'!$B$${summaryStart}:$B$${summaryLast}`;
  geomSeries.formula = `'Row Summary'!$N$${summaryStart}:$N$${summaryLast}`;
  geomSeries.fill = "#1565C0";
  const ytSeries = accuracyChart.series.add("Steady controller yt MAE (m)");
  ytSeries.categoryFormula = `'Row Summary'!$B$${summaryStart}:$B$${summaryLast}`;
  ytSeries.formula = `'Row Summary'!$R$${summaryStart}:$R$${summaryLast}`;
  ytSeries.fill = "#C62828";
  accuracyChart.title = "Persistent straight-row error increases northward";
  accuracyChart.hasLegend = true;
  accuracyChart.xAxis = { axisType: "textAxis" };
  accuracyChart.xAxis.title.text = "North rank (1 = southernmost)";
  accuracyChart.yAxis = { numberFormatCode: "0.00" };
  accuracyChart.yAxis.title.text = "Mean absolute error (m)";
  accuracyChart.setPosition("A24", "H42");

  const steeringChart = overview.charts.add("line", {
    chartType: "line",
    title: "Steering tracking error by row",
    hasLegend: false,
  });
  const trackingSeries = steeringChart.series.add("Steady |steer error| (pot counts)");
  trackingSeries.categoryFormula = `'Row Summary'!$B$${summaryStart}:$B$${summaryLast}`;
  trackingSeries.formula = `'Row Summary'!$Z$${summaryStart}:$Z$${summaryLast}`;
  trackingSeries.fill = "#EF6C00";
  steeringChart.title = "Steering tracking error by row";
  steeringChart.hasLegend = false;
  steeringChart.xAxis = { axisType: "textAxis" };
  steeringChart.xAxis.title.text = "North rank (1 = southernmost)";
  steeringChart.yAxis = { numberFormatCode: "0.0" };
  steeringChart.yAxis.title.text = "Mean absolute pot error (counts)";
  steeringChart.setPosition("A44", "H61");

  overview.freezePanes.freezeRows(1);
  console.log("Overview and charts complete.");

  const inspectSummary = await workbook.inspect({
    kind: "table",
    range: `Row Summary!A5:AN${Math.min(summaryLast, 15)}`,
    include: "values,formulas",
    tableMaxRows: 12,
    tableMaxCols: 40,
  });
  console.log(inspectSummary.ndjson);
  const errorScan = await workbook.inspect({
    kind: "match",
    searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
    options: { useRegex: true, maxResults: 300 },
    summary: "final formula error scan",
  });
  console.log(errorScan.ndjson);

  if (previewDir) {
    await fs.mkdir(previewDir, { recursive: true });
    const previews = [
      ["Overview", "A1:H61", "overview.png"],
      ["Row Summary", "A1:Z16", "row_summary_left.png"],
      ["Row Summary", "AA1:AN16", "row_summary_right.png"],
      ["Row Segments", "A1:N18", "row_segments.png"],
      ["North-Direction", "A1:L14", "north_direction.png"],
      ["Straight Samples", "A1:P18", "straight_samples_left.png"],
      ["Straight Samples", "Q1:AX18", "straight_samples_right.png"],
      ["Definitions & PID", "A1:F40", "definitions_pid.png"],
    ];
    for (const [sheetName, range, filename] of previews) {
      console.log(`Rendering ${sheetName} ${range}...`);
      const blob = await workbook.render({ sheetName, range, scale: 1, format: "png" });
      await fs.writeFile(
        path.join(previewDir, filename),
        new Uint8Array(await blob.arrayBuffer()),
      );
    }
  }

  await fs.mkdir(path.dirname(outputPath), { recursive: true });
  const exported = await SpreadsheetFile.exportXlsx(workbook);
  console.log("Workbook exported; saving file...");
  await exported.save(outputPath);
  return {
    outputPath,
    sampleCount: data.rawSamples.length,
    rowCount: data.observedMeta.length,
    lastRawRow,
    summaryLast,
  };
}

async function main() {
  const args = parseArgs(process.argv);
  if (args.help) {
    console.log(usage());
    return;
  }
  const data = await loadAndAnalyze(args);
  if (!data.rawSamples.length) {
    throw new Error("No aligned AUTO straight-row samples were found.");
  }
  const result = await buildWorkbook(
    data,
    path.resolve(args.output),
    args["preview-dir"] ? path.resolve(args["preview-dir"]) : null,
  );
  console.log(JSON.stringify(result, null, 2));
}

main().catch((error) => {
  console.error(error.stack || String(error));
  process.exitCode = 1;
});
