import fs from "node:fs/promises";
import path from "node:path";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const repoRoot = "C:/Repos/tractor2025";
const analysisDir = path.join(repoRoot, "field_testing/jrk/20260906/analysis");
const outputDir = path.join(repoRoot, "outputs/jrk_20260906");
const outputPath = path.join(outputDir, "jrk_actuator_performance_20260906.xlsx");
const previewDir = path.join(outputDir, "previews");
const fontFamily = "Arial";

function parseCsv(text) {
  const lines = text.trim().split(/\r?\n/);
  const headers = lines[0].split(",");
  return lines.slice(1).filter(Boolean).map((line) => {
    const cells = line.split(",");
    const row = {};
    headers.forEach((header, index) => {
      const value = cells[index] ?? "";
      row[header] = value !== "" && /^-?\d+(\.\d+)?$/.test(value) ? Number(value) : value;
    });
    return row;
  });
}

async function readCsv(name) {
  return parseCsv(await fs.readFile(path.join(analysisDir, name), "utf8"));
}

function matrix(rows, columns) {
  return rows.map((row) => columns.map((column) => row[column] ?? ""));
}

function styleTitle(sheet, range) {
  range.format.font = { name: fontFamily, size: 16, bold: true, color: "#1F2937" };
}

function styleHeader(range) {
  range.format.fill = "#334155";
  range.format.font = { name: fontFamily, size: 10, bold: true, color: "#FFFFFF" };
  range.format.horizontalAlignment = "center";
  range.format.verticalAlignment = "center";
  range.format.wrapText = true;
  range.format.borders = { preset: "inside", style: "thin", color: "#FFFFFF" };
}

function styleBody(range) {
  range.format.font = { name: fontFamily, size: 10, color: "#1F2937" };
  range.format.verticalAlignment = "center";
}

const summary = JSON.parse(
  await fs.readFile(path.join(analysisDir, "jrk_analysis_summary_20260906.json"), "utf8")
);
const positionMap = await readCsv("jrk_position_performance_map_20260906.csv");
const movements = await readCsv("jrk_movement_summary_20260906.csv");
const guardSamples = await readCsv("jrk_guard_samples_20260906.csv");
const baselineSamples = await readCsv("jrk_baseline_samples_20260905.csv");

const workbook = Workbook.create();
const summarySheet = workbook.worksheets.add("Summary");
const mapSheet = workbook.worksheets.add("Position Map");
const movesSheet = workbook.worksheets.add("Movements");
const speedSheet = workbook.worksheets.add("Ground Speed Map");
const samplesSheet = workbook.worksheets.add("Guard Samples");
const baselineSheet = workbook.worksheets.add("Baseline Samples");

// Summary
summarySheet.showGridLines = false;
summarySheet.getRange("A1:F1").merge();
summarySheet.getRange("A1").values = [["JRK actuator performance"]];
styleTitle(summarySheet, summarySheet.getRange("A1"));
summarySheet.getRange("A2:F2").merge();
summarySheet.getRange("A2").values = [["Engine-off field tests, September 5–6, 2026"]];
summarySheet.getRange("A2").format.font = { name: fontFamily, size: 10, italic: true, color: "#64748B" };
summarySheet.getRange("A3:F3").format.borders = { bottom: { style: "thin", color: "#94A3B8" } };

const findings = [
  ["Lowest successful target", summary.lowest_successful_target],
  ["Lowest achieved feedback", summary.lowest_feedback],
  ["Highest staged forward peak (A)", summary.max_staged_probe_peak_A],
  ["Highest successful return peak (A)", summary.max_successful_return_peak_A],
  ["Successful staged forward moves", summary.successful_probe_count],
  ["Mechanical stop found", "No — not reached by target 1880"],
  ["Ground speed available", "No — GPS driving test still required"],
];
summarySheet.getRange("A5:B5").values = [["Finding", "Value"]];
styleHeader(summarySheet.getRange("A5:B5"));
summarySheet.getRange(`A6:B${5 + findings.length}`).values = findings;
styleBody(summarySheet.getRange(`A6:B${5 + findings.length}`));
summarySheet.getRange("A6:A12").format.font = { name: fontFamily, size: 10, bold: true, color: "#334155" };
summarySheet.getRange("B8:B9").format.numberFormat = "0.000";
summarySheet.getRange("A14:F14").values = [[
  "Target", "Average peak (A)", "Maximum peak (A)",
  "Average rate (counts/s)", "Average duration (ms)", "Trials",
]];
styleHeader(summarySheet.getRange("A14:F14"));
const summaryRows = positionMap.map((row) => [
  row.target, row.avg_peak_current_A, row.max_peak_current_A,
  row.avg_rate_counts_per_s, row.avg_duration_ms, row.trials,
]);
summarySheet.getRange(`A15:F${14 + summaryRows.length}`).values = summaryRows;
styleBody(summarySheet.getRange(`A15:F${14 + summaryRows.length}`));
summarySheet.getRange(`B15:C${14 + summaryRows.length}`).format.numberFormat = "0.000";
summarySheet.getRange(`D15:E${14 + summaryRows.length}`).format.numberFormat = "0.0";
summarySheet.getRange("A1:F40").format.font.name = fontFamily;
summarySheet.getRange("A:A").format.columnWidth = 29;
summarySheet.getRange("B:B").format.columnWidth = 26;
summarySheet.getRange("C:F").format.columnWidth = 18;
summarySheet.freezePanes.freezeRows(14);

const currentChart = summarySheet.charts.add("line", summarySheet.getRange(`A14:C${14 + summaryRows.length}`));
currentChart.title = "Peak current by JRK target (A)";
currentChart.titleTextStyle.typeface = fontFamily;
currentChart.titleTextStyle.fontSize = 12;
currentChart.legend = { position: "top", textStyle: { typeface: fontFamily, fontSize: 9 } };
currentChart.xAxis = { axisType: "textAxis", textStyle: { typeface: fontFamily, fontSize: 8 } };
currentChart.yAxis = { numberFormatCode: "0.0", numberFormatSourceLinked: false, textStyle: { typeface: fontFamily, fontSize: 9 } };
currentChart.xAxis.title.text = "JRK target (lower is farther forward)";
currentChart.yAxis.title.text = "Current (A)";
currentChart.setPosition("H3", "P17");

const rateChart = summarySheet.charts.add("line", [
  summarySheet.getRange(`A14:A${14 + summaryRows.length}`),
  summarySheet.getRange(`D14:D${14 + summaryRows.length}`),
]);
rateChart.title = "Actuator movement rate by JRK target";
rateChart.titleTextStyle.typeface = fontFamily;
rateChart.titleTextStyle.fontSize = 12;
rateChart.hasLegend = false;
rateChart.xAxis = { axisType: "textAxis", textStyle: { typeface: fontFamily, fontSize: 8 } };
rateChart.yAxis = { numberFormatCode: "0", numberFormatSourceLinked: false, textStyle: { typeface: fontFamily, fontSize: 9 } };
rateChart.xAxis.title.text = "JRK target (lower is farther forward)";
rateChart.yAxis.title.text = "Feedback counts per second";
rateChart.setPosition("H19", "P33");

// Position map
mapSheet.showGridLines = false;
mapSheet.getRange("A1:I1").merge();
mapSheet.getRange("A1").values = [["Staged forward-position performance map"]];
styleTitle(mapSheet, mapSheet.getRange("A1"));
mapSheet.getRange("A2:I2").merge();
mapSheet.getRange("A2").values = [["Aggregated successful 40-count probe movements from guarded runs 5–7"]];
mapSheet.getRange("A2").format.font = { name: fontFamily, size: 10, italic: true, color: "#64748B" };
const mapColumns = [
  "target", "trials", "avg_final_feedback", "max_final_error_counts",
  "avg_duration_ms", "avg_rate_counts_per_s", "avg_peak_current_A",
  "max_peak_current_A", "avg_moving_current_A",
];
const mapHeaders = [
  "Target", "Trials", "Average final feedback", "Maximum final error",
  "Average duration (ms)", "Average rate (counts/s)", "Average peak (A)",
  "Maximum peak (A)", "Average moving current (A)",
];
mapSheet.getRange("A4:I4").values = [mapHeaders];
styleHeader(mapSheet.getRange("A4:I4"));
mapSheet.getRange(`A5:I${4 + positionMap.length}`).values = matrix(positionMap, mapColumns);
styleBody(mapSheet.getRange(`A5:I${4 + positionMap.length}`));
mapSheet.getRange(`C5:F${4 + positionMap.length}`).format.numberFormat = "0.0";
mapSheet.getRange(`G5:I${4 + positionMap.length}`).format.numberFormat = "0.000";
mapSheet.getRange("A:I").format.autofitColumns();
mapSheet.getRange("C:I").format.columnWidth = 20;
mapSheet.freezePanes.freezeRows(4);
mapSheet.tables.add(`A4:I${4 + positionMap.length}`, true, "PositionMapTable");

// Movement summary
movesSheet.showGridLines = false;
movesSheet.getRange("A1:Q1").merge();
movesSheet.getRange("A1").values = [["Guarded movement summary"]];
styleTitle(movesSheet, movesSheet.getRange("A1"));
const moveColumns = Object.keys(movements[0]);
const moveHeaders = moveColumns.map((name) => name.replaceAll("_", " "));
movesSheet.getRange(`A3:Q3`).values = [moveHeaders];
styleHeader(movesSheet.getRange("A3:Q3"));
movesSheet.getRangeByIndexes(3, 0, movements.length, moveColumns.length).values = matrix(movements, moveColumns);
styleBody(movesSheet.getRangeByIndexes(3, 0, movements.length, moveColumns.length));
movesSheet.getRange("A:Q").format.autofitColumns();
movesSheet.getRange("A:C").format.columnWidth = 18;
movesSheet.freezePanes.freezeRows(3);
movesSheet.tables.add(`A3:Q${3 + movements.length}`, true, "MovementTable");

// Ground-speed mapping template
speedSheet.showGridLines = false;
speedSheet.getRange("A1:G1").merge();
speedSheet.getRange("A1").values = [["JRK target to ground-speed map"]];
styleTitle(speedSheet, speedSheet.getRange("A1"));
speedSheet.getRange("A2:G2").merge();
speedSheet.getRange("A2").values = [["Enter GPS measurements from a controlled driving test. Actuator-only tests cannot determine tractor speed in m/s."]];
speedSheet.getRange("A2").format = { font: { name: fontFamily, size: 10, italic: true, color: "#64748B" }, wrapText: true };
const speedTargets = [2836, 2726, 2616, 2534, 2452, 2370, 2288, 2200, 2040, 1880];
speedSheet.getRange("A4:G4").values = [[
  "JRK target", "Measured JRK feedback", "Mean GPS speed (m/s)",
  "GPS speed standard deviation", "Samples", "Mean speed (mph)", "Notes",
]];
styleHeader(speedSheet.getRange("A4:G4"));
speedSheet.getRange(`A5:A${4 + speedTargets.length}`).values = speedTargets.map((value) => [value]);
speedSheet.getRange(`B5:E${4 + speedTargets.length}`).format.fill = "#FFF7D6";
speedSheet.getRange(`F5`).formulas = [[`=IF(C5="","",C5*2.23694)`]];
speedSheet.getRange(`F5:F${4 + speedTargets.length}`).fillDown();
styleBody(speedSheet.getRange(`A5:G${4 + speedTargets.length}`));
speedSheet.getRange(`C5:D${4 + speedTargets.length}`).format.numberFormat = "0.000";
speedSheet.getRange(`F5:F${4 + speedTargets.length}`).format.numberFormat = "0.00";
speedSheet.getRange("A:G").format.autofitColumns();
speedSheet.getRange("B:D").format.columnWidth = 24;
speedSheet.getRange("G:G").format.columnWidth = 32;
speedSheet.freezePanes.freezeRows(4);

// Raw guarded samples
samplesSheet.showGridLines = false;
samplesSheet.getRange("A1:I1").merge();
samplesSheet.getRange("A1").values = [["Guarded-test raw samples"]];
styleTitle(samplesSheet, samplesSheet.getRange("A1"));
const sampleColumns = ["run", "move_id", "t_ms", "phase", "target", "current_mA", "feedback", "peak_mA", "elapsed_ms"];
samplesSheet.getRange("A3:I3").values = [["Run", "Move ID", "Time (ms)", "Phase", "Target", "Current (mA)", "Feedback", "Peak (mA)", "Elapsed (ms)"]];
styleHeader(samplesSheet.getRange("A3:I3"));
samplesSheet.getRangeByIndexes(3, 0, guardSamples.length, sampleColumns.length).values = matrix(guardSamples, sampleColumns);
styleBody(samplesSheet.getRangeByIndexes(3, 0, guardSamples.length, sampleColumns.length));
samplesSheet.getRange("A:I").format.autofitColumns();
samplesSheet.getRange("A:B").format.columnWidth = 20;
samplesSheet.freezePanes.freezeRows(3);
samplesSheet.tables.add(`A3:I${3 + guardSamples.length}`, true, "GuardSamplesTable");

// Baseline samples from September 5
baselineSheet.showGridLines = false;
baselineSheet.getRange("A1:E1").merge();
baselineSheet.getRange("A1").values = [["September 5 baseline raw samples"]];
styleTitle(baselineSheet, baselineSheet.getRange("A1"));
const baselineColumns = ["t_ms", "step", "target", "current_mA", "feedback"];
baselineSheet.getRange("A3:E3").values = [["Time (ms)", "Step", "Target", "Current (mA)", "Feedback"]];
styleHeader(baselineSheet.getRange("A3:E3"));
baselineSheet.getRangeByIndexes(3, 0, baselineSamples.length, baselineColumns.length).values = matrix(baselineSamples, baselineColumns);
styleBody(baselineSheet.getRangeByIndexes(3, 0, baselineSamples.length, baselineColumns.length));
baselineSheet.getRange("A:E").format.autofitColumns();
baselineSheet.freezePanes.freezeRows(3);
baselineSheet.tables.add(`A3:E${3 + baselineSamples.length}`, true, "BaselineSamplesTable");

await fs.mkdir(outputDir, { recursive: true });
await fs.mkdir(previewDir, { recursive: true });
const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(outputPath);

const inspections = {};
inspections.summary = (await workbook.inspect({
  kind: "table", range: "Summary!A1:F40", include: "values,formulas",
  tableMaxRows: 40, tableMaxCols: 8,
})).ndjson;
inspections.errors = (await workbook.inspect({
  kind: "match", searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!",
  options: { useRegex: true, maxResults: 300 }, summary: "final formula error scan",
})).ndjson;
inspections.drawings = (await workbook.inspect({
  kind: "drawing", sheetId: "Summary", maxChars: 4000,
})).ndjson;
await fs.writeFile(path.join(outputDir, "verification.json"), JSON.stringify(inspections, null, 2));

const previewRanges = {
  "Summary": "A1:P40",
  "Position Map": `A1:I${4 + positionMap.length}`,
  "Movements": "A1:Q35",
  "Ground Speed Map": "A1:G14",
  "Guard Samples": "A1:I30",
  "Baseline Samples": "A1:E30",
};
for (const [sheetName, range] of Object.entries(previewRanges)) {
  const preview = await workbook.render({ sheetName, range, scale: 1, format: "png" });
  const safeName = sheetName.toLowerCase().replaceAll(" ", "_");
  await fs.writeFile(path.join(previewDir, `${safeName}.png`), new Uint8Array(await preview.arrayBuffer()));
}

console.log(JSON.stringify({ outputPath, sheets: 6, charts: 2, positionRows: positionMap.length, movementRows: movements.length }));
