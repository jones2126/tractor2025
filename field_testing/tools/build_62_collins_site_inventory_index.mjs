import fs from "node:fs/promises";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const csvPath = "field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.csv";
const manifestPath = "field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory_manifest.json";
const outputPath = "field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.xlsx";
const historyPath = "field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_002_20260925/62_Collins_site_inventory.xlsx";
const previewPath = "field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory_index_preview.png";

const csvText = (await fs.readFile(csvPath, "utf8")).replace(/^\uFEFF/, "");
const manifest = JSON.parse(await fs.readFile(manifestPath, "utf8"));
const workbook = await Workbook.fromCSV(csvText, { sheetName: "Inventory" });
const sheet = workbook.worksheets.getItem("Inventory");
const used = sheet.getUsedRange();
const values = used.values;

for (let row = 1; row < values.length; row += 1) {
  values[row][0] = Number(values[row][0]);
  for (const col of [9, 10, 11, 12]) {
    if (values[row][col] !== "") values[row][col] = Number(values[row][col]);
  }
  if (values[row][16] === "True") values[row][16] = true;
  if (values[row][16] === "False") values[row][16] = false;
}
used.values = values;

sheet.showGridLines = false;
sheet.freezePanes.freezeRows(1);
sheet.freezePanes.freezeColumns(2);
used.format.font = { name: "Arial", size: 10, color: "#1F2937" };
sheet.getRange("A1:V1").format = {
  fill: "#1F4E78",
  font: { name: "Arial", size: 10, bold: true, color: "#FFFFFF" },
  horizontalAlignment: "center",
  verticalAlignment: "center",
  wrapText: true,
  rowHeight: 34,
};
sheet.getRange(`A2:V${values.length}`).format.verticalAlignment = "top";
sheet.getRange(`F2:F${values.length}`).format.wrapText = true;
sheet.getRange(`G2:G${values.length}`).format.wrapText = true;
sheet.getRange(`U2:V${values.length}`).format.wrapText = true;
sheet.getRange(`J2:M${values.length}`).format.numberFormat = "0.000000";

const widths = [10, 31, 34, 18, 31, 35, 34, 13, 17, 14, 14, 12, 12, 25, 25, 22, 13, 28, 30, 30, 48, 65];
for (let col = 0; col < widths.length; col += 1) {
  sheet.getRangeByIndexes(0, col, values.length, 1).format.columnWidth = widths[col];
}
sheet.getRange(`A2:V${values.length}`).format.rowHeight = 46;

const table = sheet.tables.add(`A1:V${values.length}`, true, "SiteInventoryTable");
table.style = "TableStyleMedium2";
table.showBandedColumns = false;
table.showFilterButton = true;

const summary = workbook.worksheets.add("Summary");
summary.showGridLines = false;
summary.getRange("A1:D1").merge();
summary.getRange("A1").values = [["62 Collins site inventory"]];
summary.getRange("A1:D1").format = { font: { name: "Arial", size: 16, bold: true, color: "#1F2937" }, rowHeight: 28 };
summary.getRange("A2:D2").merge();
summary.getRange("A2").values = [["Revision 2 · reviewed 2026-09-25 · no launchable mission"]];
summary.getRange("A2:D2").format = { font: { name: "Arial", size: 10, italic: true, color: "#5F6B76" }, rowHeight: 22 };
summary.getRange("A4:B11").values = [
  ["Item", "Value"],
  ["Mowable areas", manifest.counts.mowable_areas],
  ["Obstacles", manifest.counts.obstacles],
  ["Entry/exit access points", manifest.counts.access_points],
  ["Site reference points", manifest.counts.site_reference_points],
  ["Between-area routes", manifest.counts.transition_routes],
  ["Total mowable area (m²)", manifest.mowable_area_total_m2],
  ["Between-area route length (m)", manifest.transition_route_total_length_m],
];
summary.getRange("A4:B4").format = { fill: "#1F4E78", font: { name: "Arial", bold: true, color: "#FFFFFF" } };
summary.getRange("A5:B11").format.font = { name: "Arial", size: 10, color: "#1F2937" };
summary.getRange("B10:B11").format.numberFormat = "0.000";
summary.getRange("A13:D17").values = [
  ["Status", "Meaning", "Applies to", "Next step"],
  ["reviewed_not_field_validated", "Geometry accepted for planning", "Areas and obstacle exclusions", "Validate during the next field run"],
  ["reviewed_not_field_validated", "Boundary crossing derived from recorded travel", "Entry/exit access points", "Confirm during the next field run"],
  ["approved_existing_between_areas", "Recorded travel retained outside mowing areas", "Between-area routes", "Use only in the recorded direction"],
  ["field_recorded", "Direct GNSS point evidence", "Original coverage start", "Keep as a reference point"],
];
summary.getRange("A13:D13").format = { fill: "#1F4E78", font: { name: "Arial", bold: true, color: "#FFFFFF" }, wrapText: true };
summary.getRange("A14:D17").format = { font: { name: "Arial", size: 10, color: "#1F2937" }, wrapText: true, verticalAlignment: "top" };
summary.getRange("A19:D21").values = [
  ["Operating rule", "Detail", "", ""],
  ["Obstacle adjustments", "Tree and telephone-pole mowing exclusions already include their documented adjustments. Do not apply them twice.", "", ""],
  ["Transition inventory", "Internal garden travel is not inventoried. Coverage planners connect to the five between-area routes through the named access points.", "", ""],
];
summary.getRange("A19:D19").format = { fill: "#D9EAF7", font: { name: "Arial", bold: true, color: "#1F2937" } };
summary.getRange("B20:D20").merge();
summary.getRange("B21:D21").merge();
summary.getRange("A20:D21").format = { font: { name: "Arial", size: 10, color: "#1F2937" }, wrapText: true, verticalAlignment: "top" };
summary.getRange("A1:D21").format.font.name = "Arial";
summary.getRange("A:A").format.columnWidth = 28;
summary.getRange("B:B").format.columnWidth = 62;
summary.getRange("C:D").format.columnWidth = 26;
summary.getRange("A14:D17").format.rowHeight = 34;
summary.getRange("A20:D21").format.rowHeight = 45;

const check = await workbook.inspect({
  kind: "table",
  range: "Summary!A1:D21",
  include: "values,formulas",
  tableMaxRows: 20,
  tableMaxCols: 6,
});
console.log(check.ndjson);
const inventoryCheck = await workbook.inspect({
  kind: "table",
  range: `Inventory!A1:V${values.length}`,
  include: "values,formulas",
  tableMaxRows: 7,
  tableMaxCols: 22,
});
console.log(inventoryCheck.ndjson);
const errors = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!",
  options: { useRegex: true, maxResults: 300 },
  summary: "final formula error scan",
});
console.log(errors.ndjson);

const preview = await workbook.render({ sheetName: "Summary", range: "A1:D21", scale: 1.5, format: "png" });
await fs.writeFile(previewPath, new Uint8Array(await preview.arrayBuffer()));
const output = await SpreadsheetFile.exportXlsx(workbook);
await output.save(outputPath);
await fs.copyFile(outputPath, historyPath);
