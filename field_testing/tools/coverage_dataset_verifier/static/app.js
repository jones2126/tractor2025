const REVIEW_STATUS = "REVIEW ONLY — NOT APPROVED FOR FIELD USE";
const PURPOSE_LABELS = {
  boundary_evidence: "Boundary evidence",
  obstacle_evidence: "Obstacle evidence",
  transition_evidence: "Transition evidence",
  coordinate_frame: "Coordinate frame",
  supporting_reference: "Supporting / reference",
};
const SOURCE_TYPE_LABELS = {
  raw_recording: "Raw recording",
  recorded_path_extract: "Extract from a recording",
  derived_review_artifact: "Derived review artifact",
  existing_inventory: "Existing inventory",
  supporting_reference: "Supporting reference",
};
const SOURCE_DECISION_LABELS = {
  needs_review: "Needs review",
  use_this_source: "Use this source",
  do_not_use: "Do not use",
  needs_replacement: "Needs replacement",
};
const LEGACY_ROLE_MAP = {
  boundary_evidence: ["boundary_evidence"],
  raw_gps_log: ["boundary_evidence", "obstacle_evidence"],
  obstacle_inventory: ["obstacle_evidence"],
  transition_path: ["transition_evidence"],
  supporting_evidence: ["supporting_reference"],
};
const COLORS = ["#1f6f50", "#cf6b20", "#7b4ca0", "#2474a8", "#c33d63", "#697a32", "#8a5b35", "#4f6875"];

let project = null;
let previewSourceId = null;
let previewPayload = null;
let highlightedFeatureId = null;
const inspection = new Map();
const $ = (selector) => document.querySelector(selector);

function uid() { return `SOURCE-${Date.now()}-${Math.floor(Math.random() * 1000)}`; }
function esc(value) { return String(value ?? "").replace(/[&<>'"]/g, (char) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;"}[char])); }
function formatBytes(bytes) {
  if (!Number.isFinite(bytes)) return "—";
  const units = ["B", "KB", "MB", "GB"];
  let value = bytes; let unit = 0;
  while (value >= 1024 && unit < units.length - 1) { value /= 1024; unit += 1; }
  return `${value.toFixed(unit ? 1 : 0)} ${units[unit]}`;
}
function formatRange(range, digits = 7) {
  if (!range) return "—";
  return range.map((value) => typeof value === "number" ? value.toFixed(digits) : value).join(" → ");
}

function normalizeProject(value) {
  value.status = REVIEW_STATUS;
  value.coordinateFrame ||= {};
  value.equipment ||= {};
  value.laterStageAssumptions ||= {};
  value.sourceApproval ||= {state: "needs_review", reviewer: "", reviewDate: "", comments: "", attestation: false};
  value.sources ||= [];
  for (const source of value.sources) {
    source.purposes = Array.isArray(source.purposes) ? source.purposes : (LEGACY_ROLE_MAP[source.role] || []);
    source.sourceType ||= source.role === "raw_gps_log" ? "raw_recording" : source.role === "transition_path" ? "recorded_path_extract" : "derived_review_artifact";
    source.sourceDecision ||= "needs_review";
    delete source.role;
  }
  return value;
}

function blankProject() {
  return normalizeProject({
    schemaVersion: 1,
    siteId: "",
    siteName: "",
    surveyNotes: "",
    outputDirectory: "field_testing/sites/NEW_SITE/planning/source_dataset_verification/NEW_SITE",
    equipment: {deckWidthM: 1.0668},
    laterStageAssumptions: {laneSpacingM: 0.9652, nominalOverlapM: 0.1016, weakerRightTurnRadiusM: 1.63, softTurnReviewRadiusM: 1.9, appliedToSourceGeometry: false},
    coordinateFrame: {sourceId: "", method: "Origin is the latitude/longitude in the first data row of the selected source file.", rowSelection: "first data row"},
    sources: [],
    sourceApproval: {state: "needs_review", reviewer: "", reviewDate: new Date().toISOString().slice(0, 10), comments: "", attestation: false},
  });
}

function readForm() {
  project.siteId = $("#site-id").value.trim();
  project.siteName = $("#site-name").value.trim();
  project.outputDirectory = $("#output-directory").value.trim();
  project.surveyNotes = $("#survey-notes").value.trim();
  project.coordinateFrame.sourceId = $("#frame-source").value;
  project.coordinateFrame.rowSelection = $("#frame-row").value.trim();
  project.coordinateFrame.method = $("#frame-method").value.trim();
  const frameInspection = inspection.get(project.coordinateFrame.sourceId);
  if (frameInspection?.firstCoordinate) project.coordinateFrame.resolvedOrigin = frameInspection.firstCoordinate;
  project.equipment.deckWidthM = Number($("#deck-width").value);
  project.laterStageAssumptions.laneSpacingM = Number($("#lane-spacing").value);
  project.laterStageAssumptions.nominalOverlapM = Number($("#overlap").value);
  project.laterStageAssumptions.weakerRightTurnRadiusM = Number($("#right-radius").value);
  project.laterStageAssumptions.softTurnReviewRadiusM = Number($("#soft-radius").value);
  project.laterStageAssumptions.appliedToSourceGeometry = false;
  project.sourceApproval.reviewer = $("#reviewer").value.trim();
  project.sourceApproval.reviewDate = $("#review-date").value;
  project.sourceApproval.comments = $("#approval-comments").value.trim();
  project.sourceApproval.attestation = $("#attestation").checked;
  project.status = REVIEW_STATUS;
  return project;
}

function fillForm() {
  normalizeProject(project);
  $("#site-id").value = project.siteId || "";
  $("#site-name").value = project.siteName || "";
  $("#output-directory").value = project.outputDirectory || "";
  $("#survey-notes").value = project.surveyNotes || "";
  $("#frame-row").value = project.coordinateFrame?.rowSelection || "";
  $("#frame-method").value = project.coordinateFrame?.method || "";
  $("#deck-width").value = project.equipment?.deckWidthM ?? "";
  $("#lane-spacing").value = project.laterStageAssumptions?.laneSpacingM ?? "";
  $("#overlap").value = project.laterStageAssumptions?.nominalOverlapM ?? "";
  $("#right-radius").value = project.laterStageAssumptions?.weakerRightTurnRadiusM ?? "";
  $("#soft-radius").value = project.laterStageAssumptions?.softTurnReviewRadiusM ?? "";
  $("#reviewer").value = project.sourceApproval?.reviewer || "";
  $("#review-date").value = project.sourceApproval?.reviewDate || new Date().toISOString().slice(0, 10);
  $("#approval-comments").value = project.sourceApproval?.comments || "";
  $("#attestation").checked = Boolean(project.sourceApproval?.attestation);
  renderPriorPackage(); renderSources(); renderFrameSources(); renderPreview(); updateGate();
}

function repoFileUrl(path) { return `/repo-file?path=${encodeURIComponent(path)}`; }

function renderPriorPackage() {
  const card = $("#prior-package");
  const prior = project.priorApprovedPerimeter;
  card.hidden = !prior;
  if (!prior) return;
  $("#prior-title").textContent = prior.title || "Prior perimeter package";
  $("#prior-scope").textContent = prior.approvalScope || "";
  $("#prior-execution").textContent = prior.executionStatus || "Execution status not recorded.";
  $("#prior-notes").textContent = prior.notes || "";
  $("#prior-waypoints").textContent = Number(prior.waypointCount || 0).toLocaleString();
  $("#prior-length").textContent = `${Number(prior.routeLengthM || 0).toFixed(3)} m`;
  $("#prior-speed").textContent = `${Number(prior.speedMps || 0).toFixed(2)} m/s`;
  $("#prior-hash").textContent = prior.missionSha256 || "Not recorded";
  $("#prior-play").href = repoFileUrl(prior.replayPath);
  $("#prior-validation").href = repoFileUrl(prior.validationPath);
  $("#prior-readme").href = repoFileUrl(prior.readmePath);
}

function renderFrameSources() {
  const selected = project.coordinateFrame?.sourceId || "";
  $("#frame-source").innerHTML = `<option value="">Choose a source…</option>` + project.sources.map((source) => `<option value="${esc(source.id)}">${esc(source.name)} (${esc(source.id)})</option>`).join("");
  $("#frame-source").value = selected;
  const origin = inspection.get(selected)?.firstCoordinate || project.coordinateFrame?.resolvedOrigin;
  $("#frame-origin").innerHTML = origin
    ? `<strong>Detected origin:</strong> ${Number(origin.latitude).toFixed(9)}, ${Number(origin.longitude).toFixed(9)} · data row ${esc(origin.dataRow)}${origin.time ? ` · ${esc(origin.time)}` : ""}`
    : "Verify the selected source to show the exact coordinate origin.";
}

function renderInspection(source) {
  const result = inspection.get(source.id);
  if (!result) return `<div class="inspection pending">Not verified in this session.</div>`;
  if (result.loading) return `<div class="inspection pending">Calculating hash and inspecting content…</div>`;
  const messages = [...(result.errors || []), ...(result.warnings || [])];
  return `<div class="inspection ${result.ok ? "pass" : "fail"}">
    <div class="inspection-grid">
      <div class="metric"><small>Status</small><strong>${result.ok ? "Passed" : "Failed"}</strong></div>
      <div class="metric"><small>Size</small><strong>${formatBytes(result.bytes)}</strong></div>
      <div class="metric"><small>${result.rowCount != null ? "Rows" : "Features"}</small><strong>${result.rowCount ?? result.featureCount ?? "—"}</strong></div>
      <div class="metric"><small>SHA-256</small><code title="${esc(result.sha256 || "")}">${esc(result.sha256 || "—")}</code></div>
      <div class="metric"><small>Latitude range</small><code>${esc(formatRange(result.latitudeRange))}</code></div>
      <div class="metric"><small>Longitude range</small><code>${esc(formatRange(result.longitudeRange))}</code></div>
      <div class="metric"><small>Time range</small><code>${esc(formatRange(result.timeRange, 0))}</code></div>
      <div class="metric"><small>Format</small><strong>${esc(result.format || "—")}</strong></div>
    </div>
    ${result.columns?.length ? `<div class="columns"><strong>Detected columns:</strong> ${esc(result.columns.join(", "))}</div>` : ""}
    ${result.geometryTypes?.length ? `<div class="columns"><strong>Geometry types:</strong> ${esc(result.geometryTypes.join(", "))}</div>` : ""}
    ${messages.length ? `<div class="columns"><strong>Notes:</strong> ${messages.map(esc).join(" · ")}</div>` : ""}
  </div>`;
}

function purposeOptions(source) {
  return Object.entries(PURPOSE_LABELS).map(([value, label]) => `<label class="purpose-option"><input type="checkbox" data-purpose="${value}" ${source.purposes.includes(value) ? "checked" : ""}>${label}</label>`).join("");
}

function renderSources() {
  $("#source-list").innerHTML = project.sources.map((source, index) => {
    const purposeBadges = source.purposes.map((purpose) => `<span class="badge">${esc(PURPOSE_LABELS[purpose] || purpose)}</span>`).join("");
    const typeClass = source.sourceType === "raw_recording" || source.sourceType === "recorded_path_extract" ? "raw" : "derived";
    return `<article class="source-card" data-source-index="${index}">
      <div class="source-top">
        <div class="source-title"><h3>${esc(source.name || source.id)}</h3><span class="badge ${typeClass}">${esc(SOURCE_TYPE_LABELS[source.sourceType] || source.sourceType)}</span>${purposeBadges}${source.required ? '<span class="badge required">Required</span>' : ""}</div>
        <div class="source-actions"><button class="small-button preview-source" type="button">View evidence</button><button class="small-button verify-source" type="button">Verify file</button><button class="small-button danger remove-source" type="button">Remove</button></div>
      </div>
      <div class="source-fields">
        <label>Stable source ID<input data-field="id" value="${esc(source.id)}"></label>
        <label>Source type<select data-field="sourceType">${Object.entries(SOURCE_TYPE_LABELS).map(([value, label]) => `<option value="${value}" ${source.sourceType === value ? "selected" : ""}>${label}</option>`).join("")}</select></label>
        <fieldset class="purpose-field"><legend>Used for</legend><div class="purpose-options">${purposeOptions(source)}</div></fieldset>
        <label class="path">Name<input data-field="name" value="${esc(source.name)}"></label>
        <label class="path">Repository-relative file path<input data-field="path" value="${esc(source.path)}"></label>
        <label class="hash">Expected SHA-256 <span class="hint">leave blank to establish a new baseline</span><input data-field="expectedSha256" value="${esc(source.expectedSha256 || "")}"></label>
        <label class="notes">Notes<textarea data-field="notes" rows="2">${esc(source.notes || "")}</textarea></label>
        <label>Source decision<select data-field="sourceDecision">${Object.entries(SOURCE_DECISION_LABELS).map(([value, label]) => `<option value="${value}" ${source.sourceDecision === value ? "selected" : ""}>${label}</option>`).join("")}</select></label>
        <label><span>Required for gate</span><select data-field="required"><option value="true" ${source.required ? "selected" : ""}>Yes</option><option value="false" ${!source.required ? "selected" : ""}>No</option></select></label>
      </div>${renderInspection(source)}
    </article>`;
  }).join("");
}

function sourceFromCard(card) { return project.sources[Number(card.dataset.sourceIndex)]; }
function updateSourceFromInput(input) {
  const card = input.closest(".source-card"); if (!card) return;
  const source = sourceFromCard(card); const oldId = source.id;
  source[input.dataset.field] = input.dataset.field === "required" ? input.value === "true" : input.value;
  if (["path", "expectedSha256", "id"].includes(input.dataset.field)) inspection.delete(oldId);
  if (input.dataset.field === "id" && project.coordinateFrame.sourceId === oldId) project.coordinateFrame.sourceId = source.id;
  renderFrameSources(); updateGate();
}

async function verifySource(source, render = true) {
  inspection.set(source.id, {loading: true}); if (render) renderSources();
  try {
    const response = await fetch("/api/inspect", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({path: source.path, expectedSha256: source.expectedSha256 || ""})});
    const result = await response.json(); inspection.set(source.id, result);
    if (result.ok && !source.expectedSha256) source.expectedSha256 = result.sha256;
  } catch (error) { inspection.set(source.id, {ok: false, errors: [`Could not contact local verifier: ${error.message}`]}); }
  if (render) { renderSources(); renderFrameSources(); updateGate(); }
}
async function verifyAll() {
  readForm(); setMessage("Verifying required files… the large GPS log may take a moment."); $("#verify-all").disabled = true;
  for (const source of project.sources.filter((item) => item.required)) await verifySource(source, false);
  renderSources(); renderFrameSources(); updateGate(); $("#verify-all").disabled = false;
  const nonHumanBlockers = gateBlockers().filter((item) => !item.startsWith("Enter the reviewer") && !item.startsWith("Check the source"));
  setMessage(nonHumanBlockers.length ? "Verification finished. Review the listed file or setup issues." : "All required files passed mechanical verification. Visual evidence and human approval are still required.", nonHumanBlockers.length ? "error" : "success");
}

function gateBlockers() {
  readForm(); const blockers = [];
  if (!project.siteId) blockers.push("Enter a site ID.");
  if (!project.siteName) blockers.push("Enter a friendly site name.");
  if (!project.outputDirectory) blockers.push("Enter an output directory.");
  if (!(project.equipment.deckWidthM > 0)) blockers.push("Enter a deck width greater than zero.");
  const purposes = new Set(project.sources.filter((source) => source.required).flatMap((source) => source.purposes));
  const sourceIds = project.sources.map((source) => source.id.trim()).filter(Boolean);
  if (sourceIds.length !== new Set(sourceIds).size) blockers.push("Give every source a unique stable ID.");
  if (project.sources.some((source) => !source.id.trim())) blockers.push("Give every source a stable ID.");
  for (const purpose of ["boundary_evidence", "obstacle_evidence", "transition_evidence"]) if (!purposes.has(purpose)) blockers.push(`Add a required source used for ${PURPOSE_LABELS[purpose].toLowerCase()}.`);
  if (!project.sources.some((source) => source.required && source.sourceType === "raw_recording")) blockers.push("Add at least one required raw recording.");
  for (const source of project.sources.filter((item) => item.required)) {
    const result = inspection.get(source.id);
    if (!result || result.loading) blockers.push(`Mechanically verify ${source.name || source.id}.`);
    else if (!result.ok) blockers.push(`Resolve verification failure for ${source.name || source.id}.`);
    if (source.sourceDecision !== "use_this_source") blockers.push(`Review the evidence and decide to use or replace ${source.name || source.id}.`);
  }
  const frame = project.sources.find((source) => source.id === project.coordinateFrame.sourceId);
  if (!frame) blockers.push("Choose a coordinate-frame source.");
  else if (!inspection.get(frame.id)?.ok) blockers.push("Mechanically verify the coordinate-frame source.");
  if (!project.sourceApproval.reviewer) blockers.push("Enter the reviewer name.");
  if (!project.sourceApproval.reviewDate) blockers.push("Enter the review date.");
  if (!project.sourceApproval.attestation) blockers.push("Check the source-verification attestation.");
  return [...new Set(blockers)];
}
function updateGate() {
  if (!project) return;
  const required = project.sources.filter((source) => source.required);
  const passed = required.filter((source) => inspection.get(source.id)?.ok).length;
  $("#source-progress").textContent = `${passed}/${required.length} mechanically verified`;
  const blockers = gateBlockers();
  $("#blockers").className = blockers.length ? "blockers" : "blockers ready";
  $("#blockers").innerHTML = blockers.length ? `<strong>Approval is blocked by:</strong><ul>${blockers.map((item) => `<li>${esc(item)}</li>`).join("")}</ul>` : "Ready to approve the identified source inputs. Geometry remains unapproved.";
  $("#approve-sources").disabled = blockers.length > 0;
  $("#gate-status").className = blockers.length ? "gate-status" : "gate-status complete";
  $("#gate-status").innerHTML = blockers.length ? '<span class="status-dot"></span><strong>Gate blocked</strong><small>Finish the checklist below</small>' : '<span class="status-dot"></span><strong>Ready for source approval</strong><small>Geometry remains unapproved</small>';
}

function geometryPoints(geometry) {
  const points = [];
  function visit(value) {
    if (!Array.isArray(value)) return;
    if (value.length >= 2 && Number.isFinite(Number(value[0])) && Number.isFinite(Number(value[1]))) { points.push([Number(value[0]), Number(value[1])]); return; }
    value.forEach(visit);
  }
  visit(geometry?.coordinates); return points;
}
function drawGeometry(ctx, geometry, projectPoint, color, emphasis) {
  const type = geometry?.type; const coordinates = geometry?.coordinates; const lineWidth = emphasis ? 4 : 2;
  function path(points, close = false, fill = false) {
    if (!points?.length) return; ctx.beginPath(); points.forEach((point, index) => { const [x, y] = projectPoint(point); index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    if (close) ctx.closePath(); if (fill) { ctx.fillStyle = `${color}28`; ctx.fill(); } ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.stroke();
  }
  if (type === "Point") { const [x, y] = projectPoint(coordinates); ctx.beginPath(); ctx.arc(x, y, emphasis ? 7 : 5, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill(); }
  if (type === "MultiPoint") coordinates.forEach((point) => drawGeometry(ctx, {type: "Point", coordinates: point}, projectPoint, color, emphasis));
  if (type === "LineString") path(coordinates);
  if (type === "MultiLineString") coordinates.forEach((line) => path(line));
  if (type === "Polygon") coordinates.forEach((ring, index) => path(ring, true, index === 0));
  if (type === "MultiPolygon") coordinates.forEach((polygon) => polygon.forEach((ring, index) => path(ring, true, index === 0)));
}
function drawPreviewMap() {
  if (!previewPayload) return;
  const canvas = $("#evidence-map"); const rect = canvas.getBoundingClientRect(); const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, rect.width * dpr); canvas.height = Math.max(1, rect.height * dpr);
  const ctx = canvas.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, rect.width, rect.height); ctx.fillStyle = "#f8faf8"; ctx.fillRect(0, 0, rect.width, rect.height);
  const points = previewPayload.features.flatMap((feature) => geometryPoints(feature.geometry));
  if (!points.length) { ctx.fillStyle = "#657168"; ctx.fillText("No drawable coordinates detected.", 20, 30); return; }
  const xs = points.map((point) => point[0]); const ys = points.map((point) => point[1]);
  let minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  if (minX === maxX) { minX -= 1; maxX += 1; } if (minY === maxY) { minY -= 1; maxY += 1; }
  const pad = 28; const scale = Math.min((rect.width - pad * 2) / (maxX - minX), (rect.height - pad * 2) / (maxY - minY));
  const projectPoint = ([x, y]) => [pad + (x - minX) * scale, rect.height - pad - (y - minY) * scale];
  previewPayload.features.forEach((feature, index) => drawGeometry(ctx, feature.geometry, projectPoint, COLORS[index % COLORS.length], feature.id === highlightedFeatureId));
}

function renderPreview() {
  const source = project?.sources.find((item) => item.id === previewSourceId);
  $("#preview-empty").hidden = Boolean(source); $("#preview-workspace").hidden = !source; if (!source) return;
  $("#preview-title").textContent = source.name; $("#preview-type").textContent = SOURCE_TYPE_LABELS[source.sourceType] || source.sourceType; $("#preview-path").textContent = source.path;
  const derived = source.sourceType === "derived_review_artifact" || source.sourceType === "existing_inventory";
  $("#lineage-warning").innerHTML = derived ? "<strong>Derived evidence:</strong> this file was produced from other records. It should not be trusted merely because it exists or passes a hash check." : "<strong>Recorded evidence:</strong> this file contains captured or extracted field data. Its interpretation still requires human review.";
  const provenance = source.provenance || {};
  $("#lineage-summary").textContent = provenance.summary || source.notes || "No additional lineage summary has been recorded yet.";
  $("#lineage-generator").textContent = provenance.generatedBy || (source.sourceType === "raw_recording" ? "Direct field recording; no geometry generator." : "Not yet recorded.");
  $("#lineage-inputs").innerHTML = (provenance.sourceInputs || []).length ? provenance.sourceInputs.map((item) => `<li>${esc(item)}</li>`).join("") : "<li>No parent inputs recorded.</li>";
  const legacyLink = $("#legacy-viewer-link");
  if (source.legacyViewerPath) { legacyLink.hidden = false; legacyLink.href = `/repo-file?path=${encodeURIComponent(source.legacyViewerPath)}`; } else { legacyLink.hidden = true; legacyLink.removeAttribute("href"); }
  if (!previewPayload) { $("#feature-list").innerHTML = '<div class="inspection pending">Loading geometry preview…</div>'; return; }
  $("#feature-list").innerHTML = previewPayload.features.map((feature, index) => `<button class="feature-item ${feature.id === highlightedFeatureId ? "active" : ""}" data-feature-id="${esc(feature.id)}" style="--feature-color:${COLORS[index % COLORS.length]}"><span class="feature-swatch"></span><span><strong>${esc(feature.name)}</strong><small>${esc(feature.kind)} · ${esc(feature.geometry?.type || "unknown geometry")}</small></span></button>`).join("") || "No drawable features detected.";
  drawPreviewMap();
}
async function loadPreview(source) {
  previewSourceId = source.id; previewPayload = null; highlightedFeatureId = null; renderPreview(); $("#preview-workspace").scrollIntoView({behavior: "smooth", block: "start"});
  try { const response = await fetch("/api/preview", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({path: source.path})}); const result = await response.json(); if (!response.ok || !result.ok) throw new Error((result.errors || ["Preview failed."]).join(" ")); previewPayload = result; renderPreview(); }
  catch (error) { $("#feature-list").innerHTML = `<div class="inspection fail">${esc(error.message)}</div>`; }
}

function setMessage(text, kind = "") { $("#message").className = `message ${kind}`; $("#message").textContent = text; }
async function save(approve) {
  readForm(); setMessage(approve ? "Rechecking files and recording source approval…" : "Saving a review draft…");
  try { const response = await fetch(approve ? "/api/approve-sources" : "/api/save-draft", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(project)}); const result = await response.json(); if (!result.ok) throw new Error((result.errors || ["Save failed."]).join(" ")); project = normalizeProject(result.project); setMessage(`${approve ? "Tab 1 source list approved" : "Draft saved"}. Project: ${result.projectPath}. Audit ledger: ${result.ledgerPath}. Geometry remains unapproved.`, "success"); }
  catch (error) { setMessage(error.message, "error"); }
}
function downloadProject() { readForm(); const blob = new Blob([JSON.stringify(project, null, 2) + "\n"], {type: "application/json"}); const link = document.createElement("a"); link.href = URL.createObjectURL(blob); link.download = `${project.siteId || "site"}-source-project-REVIEW_ONLY.json`; link.click(); URL.revokeObjectURL(link.href); }
async function importProject(file) {
  try { const imported = JSON.parse(await file.text()); if (!Array.isArray(imported.sources)) throw new Error("The selected JSON has no source register."); project = normalizeProject(imported); inspection.clear(); previewSourceId = null; previewPayload = null; fillForm(); setMessage("Working copy imported. Verify and view its evidence before approval.", "success"); }
  catch (error) { setMessage(`Import failed: ${error.message}`, "error"); }
}

document.addEventListener("input", (event) => { if (event.target.matches("[data-field]")) updateSourceFromInput(event.target); else updateGate(); });
document.addEventListener("change", (event) => {
  if (event.target.matches("[data-field]")) updateSourceFromInput(event.target);
  else if (event.target.matches("[data-purpose]")) { const source = sourceFromCard(event.target.closest(".source-card")); const purpose = event.target.dataset.purpose; source.purposes = event.target.checked ? [...new Set([...source.purposes, purpose])] : source.purposes.filter((item) => item !== purpose); renderSources(); updateGate(); }
  else updateGate();
});
$("#source-list").addEventListener("click", async (event) => {
  const card = event.target.closest(".source-card"); if (!card) return; const source = sourceFromCard(card);
  if (event.target.closest(".preview-source")) await loadPreview(source);
  if (event.target.closest(".verify-source")) { readForm(); await verifySource(source); }
  if (event.target.closest(".remove-source")) { inspection.delete(source.id); project.sources.splice(Number(card.dataset.sourceIndex), 1); if (previewSourceId === source.id) { previewSourceId = null; previewPayload = null; } renderSources(); renderFrameSources(); renderPreview(); updateGate(); }
});
$("#feature-list").addEventListener("click", (event) => { const item = event.target.closest("[data-feature-id]"); if (!item) return; highlightedFeatureId = item.dataset.featureId; renderPreview(); });
$("#add-source").addEventListener("click", () => { project.sources.push({id: uid(), name: "New source", sourceType: "supporting_reference", purposes: ["supporting_reference"], sourceDecision: "needs_review", required: false, path: "", expectedSha256: "", notes: ""}); renderSources(); renderFrameSources(); updateGate(); });
$("#new-project").addEventListener("click", () => { if (!window.confirm("Start a blank site project? Unsaved changes on this page will be discarded.")) return; project = blankProject(); inspection.clear(); previewSourceId = null; previewPayload = null; fillForm(); setMessage("Blank site project created. Add only this yard's own source files.", "success"); });
$("#verify-all").addEventListener("click", verifyAll);
$("#save-draft").addEventListener("click", () => save(false));
$("#approve-sources").addEventListener("click", () => save(true));
$("#export-project").addEventListener("click", downloadProject);
$("#import-project").addEventListener("click", () => $("#project-file").click());
$("#project-file").addEventListener("change", (event) => event.target.files[0] && importProject(event.target.files[0]));
window.addEventListener("resize", () => previewPayload && drawPreviewMap());

fetch("/api/default-project").then((response) => response.json()).then((data) => { project = normalizeProject(data); fillForm(); }).catch((error) => { document.body.innerHTML = `<p>Could not load the local tool: ${esc(error.message)}</p>`; });
