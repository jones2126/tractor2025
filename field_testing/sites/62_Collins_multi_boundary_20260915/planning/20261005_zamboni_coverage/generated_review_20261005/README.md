# 62 Collins Zamboni coverage — review only

**REVIEW ONLY — NOT APPROVED FOR FIELD USE**

This package sequences the three mowing sections as backyard/gardens, front
yard, and over-road. Each section begins with three boundary/headland passes,
then north-south stripes. Ordinary reversals span four columns (about 3.86 m), supporting a 1.90 m soft
turn; all four column groups are completed later. Recorded
between-area routes connect the sections.

- Active obstacles: 15 (`O012` / `OBS-NEW-012` removed)
- Deck: 1.0668 m (42 in)
- Stripe spacing: 0.9652 m (38 in)
- Nominal overlap: 0.1016 m (4 in)
- Target turn radius: 1.90 m
- Evidence-based fallback radius: 1.63 m
- Additional review margin outside obstacle exclusions: 0.15 m
- Planned route length: 8095.1 m
- Estimated time at 1.00 m/s: 134.9 minutes
- Planned deck coverage: 97.10%
- Uncovered review area: 135.26 m²
- Radius-validated soft turns: 90
- Contained boundary relocations requiring curvature smoothing: 42
- Uncontained or missing connections: 1

## Review files

- `62_Collins_zamboni_coverage_REVIEW_ONLY.png` — full-site and per-section static review.
- `62_Collins_zamboni_coverage_INTERACTIVE_REVIEW_ONLY.html` — replay, layer controls, section filter, and deck footprint.
- `coverage_validation_report.json` — coverage, clearance, and turn validation.
- `stripe_order.csv` — exact Zamboni stripe order.
- `turn_report.csv` — every connector and its radius/containment result.
- `review_waypoint_audit.csv` — dense review points and lineage; not a mission file.
- `62_Collins_zamboni_coverage_REVIEW_ONLY.geojson` — reconciled review geometry.
- `input_manifest.json` — source hashes and planning rules.

No launcher, approval flag, or executable five-column mission is present.
Red dashed connectors follow contained review routes where possible, but still
require curvature smoothing. Any uncontained connector is a hard review blocker.
Neither category is approved for a later field mission.
