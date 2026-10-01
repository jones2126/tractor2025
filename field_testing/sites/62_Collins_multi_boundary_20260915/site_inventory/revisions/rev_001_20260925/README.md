# 62 Collins site inventory

Revision 1 is the durable site description for coverage planning. It contains three mowable areas, two obstacles, and eight approved existing transition paths. It does not contain a launchable mission.

## Current files

- `62_Collins_site_inventory.geojson` is the authoritative machine-readable geometry.
- `62_Collins_site_inventory.csv` is the human-readable asset index.
- `62_Collins_site_inventory.xlsx` is a formatted copy of the same index.
- `62_Collins_site_inventory_map.html` is the interactive review map.
- `62_Collins_site_inventory_map.png` is the static review map.
- `62_Collins_site_inventory_manifest.json` records counts, source hashes, and inventory rules.

- `62_Collins_site_inventory_validation.json` records the geometry and count checks.

## Geometry rules

Mowable polygons describe the permitted deck coverage. Obstacle features separately retain recorded evidence, tested tractor-center limits, and mowing exclusions. Transition paths are recorded tractor centerlines and are approved only in their original direction.

The tree mowing exclusion already includes the 0.5334 m deck-edge derivation. The telephone-pole exclusion already includes 0.6096 m clearance and a 0.02 m numerical margin. Do not add either adjustment a second time.

## Status

Revision 1 was reviewed on 2026-09-25. New mowing boundaries and obstacle exclusions remain `reviewed_not_field_validated` until the next field run. Existing transitions are stored as `approved_existing` and remain direction-specific.

## Revision history

The immutable revision snapshot is in `revisions/rev_001_20260925/`.
