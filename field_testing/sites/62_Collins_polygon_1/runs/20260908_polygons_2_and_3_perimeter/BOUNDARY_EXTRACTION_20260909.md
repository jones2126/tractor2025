# Polygon 2 and 3 boundary extraction — 2026-09-09

## Source

The handheld perimeter session was recorded as one continuous field log:

`62_collins_polygon_2_20260908.csv`

Despite its original filename, the log contains the transition to Polygon 2,
the Polygon 2 boundary, the transition to Polygon 3, the Polygon 3 boundary,
and the return to the parking location. The original transferred file SHA-256
is:

`e8abf55919e3072c01453bc9456c3d204c2f150c8842091262d06a1176b990f9`

Git stores CSV files with LF line endings. The repository-normalized SHA-256
is:

`6af981e96a6d28e316656c8098549a81994178a5dbd74adad69b5e9283f0cf59`

Six deliberate Pause sections separate the five driving sections.

## Polygon 2

- Retained elapsed time: 120.76–207.64 seconds.
- Source data rows: 2416–4154 after the CSV header.
- The first 3.98 seconds of the original candidate loop were removed to drop
  the initial approach leg.
- All 1,739 accepted source rows were RTK Fixed.
- Candidate thinning: 0.50 m minimum spacing, producing 93 points.
- Final simplification: 0.10 m, producing 20 vertices.
- Final area: approximately 115.7 m².
- Final perimeter: approximately 48.6 m.
- Implicit closing edge: 0.26 m.
- Final CSV SHA-256:
  `e277376d764395f76940c80dba328d8329cba7b5b6c24594d55154cde512a24f`.

The planner-ready boundary is:

`field_testing/sites/62_Collins_polygon_2/01_boundary_final.csv`

## Polygon 3

- Retained elapsed time: 325.37–456.30 seconds.
- Source data rows: 6508–9127 after the CSV header.
- The first 0.97 seconds of the original candidate loop were removed.
- 2,442 RTK Fixed source rows were accepted; 178 RTK Float rows were excluded.
- Candidate thinning: 0.50 m minimum spacing, producing 110 points.
- Final simplification: 0.10 m, producing 26 vertices.
- Final area: approximately 190.0 m².
- Final perimeter: approximately 61.6 m.
- Implicit closing edge: 0.83 m.
- Final CSV SHA-256:
  `729bdf4d988289482408cb569d4e8b9ae3c695c34835250b7f07adfa3b3fe0ce`.

The planner-ready boundary is:

`field_testing/sites/62_Collins_polygon_3/01_boundary_final.csv`

## Validation

Both final CSV files use the project-standard boundary columns:

`sequence,lat,lon,east_m,north_m,source_sequence,notes`

Both reload through `site_planner_common_20260724.load_boundary`, have
continuous one-based sequence numbers, and form valid simple polygons. The
format closes the polygon implicitly from the last vertex back to the first;
the first vertex is not duplicated as the final CSV row.
