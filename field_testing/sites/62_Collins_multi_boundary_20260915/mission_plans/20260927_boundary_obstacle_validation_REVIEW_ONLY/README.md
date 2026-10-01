# 62 Collins boundary and obstacle validation paths — review only

This package is the first planning step toward a supervised, blades-off field validation. It is deliberately not runnable.

- Tractor-center paths use the 0.5334 m deck half-width plus a 0.3048 m (12-inch) first-run margin.
- Future nominal test settings are 0.35 m/s and 1.00 m lookahead; they are not yet authorized.
- Solid paths validate proposed outer boundaries; dashed paths validate obstacle clearances.
- Green paths pass the initial geometry-only audit. Red paths conflict with an edge or another obstacle and must be resolved.
- No transitions, mission text file, dashboard, or launcher have been generated.
- After owner review, the next build should select route direction/start points, use only approved paths, and add demonstrated connectors.

Review the PNG, GeoJSON, and audit CSV before authorizing a field build.
