# Diagnostic evidence: 62_Collins_polygon_1 / 20260727_144659

- Stripe samples: 26,585
- Stripe analyzed duration: 1331.0 s
- Stripe rows observed: 39

## Generated evidence

- `stripe_by_row.csv`
- `stripe_north_vs_south.csv`
- `stripe_by_geographic_quartile.csv`
- `stripe_by_direction.csv`
- `stripe_by_direction_and_geography.csv`
- `stripe_by_actual_speed.csv`
- `stripe_by_heading_error.csv`
- `stripe_row_performance.svg`
- `circling_window_1hz.csv`
- `circling_connector_audit.csv`
- `circling_event.svg`
- `diagnostic_summary.json`

## Perimeter candidate

- `perimeter_candidate/02_plan_settings_boundary_outer.json`
- `perimeter_candidate/62_Collins_polygon_1_boundary_outer_candidate_mission.txt`
- `perimeter_candidate/62_Collins_polygon_1_boundary_outer_candidate_mission_audit.csv`
- `perimeter_candidate/62_Collins_polygon_1_boundary_outer_candidate_preview.png`
- `perimeter_candidate/62_Collins_polygon_1_boundary_outer_candidate_validation.json`
- `perimeter_candidate/62_Collins_polygon_1_boundary_outer_candidate_validation.png`
- `perimeter_candidate/perimeter_geometry_comparison.json`
- `perimeter_candidate/perimeter_geometry_comparison.png`

Containment passed with no route outside the finalized boundary. The
independent validator status is `REVIEW` because sparse boundary vertices yield
a 0.46 m sampled-radius warning. Treat this as a supervised candidate, not
field approval.
