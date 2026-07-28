# July 27, 2026 field-test findings

Site: `62_Collins_polygon_1`
Run: `20260727_144659`

No source log, original mission, controller, or planner file was changed to
produce these findings.

## Straight stripes

The northern rows are objectively worse. Across the four geographic
quartiles from south to north, controller mean absolute CTE was 0.132, 0.271,
0.439, and 0.656 m. Direction-aware geometric line error was 0.073, 0.153,
0.301, and 0.421 m. The row-mean geometric-error trend was 0.0131 m of added
error per metre northward with R² = 0.885.

The result remains after separating opposite travel directions. In the north
quartile, eastbound and westbound controller MAE was 0.664 and 0.646 m;
geometric MAE was 0.424 and 0.417 m. In the south quartile, the corresponding
values were 0.133/0.131 m and 0.079/0.067 m.

Travel direction strongly affected actual speed but only weakly affected CTE.
At the same 0.75 m/s command, eastbound travel averaged 0.607 m/s and
westbound travel averaged 0.726 m/s. Their overall controller MAE was
0.361/0.377 m and geometric MAE was 0.234/0.229 m. This supports an
incline/load effect on propulsion, but slower travel direction by itself does
not explain the northern tracking degradation.

Signed error is systematic. In the north quartile, controller `yt` averaged
-0.598 m: the selected target was to the tractor's right. Direction-aware
geometric error averaged +0.388 m: the tractor was left of the intended travel
line. Pure Pursuit responded with a mean normalized right-steer command of
-0.268. This is consistent correction sense, not an absolute-value artifact.

Pure Pursuit lookahead inflates controller CTE when heading alignment is poor.
For samples above 10 degrees absolute heading error, controller MAE was
1.097 m while geometric line MAE was 0.302 m. The independent geometric metric
still confirms real northern displacement.

Steering saturation was not the primary limitation. Across all stripe samples,
normalized Pure Pursuit command saturation occurred for 0.13% of time and
steering PWM saturation occurred for 0%. The north quartile still had 0% PWM
saturation. Steering-feedback error did rise in parts of the north, especially
on westbound rows, so P-only steady-state error, deadband/backlash, mechanical
bias, traction, or lateral terrain load remain plausible contributors. These
logs do not identify one of those as the sole cause.

## Final circling event

Pursuit index 4033 is the zero-based index of audit waypoint 4034. It is a
connector point on a planned 1.9 m-radius keyhole. The controller's configured
minimum turn radius is approximately 1.768 m, so the planned local curvature
was feasible.

At elapsed 3157.244 s, index 4033 was initially selected 1.872 m away and
0.863 m ahead in tractor coordinates. It was not initially selected behind the
tractor. The command was already saturated full right because the target was
1.661 m to the right.

The advancement loop starts its search at the existing index and selects that
same waypoint whenever its distance is greater than its 1.5 m lookahead. The
minimum distance reached during the entire AUTO lock was 1.502686 m. Missing
the advancement threshold by 2.7 mm caused the same fixed target to be selected
indefinitely.

The AUTO lock lasted 136.982 s, covered 62.50 m, and accumulated 1428.4 degrees
of heading change (about 3.97 rotations). During elapsed 3180–3230 s, a later
connector waypoint was closer than index 4033 for 53.7% of samples, but the
controller did not consider it because the current index never advanced.

The root cause is brittle waypoint advancement and stale-target lock, not an
infeasible connector. A heading-only "target behind" rule is not sufficient:
the target was behind for only 44.3% of the AUTO lock as the tractor orbited it.

Recommended recovery design:

1. Estimate monotonic path progress by projecting the tractor onto a bounded
   forward window of mission segments.
2. Select the lookahead target by forward arc length from that progress point,
   never by rescanning only from a stale discrete target.
3. Bound reacquisition to the current path section (particularly a connector)
   so it cannot jump into a later stripe.
4. If progress remains stalled or lateral distance exceeds a safe corridor,
   command neutral/stop and require manual recovery. Do not silently abandon a
   connector and continue mowing.
