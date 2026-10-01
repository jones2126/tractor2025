# September 23, 2026 main-backyard corner diagnosis

## Corrected region selection

This analysis uses the **southeast/lower-right corner of the nested `main_backyard_ring_*` paths**, not the recorded-manual stripes in the center. Each lower-right corner is paired with the northwest/upper-left corner from the same ring. Aggregate results cover complete rings 1–18; rings 4, 8, and 13 are labeled LR-1…3 and UL-1…3 as outer/middle/inner problem examples with healthy RTK/heading.

## Bottom line

There is a deterministic difference. Across all 18 matched rings, lower-right mean absolute geometric CTE is **0.173 m** versus **0.156 m** upper-left. Mean per-ring P95 is **0.246 versus 0.224 m**; lower-right is worse on 12/18 matched pairs. This is a real but modest centerline-tracking penalty of about 2.3 cm at P95.

The larger difference is controller/path geometry. Lower-right corners turn through **91.9°** on average versus **49.0°** upper-left, with median curvature radii **1.63 versus 1.73 m**. Logged/dashboard `cross_track_err_m` averages **0.521 versus 0.331 m** and is higher lower-right on 17/18 pairs.

Lower-right also demands roughly twice the right-steering command (median-command mean -0.680 versus -0.358), with steering-error P95 **75.8 versus 30.9 counts** and estimated lag **0.100 versus 0.067 s**. Steering error is larger lower-right on 15/18 pairs. That makes **the sharper ~90° planned corner plus Pure Pursuit corner-cutting and higher steering demand** the strongest supported cause; mower-deck swept geometry can amplify the resulting inside gap.

Positive signed CTE means tractor-left of travel. All selected and aggregate corners are right turns, so negative signed CTE—or positive direction-normalized `inside_cte`—means the tractor is inside the planned curve.

## Selected matched turns

|Turn|Ring|Region|Dir.|Turn °|Radius m|Entry/apex/exit WP|Actual m/s|Mean |CTE| m|P95 |CTE| m|Apex inside m|Exit inside m|Logged CTE mean m|Steer cmd|Steer error P95|Lag s|

|---|---:|---|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|LR-1|4|lower-right|right|90.7|1.63|3967/3978/3990|1.00|0.181|0.243|+0.230|+0.138|0.519|-0.708|43.5|0.10|
|UL-1|4|upper-left|right|44.1|1.81|4509/4516/4522|0.93|0.118|0.174|+0.167|+0.097|0.334|-0.351|22.2|0.10|
|LR-2|8|lower-right|right|92.1|1.63|8163/8174/8184|1.00|0.176|0.303|+0.248|+0.108|0.471|-0.585|39.2|0.10|
|UL-2|8|upper-left|right|47.9|1.63|7839/7846/7853|0.93|0.119|0.203|+0.180|+0.112|0.389|-0.429|23.0|0.05|
|LR-3|13|lower-right|right|94.4|1.63|10805/10815/10826|1.01|0.223|0.280|+0.280|+0.241|0.495|-0.627|88.0|0.15|
|UL-3|13|upper-left|right|53.2|1.63|10602/10609/10616|0.91|0.161|0.235|+0.209|+0.138|0.370|-0.485|33.2|0.05|

## All-ring regional comparison

|Metric|Lower-right|Upper-left|Difference|

|---|---:|---:|---:|
|Mean absolute geometric CTE|0.173 m|0.156 m|+0.017 m|
|Mean per-ring P95 geometric CTE|0.246 m|0.224 m|+0.023 m|
|Mean planned turn angle|91.9°|49.0°|+42.9°|
|Median curvature radius|1.628 m|1.732 m|-0.104 m|
|Logged/dashboard CTE mean|0.521 m|0.331 m|+0.190 m|
|Logged/dashboard CTE P95|0.772 m|0.508 m|+0.264 m|
|Inside CTE at apex|+0.229 m|+0.201 m|+0.028 m|
|Actual speed|0.924 m/s|0.894 m/s|+0.030 m/s|
|Median normalized steering command|-0.680|-0.358|-0.321|
|Steering error absolute P95|75.8 counts|30.9 counts|+44.9 counts|
|Estimated steering lag|0.100 s|0.067 s|+0.033 s|
|Path-heading error absolute P95|12.5°|11.9°|+0.6°|
|Median lookahead|1.851 m|1.929 m|-0.077 m|

## Selected locations and telemetry

|Turn|Entry lat, lon|Apex lat, lon|Exit lat, lon|Approach °|Target / measured counts|Heading median / abs P95 °|Accuracy P95 °|RTK / valid / carrier fixed %|

|---|---|---|---|---:|---:|---:|---:|---:|
|LR-1|40.4852720, -80.3323154|40.4852611, -80.3323226|40.4852582, -80.3323401|187.0|289 / 298|+3.3 / 12.9|0.53|100 / 100 / 100|
|UL-1|40.4855665, -80.3327197|40.4855753, -80.3327129|40.4855803, -80.3327008|25.9|405 / 397|+3.6 / 11.3|0.55|100 / 100 / 100|
|LR-2|40.4853146, -80.3323583|40.4852988, -80.3323638|40.4852945, -80.3323836|177.6|329 / 325|+4.9 / 14.1|0.53|100 / 100 / 100|
|UL-2|40.4855373, -80.3326865|40.4855486, -80.3326780|40.4855521, -80.3326690|23.6|382 / 388|+3.5 / 12.4|0.55|100 / 100 / 100|
|LR-3|40.4853563, -80.3324222|40.4853427, -80.3324257|40.4853371, -80.3324449|172.5|315 / 316|+3.4 / 14.1|0.53|100 / 100 / 100|
|UL-3|40.4855033, -80.3326431|40.4855125, -80.3326361|40.4855165, -80.3326236|21.0|363 / 373|+1.8 / 13.4|0.55|100 / 100 / 100|

## Existing logged CTE check

Across all 36 matched corner cores, logged `cross_track_err_m` versus independently computed geometric |CTE| has correlation **0.013** and RMSE **0.367 m**. The log dictionary defines it as `abs(yt_m)`: lateral offset to the lookahead target in the tractor frame. It is useful for Pure Pursuit demand, but it is not nearest-path geometric CTE.

## Answers to A–F

**A. Deterministic difference?** Yes. Lower-right geometric CTE P95 is 0.246 versus 0.224 m, and the controller-demand metric is higher on 17/18 matched rings.

**B. Strongest measured difference?** Planned sweep angle (91.9° versus 49.0°), followed by logged lookahead-target offset (0.521 versus 0.331 m) and steering demand/error.

**C. Likely cause?** Primarily path geometry/Pure Pursuit corner-cutting under a much stronger right-turn demand, with steering response as a secondary contributor. Speed differs by only a few hundredths of a metre per second on average. RTK position stayed fixed and heading validity was 100%; isolated carrier/accuracy degradations do not repeat across the lower-right set. Terrain is not recoverable from these logs. Deck footprint/overlap likely converts the modest centerline error into visible uncut wedges.

**D. Where?** The repeatable bias is strongest around the apex: inside displacement averages +0.229 m lower-right versus +0.201 m upper-left. Both groups remain somewhat inside during exit/recovery.

**E. CTE sign?** Yes. All matched corners are right turns, and the independently computed signed CTE is consistently negative/inside near the lower-right apexes.

**F. Dashboard CTE trustworthy?** Trustworthy as a lookahead-target/Pure Pursuit demand value, not as geometric path error. Correlation with independent |CTE| is 0.013; RMSE is 0.367 m.

## Mowing geometry

The mission was originally documented as a supervised blades-off tracking-validation build, not a deck-footprint coverage plan. The data supports a real lower-right tracking penalty, but it is only about 2–3 cm at P95. The visibly uncut grass is therefore most plausibly the combination of that inside bias with the tighter/full-90° corner and the physical deck trajectory. Exact swept coverage still requires antenna-to-rear-axle/deck offsets and effective cutting width.

## Plots

- [Plan view of the nested rings and selected matched corners](turn_plan_view.svg)
- [Independent signed CTE through normalized turn progress](cte_normalized.svg)
- [Steering target versus measured](steering_representative.svg)
- [Speed through representative curves](speed_representative.svg)

## Reproducibility

Run from the repository root:

```powershell
& 'C:\Users\al532\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' field_testing/tools/analyze_20260923_turns.py
```

Exact source and provenance files:

- `field_testing\sites\62_Collins_multi_boundary_20260915\runs\20260923_124933_master_manual_5hz\pursuit_log_20260923_124935.csv` — SHA-256 `220053d195d93419ccbdfe7d0b12551b67db9698b4844d909cd36172f49f8b63`
- `field_testing\sites\62_Collins_multi_boundary_20260915\runs\20260923_124933_master_manual_5hz\master_manual_field_20260923_124933.csv` — SHA-256 `ea6c20f38bd2dbe987162cf93b948cbe62e7b45ed06fbc5c99e02ad2c4a18e13`
- `field_testing\sites\62_Collins_multi_boundary_20260915\mission_plans\20260915_master_boundary_replay\generated_master_manual_field_20260920\62_Collins_master_manual_resampled_1mps_20260920.txt` — SHA-256 `0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4`
- `field_testing\sites\62_Collins_multi_boundary_20260915\mission_plans\20260915_master_boundary_replay\generated_master_manual_field_20260920\62_Collins_master_manual_resampled_1mps_20260920_audit.csv` — SHA-256 `844e3358195283bb3225e9958c4ce4637240c993c604a1d15484619249d67932`
- `field_testing\sites\62_Collins_multi_boundary_20260915\analysis\20260923_mission_replay\replay_20260923_124933_master_manual_5hz.html` — SHA-256 `37bb438b8e7a4f451dcfdfd37c5116ba8f1492b326dfc546312b77101002cdaf`
- `field_testing\tools\build_tractor01_mission_replay_20260923.py` — SHA-256 `c0454534b14d7e82fce63d2372d0e43883f87e76688fdcea78047a8ee7fc4755`
- `field_testing\sites\62_Collins_multi_boundary_20260915\mission_plans\20260915_master_boundary_replay\MASTER_MANUAL_FIELD_TEST_20260920.md` — SHA-256 `bf550ae3a6c7991c5bb1cf0735803204f09be3baf32eef17dc244de8a7bc61a5`

Key calculations:

- Local east/north projection matches the replay builder: `x=(lon-lon0)*111320*cos(lat0)`, `y=(lat-lat0)*110540`.
- Geometric CTE projects each tractor position to the closest segment in a waypoint-hinted local path window; sign is the 2-D cross product of path tangent with tractor-minus-projection.
- Each ring is normalized independently; southeast is the maximum `(zEast-zNorth)` corner and northwest is the maximum `(-zEast+zNorth)` corner.
- Metrics use the sustained-curvature core (|curvature| ≥ 0.15 1/m, short gaps bridged). Apex is the sample closest to half the net heading rotation.
- Approximate radius is the reciprocal of median absolute planned curvature within the core. Exit/recovery uses normalized progress 0.75–1.15.

## Next-step candidates (not implementation instructions)

1. Measure the GNSS-reference-to-deck geometry and effective cutting width, then generate a swept-deck overlay for the southeast corners.
2. Compare a larger-radius or two-stage southeast corner in replay, preserving the same ring spacing, before changing controller gains.
3. If geometry alone is insufficient, evaluate curvature-aware lookahead/speed or right-turn steering feed-forward against these same 18 matched pairs.
