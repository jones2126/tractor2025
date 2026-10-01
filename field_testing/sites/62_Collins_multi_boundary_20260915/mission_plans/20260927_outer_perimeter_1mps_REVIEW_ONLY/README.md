# 62 Collins outer-perimeter mission review

This package contains three separate controller-format outer-perimeter paths at 1.0 m/s. It is for review, not field execution.

- The revision-3 outer boundary is treated as the desired left deck-edge perimeter. The clockwise GPS centerline is inset by the 0.5334 m deck half-width.
- Where an obstacle touches an outer boundary, the path detours around the obstacle exclusion plus the 0.5334 m deck half-width and a 12-inch first-run margin.
- Every area runs clockwise so the tractor's left deck edge faces the outer perimeter. Each path starts near its named recorded access point.
- Owner review confirms adequate room around OBS-NEW-003, 007, 011, and 012. Their final paths must use broad tangent-connected detours, not the current hard polygon-cut joins.
- Red X markers identify adjacent waypoint heading changes of at least 45 degrees; those corners must be smoothed or replaced before field authorization.
- The three files are intentionally separate. Access connectors, between-area sequencing, start-pose checks, and recovery phase gates must be resolved before creating a master mission.
- No launcher or dashboard target has been generated.

Review the PNG and report first. The obstacle-loop mission will be built separately at 0.75 m/s around obstacles and 1.0 m/s on approved connectors.
