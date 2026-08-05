# Head-valid straight diagnostic — 2026-08-05

This is a 20.0 m straight-only diagnostic beginning at the parked pose measured
after the 15 m test and proceeding at compass heading 168.228 degrees.

- Speed: 0.50 m/s
- Lookahead: 2.00 m
- Waypoint spacing: 0.25 m
- Autonomous turns: none
- Expected runtime: approximately 40 seconds plus any GPS safety stops
- Deck: disengaged

The purpose is to reproduce and measure short `headValid=False` events without
the aggressive 1.00 m lookahead or keyhole geometry of the inner-stripes
mission. The controller remains fail-closed and commands a stop whenever
`headValid` is false.

The centerline is contained in the recorded site polygon. Calculated minimum
centerline-to-boundary clearance is approximately 2.60 m, so this is a tightly
supervised diagnostic. Use Pause immediately for any unexpected behavior.

The JRK feedback anomaly found in the aborted inner-stripes start is unresolved.
Do not run this diagnostic unless the operator accepts that limitation and is
ready to select Pause immediately.
