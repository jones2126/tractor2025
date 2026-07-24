# 62 Collins Drive Coverage Mission

**Status: static validation PASS — not yet field approved**

The executable controller file is:

```text
62_Collins_Dr_mission.txt
```

It contains the controller's required five whitespace-separated columns:

```text
lat lon yaw_rad lookahead_m speed_mps
```

## Mission summary

| Item | Value |
|---|---:|
| Coverage angle | 105° math frame |
| Compass stripe headings | 345° / 165° |
| Lane spacing | 0.9906 m (39.0 in) |
| Planning turn radius | 1.90 m |
| Waypoints | 1,424 |
| Route length | 651.0 m |
| Speed range | 0.30–0.50 m/s |
| Maximum waypoint gap | 0.50 m |
| Minimum validated sampled radius | 1.88 m |
| Static validation | PASS |

Manual circle tests measured approximately 1.05 m full-left and 1.63 m
full-right radii. The mission uses one common 1.90 m planning radius, about
0.27 m gentler than the weaker measured right turn.

The mission uses compact three-arc agricultural keyhole/omega connectors. One
boundary-constrained fallback is required between stripe 2 and stripe 3 and is
marked with a red triangle in the mission preview. This maneuver requires
specific visual and low-speed field review.

## Versioned files

| File | Purpose |
|---|---|
| `62_Collins_Dr_mission.txt` | Executable Pure Pursuit mission |
| `01_boundary_final.csv` | Reviewed final site polygon |
| `02_plan_settings.json` | Exact planner inputs used on the Windows planning machine |
| `02_coverage_segments.csv` | Reviewed coverage rows and execution order |
| `62_Collins_Dr_mission_audit.csv` | Waypoint-by-waypoint local geometry and controller parameters |
| `62_Collins_Dr_mission_build_report.json` | Connector modes, radii, containment, and build provenance |
| `62_Collins_Dr_mission_validation.json` | Independent static validation results |
| `62_Collins_Dr_mission_preview.png` | Visual mission checkpoint |
| `62_Collins_Dr_mission_validation.png` | Visual validation checkpoint |

The settings and reports retain their original absolute Windows source paths as
provenance. The executable mission itself has no path dependency.

## Integrity

SHA-256 for `62_Collins_Dr_mission.txt`:

```text
A17A34EE28531C39D62E0D77FD736B3442A4F86D82F92E89839DC91712234BF0
```

## Visual checkpoints

![Executable mission preview](62_Collins_Dr_mission_preview.png)

![Independent validation](62_Collins_Dr_mission_validation.png)

## Field hold point

Static `PASS` confirms file structure, containment, waypoint spacing, yaw,
speed, and sampled curvature. It does not confirm map-image accuracy, terrain,
mower-deck clearance, controller tracking, or obstacle clearance.

Before normal operation:

1. Pull this exact Git revision onto the tractor RPi.
2. Confirm the mission SHA-256.
3. Perform a controller-load/no-motion check.
4. Review the stripe 2-to-3 boundary fallback.
5. Conduct a supervised low-speed test with RTK Fixed and immediate e-stop
   access.

This directory contains exact geographic coordinates. Do not publish it in a
public repository unless that disclosure is intentional.
