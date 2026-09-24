# September 18 mission replays

Open either `.html` file in a browser on the field laptop. Both files are
self-contained and read-only; no network connection, tractor connection, or
web server is needed.

- `replay_20260918_121342.html`: morning main-backyard mission, including the
  0.50 m/s command periods.
- `replay_20260918_134506.html`: afternoon 1.00 m/s continuation, including
  the stop at the NRF24 radio-loss event. Use **Next event** or the event menu
  to jump to `Radio RADIO_LOSS` at controller elapsed 378.82 s.

The pale route is the planned mission; green is the recorded tractor trail.
Yellow marks the tractor and its heading, and pink marks the controller's
recorded target when available. The lower graph compares commanded and actual
speed through time. The fact panel combines each pursuit sample with the
nearest field-telemetry sample.

This is a historical replay, not a simulation or a proposed route. The older
2026-08-31 replay builder discarded non-driving samples; this version retains
sampled waits and every important state transition so the radio failure is
visible. Normal driving is sampled at up to 2 Hz and waits at up to 0.5 Hz;
transitions are always included. The original logs remain authoritative for
exact 20 Hz timing.

Rebuild both files from the archived logs with:

```powershell
python field_testing/tools/build_tractor_mission_replays_20260919.py
```

The builder checks the archived pursuit and field-log SHA-256 values before
generating output. It does not modify the raw run archives.
