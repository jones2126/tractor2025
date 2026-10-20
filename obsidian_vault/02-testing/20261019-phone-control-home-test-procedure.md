# 2026-10-19 phone-control home test procedure

## Scope

This procedure uses the real phone but no Tractor01 or field components. It can
verify browser behavior and server state transitions against dry-run or mocked
telemetry. It cannot verify the Teensy, JRK, steering hardware, engine, wheels,
RTK, Tractor01 networking, or field routers.

Record every result as one of:

- **automated verified**;
- **real-phone verified against non-tractor endpoint**;
- **inferred from October 6 evidence**; or
- **deferred for Tractor01**.

## Automated gate

From the repository root on the development computer:

```powershell
python -m unittest `
  tractor_rpi.testing.test_wifi_primary_control_20261003 `
  tractor_rpi.testing.test_mission_dashboard_stale_state_20261019
```

Expected result: all tests pass. These tests cover session expiry/reclaim,
guarded mode selection, Manual-only recovery policy, explicit ZeroTier-primary policy,
bounded flight-recorder ingestion, and dashboard stale-state markers.

## Real-phone browser checks

These checks require serving the control page from a trusted HTTPS origin that
the phone can reach. Do not weaken certificate checking merely to make Wake
Lock or microphone features available.

1. Start only a dry-run/test server with simulated expected-firmware telemetry.
   The required flags are `--dry-run --simulate-bridge`. The server refuses
   simulated telemetry without dry-run. Do not connect UDP 6004 to a tractor or
   actuator.
2. Open the keyed test URL on the phone. Confirm it claims a new session in
   Pause and shows the Wake Lock result.
3. Leave the page visible for at least two minutes. Confirm the UI remains
   responsive and flight-recorder batches appear in a distinct
   `phone_flight_recorder_*.jsonl` file.
4. Arm simulated Manual using the guarded two-button action. Hide the page or
   lock the phone. Confirm the server records `page_hidden` when delivery is
   possible; otherwise confirm the simulated server freshness expiry removes
   the session.
5. Restore the page. Confirm it clears the old session, reconnects in Pause,
   zeros the sliders, and does not resume Manual or Auto.
6. Wait through the configured stability period and confirm that motion still
   requires a new guarded selection.
7. Repeat while toggling the phone between home Wi-Fi and cellular. Treat the
   Network Information API record as a hint only. Record whether the browser
   emitted `online`, `offline`, `connection_hint_changed`, visibility, or Wake
   Lock events and whether the HTTPS request stream stopped.
8. Close and reopen the browser. Confirm unsent events survived in bounded
   local storage and upload after reconnection.

Do not call the Wi-Fi/cellular exercise a physically qualified ZeroTier motion
test. It is evidence about the real phone/browser/overlay behavior against the
chosen home test endpoint.

For this test only, the server permits a ZeroTier subnet override when both
`--dry-run` and `--simulate-bridge` are present. Production startup rejects
custom subnet overrides. The explicit `zerotier-primary` policy is the intended
production selection, while physical acceptance remains deferred.

## 2026-10-19 real-phone result

Classification: **real-phone verified against non-tractor endpoint**.

- Pixel 6 reached the dry-run server over ZeroTier on home Wi-Fi and on 5G.
- Guarded Manual remained at zero drive and zero steering throughout.
- Locking/hiding the page changed Manual to Pause and restored only a fresh
  Pause session.
- The initial Airplane-mode test exposed a stale local `MANUAL LIVE` label even
  though the server had latched Pause. An acknowledgement watchdog, request
  timeout, late-response guard, and queued heartbeat were added and retested.
- In the corrected test the phone changed to
  `RECONNECTING · TRACTOR PAUSED` 506 ms after its last command acknowledgement.
- The original dry-run server independently latched Pause about 949 ms after
  its last accepted command because its threshold was 800 ms. The server
  threshold was then aligned to the unchanged 500 ms Teensy target. The
  physical Teensy watchdog was not present and therefore remains unverified.
- After Airplane mode was removed, ZeroTier recovered automatically over 5G;
  it did not need to be cycled. The new session began in Pause at about 110 ms
  displayed RTT.
- One Wi-Fi-to-5G transition required cycling ZeroTier, so automatic underlay
  migration is not yet consistently demonstrated.
- Before the heartbeat-queue fix, a 92-second 5G Manual-zero sample had 162 ms
  median RTT, 283 ms p95 RTT, and 424 ms maximum command gap. After the fix, a
  short 7.6-second sample had 110 ms average RTT, 187 ms p95 RTT, and 374 ms
  maximum command gap. A longer Tractor01-side soak is still required.

## Tractor01 deployment gate — deferred

Before installing these changes on Tractor01:

1. Review the authority policy and service diff.
2. Confirm the expected Wi-Fi-primary firmware is actually installed.
3. Re-run the automated suite on Tractor01.
4. Perform engine-off stationary tests, including real page hide/lock and
   server restart.
5. Perform wheels-raised steering/JRK tests and prove physical neutralization
   inside the 500 ms bound.
6. Measure each candidate network separately with persistent route/interface
   logging and the real phone.
7. Only after those pass, prepare a separately approved very-low-speed short
   route. Do not use the 19,825-waypoint mission for initial fault injection.
