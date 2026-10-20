# Tractor01 Manual Recovery — wallet card

**RECOVERY MANUAL ONLY — BLADES OFF — KEEP PHYSICAL E-STOP READY**

1. Clear/secure tractor; PTO off.
2. Select physical **PAUSE**; verify neutral/no motion.
3. Tractor01: `sudo tractor-manual-recovery start`
4. Phone: join verified onboard-router SSID **TractorField**.
5. Open recovery-only ntfy link. Verify:
   - `RECOVERY MANUAL ONLY`
   - `https://192.168.8.20:8765`
   - `eth0`, clients `192.168.8.0/24`
   - `CONNECTED · PAUSE`
6. Inspect clear route to parking.
7. Hold **MODE SELECT** + press **MANUAL**.
8. Drive slowly: max 25% reverse / 60% forward.
9. Park → Neutral → guarded **PAUSE** → verify stopped.
10. Tractor01: `sudo tractor-manual-recovery stop`
11. Shut down safely; collect recovery evidence.

If anything is stale, ambiguous, disconnected, or unexpected: **physical
E-stop, remain in Pause, do not re-arm.** Recovery stop never restarts normal
control automatically.
