# DRAFT - UNTESTED IN THE FIELD

# Review of `mission-startup-process.txt`

The original file was preserved unchanged. Its useful equipment-preparation,
base-station download, PuTTY, preflight-log, mission dashboard, evidence-copy,
and shutdown concepts were retained in a new Approval A checklist.

Corrections made:

- Separated stationary instrumentation work from moving missions and blade
  operation. Approval A contains no engine start, Manual driving, production
  Auto, mission start, or blade engagement.
- Distinguished the two operator-reported phone networks: `TractorField` on
  the `192.168.10.1` router for the production-like ZeroTier trial, and
  `GL-SFT1200-9f4` on the onboard `192.168.8.1` router for the direct trial to
  Tractor01 at `192.168.8.20`.
- Added the non-negotiable current-session Heading-F9P profile application,
  independent readback, `rtcm-server` active check, heading reacquisition, and
  fresh `MISSION PREFLIGHT PASS` in the required order.
- Removed the ambiguous transmission-release sequence from Approval A and
  replaced it with the operator-confirmed mechanical fact recorded on
  2026-10-29: the lever on the rear of the tractor is connected to the
  hydrostatic transmission; pulling it out disengages the transmission, so a
  JRK forward/reverse command cannot propel the tractor. Approval A now
  requires this lever fully pulled out as an independent mechanical barrier.
  The operator further confirmed that the lever is pushed into the
  drive-enabled position only when the tractor is ready for authorized physical
  movement. Approval A never reaches that hold point.
- Added deployment tests, diagnostics-file verification, explicit proof that
  `COMMUNICATIONS DEGRADED` remains disabled, dry-run A/B commands, stop
  conditions, evidence hashing/copying, and clean Pi shutdown.
- Added a URL-republish helper that preserves the running service and persistent
  key rather than restarting control merely to obtain another ntfy notice.
- Updated the mission evidence collector to include readable files from
  `/var/log/pilog`.
- Recorded that a mission replay is not applicable to stationary Approval A.
  A later moving run must use a run-specific replay build after its evidence
  archive is copied and verified; the October 21 builder is hard-coded and is
  not a generic future-run command.

Operator confirmation recorded on 2026-10-29 for a later blades-off moving
test: enable voice guidance; drive to the start in Manual; select Pause; pull
the rear transmission-disengage lever fully out; walk to the dashboard; press
Start Mission; wait for `READY FOR GUARDED AUTO`; return to the tractor; push
the lever into the drive-enabled position; and press Auto on the handheld.
This is operating-sequence evidence, not field authorization.

The operator selected the first Approval A session as one 10-minute Trial A
run plus one 10-minute Trial B run, followed by evidence review. This choice is
session scope, not field authorization. Any later full qualification remains a
separate decision and would be ten runs total: five A runs and five B runs in
randomized order.

The operator confirmed that Auto is selected while standing on the ground
beside the tractor. The response said "about feet away" without a number, so
the exact distance remains intentionally unrecorded rather than guessed.

For a separately approved future blade-authorized mission, the operator also
confirmed the original order: stand behind the rear wheel and engage PTO;
push the rear transmission lever into the drive-enabled position; move to the
side of the tractor while remaining on the ground; then press Auto on the
handheld. Recording this sequence does not authorize blades or motion.

Still requiring field/operator confirmation or test-day evidence:

- the omitted numeric distance in the operator's statement that Auto is
  selected while standing on the ground beside the tractor "about [number]
  feet away"; this does not affect stationary Approval A;
- test-day SSID/BSSID/band/signal records from the phone for both named
  networks, plus router system/client-statistic exports; and
- a verified, read-only way to collect per-station Wi-Fi retry counters from
  each router. This must not become an improvised router configuration change
  on test day.

