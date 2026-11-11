# Tractor01 continuous battery-voltage logger

**DRAFT - UNTESTED IN THE FIELD**

This independent, read-only service listens to the existing UDP 6003 status
broadcast and records the JRK VIN measurement every ten seconds. It sends no
tractor command and does not run inside the Pure Pursuit, RPi safety supervisor,
or Teensy serial loops.

Daily files are stored as
`/var/lib/tractor-battery/tractor_battery_YYYY-MM-DD.csv`. Each row includes the
measured millivolts, validity flag, telemetry age, boot identity, system uptime,
firmware identity, operating modes, JRK diagnostic sequence/timeouts, and JRK
halting-error field. Files older than 90 days are removed using an exact
logger-specific filename pattern.

The voltage provides a conservative context hint: charging source detected,
no charging source detected, low voltage, or invalid. Voltage alone cannot
distinguish the engine alternator from an attached battery charger. A future
engine-run or charger-state input is still needed if those two sources must be
identified separately rather than annotated from field-session context.

The unit starts after `tractor-rpi-arbitrator.service` during a normal boot but
remains independent so a bridge restart does not terminate the logger. If the
UDP feed becomes stale, the retained millivolt value is explicitly marked
invalid until fresh telemetry returns. It is independent of mission dashboards
and mission loggers.

Installation and activation are intentionally separate. On Tractor01, after a
repository pull and explicit approval for a stationary motor-off check:

```bash
sudo /home/al/tractor2025/tractor_rpi/setup/install_tractor_battery_logger_20261111.sh --start-now
```

Verification should confirm that the service is active, a valid row arrives at
least every ten seconds, the voltage agrees with a multimeter within the JRK's
expected measurement tolerance, mission/preflight UDP consumers still receive
their normal rates, and stopping the service does not affect control services.

This logger is observational. It does not impose a low-voltage threshold or
change tractor behavior.
