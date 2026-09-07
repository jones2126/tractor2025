# Bridgeville RTK Base — Power Scheduler Redesign (Draft)

Status: design agreed in conversation 2026-09-04, refined the same day across several rounds. Hardware BOM and pin assignments are now finalized; ESP32/Pi firmware not yet written. This captures the decisions so implementation can pick up later without re-deriving them.

## Why

Today the Starlink Mini + router run continuously off the battery (not switched), while only the Pi is on the Renogy Load terminals. Starlink Mini alone is roughly 20-40W (spikes 40-60W for 1-3 min on cold start); the router (TP-Link Archer AXE5400, tri-band WiFi 6E) ships with a 12V/2.5A (30W max) adapter, though typical draw is usually well under that ceiling. Combined, that's the dominant load on a 100W panel / 12V 10-12Ah battery system. Running it only during scheduled windows (default 08:00-10:00, extended on "work days") cuts that to a couple of hours a day instead of 24.

Sources: [Starlink Mini Power Consumption](https://gridwright.com/blog/starlink-mini-power-consumption), [Anker SOLIX Starlink Mini guide](https://www.ankersolix.com/blogs/smart-home/starlink-mini-power-consumption)

## Network topology (confirmed)

Starlink Mini is in **bypass mode** — it's just the satellite modem/WAN feed. A **TP-Link Archer AXE5400** router does the actual routing, DHCP, and the `TractorField` WiFi SSID. Both are powered together off the same switched 12V branch. Implication for the ESP32's nightly sync: the router's local WiFi AP typically comes up quickly after power-on, but actual internet reachability depends on Starlink completing satellite acquisition (~1-2 min, sometimes longer) — the firmware should retry the HTTPS GET with backoff for a couple of minutes rather than assuming one attempt right after WiFi association will succeed.

## Hardware not yet in the repo docs (as of this writing)

- Starlink Mini + AXE5400 router provide the `TractorField` WiFi SSID and Ethernet to the Pi. Both are powered directly off the 12V battery via the Renogy — **not** through the Load terminals.
- Only the Pi is on the Renogy Load leads, through a Matek UBEC DUO 4A converter (SKU U4A2P — the existing TOBSUN replacement noted in `setup-Bridgeville.md`).
- `setup-Bridgeville.md` and `setup/README.md` should be updated to reflect this once the new design is built (currently they only describe the Pi/GPS/ESP32-logger power path).

## Finalized architecture

**Battery bus tap**: both switched branches (relay and the Matek UBEC's input) move off the Renogy Load terminals onto a direct always-hot 12V battery tap.

**Starlink+router branch — single-coil latching relay + H-bridge driver**:
- Relay: **Songle SRD-12VDC-SL-C** (single-coil latching/"self-locking", Form C/SPDT, 10A@28VDC contacts — comfortably covers the combined Starlink Mini + AXE5400 current). Reverse coil polarity sets vs. resets it; holds position with zero coil current once switched. [LCSC datasheet](https://www.lcsc.com/datasheet/C30431.pdf), widely available on Amazon (e.g. search "SRD-12VDC-SL-C").
- Driver: **TI DRV8871** H-bridge breakout (e.g. [Adafruit's board](https://www.amazon.com/Adafruit-DRV8871-Motor-Driver-Breakout/dp/B06Y4VRXN4), 3.6A max, VM range 6.5-45V so it runs directly off the 12V battery bus — a 5V-rail supply would be under its minimum). ESP32 GPIO25→IN1, GPIO26→IN2: IN1=H/IN2=L sets the relay (power ON), IN1=L/IN2=H resets it (power OFF), both low = idle with zero coil current. Relay COM ties to the battery bus, NO feeds the Starlink+router branch, NC is unused.
- A 12V-coil relay was chosen (over the 5V `SRD-05VDC-SL-C` variant considered earlier) specifically so the DRV8871 can be powered straight from the battery bus rather than needing a second, lower-voltage driver IC.

**ESP32 + Pi power — one Matek UBEC DUO 4A (U4A2P)**, the same converter already in the build, rewired:
- **Out-2** (plain 5V, no on/off control) → ESP32. Always hot whenever the converter has 12V input.
- **Out-1** (5V, jumper-set) → feeds a P-MOSFET high-side switch, not the Pi directly.
- **Finding (from the U4A2P manual, reviewed 2026-09-04): the board's built-in Aux control is not used.** It's an RC-receiver-style PWM channel that defaults Out-1 back ON whenever no valid signal is present — including whenever the ESP32 is in deep sleep, which is most of the day. Using it would fight the goal (Pi would power back on exactly when the ESP32 sleeps to save power). Decision: leave the Aux pins disconnected (Out-1 then just stays always-on) and switch the Pi downstream with a MOSFET instead.
- **Pi switch**: IRF9540N (P-channel, TO-220 — easy to hand-solder, huge current/voltage margin over the ~0.7-1.2A a Pi 3B draws) as a high-side switch, driven through a 2N3904 NPN level-shifting stage from ESP32 GPIO27 (base resistor 10kΩ; gate pull-up resistor 10kΩ from the MOSFET gate to Out-1's 5V, so the default state with the ESP32 pin low/floating is Pi **OFF** — the safe default, opposite of the Aux-control board's behavior). High-side (switching the +5V feed, not the ground return) was chosen over a low-side switch because the Pi's USB link to the ESP32 shares a ground path that could otherwise partially back-feed the Pi even with only its ground cut.
- Current headroom is comfortable throughout: Out-1/IRF9540N are rated far above the Pi's draw.

This also drops the Pi channel's dependency on Renogy's Modbus load on/off register (there's at least one documented report of the 10A Wanderer ignoring the `0xE01D` load-mode command).

**ESP32 pin plan** (WROOM-32 DevKit, RTC-capable GPIOs chosen to avoid boot-strapping pins and support `gpio_hold_en()` through deep sleep):

| Pin | Function |
|---|---|
| GPIO32 | DS18B20 temp sensor data (existing) |
| GPIO16 / GPIO17 | UART2 RX/TX to Renogy RS232-TTL (existing) |
| USB | Serial link to Pi, CSV downloads etc. (existing) |
| GPIO25 | DRV8871 IN1 (relay SET) |
| GPIO26 | DRV8871 IN2 (relay RESET) |
| GPIO27 | Pi power switch control (via 2N3904/IRF9540N) |
| GPIO33 | Wake-trigger input from Pi (ext0 deep-sleep wake source) |

**Schedule source**: publish the Google Sheet as CSV (File → Share → Publish to web → CSV) rather than using the Sheets API — a plain HTTPS GET is much simpler for the ESP32 than API-key/OAuth handling on a memory-constrained MCU. **All schedule-fetching and parsing logic lives only in the ESP32 firmware** — the Pi never fetches or parses the sheet itself (see mid-day overrides below, revised from the earlier draft).

**Nightly autonomous sync (11pm)**:
1. ESP32 wakes from deep sleep, drives DRV8871 to latch the Starlink+router relay on.
2. Retries HTTPS GET of the published CSV with backoff for up to a couple of minutes (router WiFi AP comes up quickly; actual internet path depends on Starlink completing satellite acquisition).
3. Parses the day's schedule, caches it (NVS/SPIFFS) — survives deep sleep and reboots.
4. Posts battery charge level via ntfy (same pattern as the existing wifi-monitor script).
5. After ~10 minutes, unlatches the Starlink+router relay unless the cached schedule says the window should be on right now.
6. Back to deep sleep until the next hourly wake.

**Hourly wakes (all other hours)**: no network needed — compare RTC time to the cached schedule, drive the relay and the Pi switch if a transition is due, go back to sleep. Idempotent: no action if already in the correct state.

**Mid-day overrides / forcing a fresh schedule pull** (e.g. "it's Monday 08:00, keep the system up until 15:00") — **revised**: the Pi does *not* fetch the sheet itself anymore, to keep schedule-retrieval logic in one place (the ESP32). Instead:
- The Pi has a spare GPIO wired to ESP32 GPIO33 (a dedicated wake-trigger line, separate from the USB-serial CSV link — UART alone can't wake the ESP32 from deep sleep, but a GPIO edge configured as an ext0 wake source can).
- SSH into the Pi over ZeroTier (already set up) and pulse that GPIO. If the ESP32 is asleep, this wakes it immediately (RTC ext0 wake); if it's already awake, it can just poll the same pin.
- On that wake reason, the ESP32 immediately re-fetches and re-parses the CSV itself (same logic as the nightly sync, since the Starlink+router branch is already on during an active window) and applies the result right away.
- This avoids needing to reach an ESP32 web interface on the local network, which the user correctly flagged as unreliable when off-site, and keeps the Pi's role trivial (one GPIO pulse) rather than duplicating schedule-parsing logic on both devices.

**Safe Pi shutdown before power-cut**: today the Pi runs continuously, so this has never come up. Once the Pi is power-cycled roughly daily (or more, on extended work days), a hard power cut risks SD card corruption. Before the ESP32 drops the Pi's MOSFET-gate GPIO at the end of a window, it should send a "shutting down" message over the existing USB-serial link; the Pi needs a small listener (systemd unit or similar) that runs `sudo shutdown -h now` on receipt; ESP32 waits ~30-45s (or for an ack) before cutting power.

## Bill of materials (new parts)

- 1x Songle SRD-12VDC-SL-C latching relay
- 1x DRV8871 H-bridge breakout module (e.g. Adafruit's, or a generic one)
- 1x IRF9540N P-channel MOSFET (TO-220)
- 1x 2N3904 NPN transistor (TO-92)
- 2x 10kΩ resistors
- Jumper wire for the new Pi↔ESP32 wake-trigger GPIO line (shares existing ground via the USB cable)
- (Matek UBEC DUO 4A / U4A2P — already owned, being rewired/repurposed, not a new purchase)

## Reference files

- `power-scheduler-esp32-redesign-DRAFT.md` — this file
- `bridgeville_power_scheduler.kicad_sch` — KiCad schematic (open directly in KiCad's Schematic Editor, no project file needed) capturing every connection above. Every part is drawn as a labelled rectangle rather than an accurate manufacturer symbol; same net name on two pins = electrically connected. Verify actual part pinouts (relay, MOSFET, NPN) against their printed diagrams before wiring — the schematic gives function per pin, not manufacturer pin-1 markings.
- `power-scheduler-protoboard-placement-guide.md` — suggested (not authoritative) physical zone layout for hand-soldering onto the 2-sided protoboard, since KiCad's PCB tools aren't built around an existing hole-labelled perfboard grid.

## Open items / next steps

- Confirm the published-CSV sheet format/columns so the ESP32 parser matches the actual layout.
- Dry-fit the SRD-12VDC-SL-C relay's pins against the protoboard's hole grid before committing to a layout — it's designed for a purpose-made PCB, not confirmed to land cleanly on generic 0.1" protoboard spacing.
- Write the ESP32 firmware (relay/MOSFET drivers, deep sleep/RTC wake loop incl. ext0 wake on GPIO33, CSV fetch/parse/cache, nightly sync with retry/backoff) and the Pi-side wake-trigger script + shutdown listener.
- Update `setup-Bridgeville.md` with the finalized wiring and the new services once built.
