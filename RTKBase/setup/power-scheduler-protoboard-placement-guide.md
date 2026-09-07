# Power Scheduler Add-On — Protoboard Placement Guide

Companion to `bridgeville_power_scheduler.kicad_sch` (open that file for the authoritative electrical connections — this document is a *suggested* physical layout for your 2-sided protoboard, columns A-X / rows 1-18, 0.1"/2.54mm pitch).

**Important caveat**: I don't have the exact mechanical footprint (pin spacing) for every part in hand, and the relay in particular is designed for a purpose-made PCB, not generic protoboard — its pins may not land perfectly on your hole grid. Treat the zones below as a starting floorplan; you may need to bend a lead or shift a part a hole or two to make everything land cleanly. Verify each part's actual pin spacing against your board before soldering anything down.

## Suggested zone layout

Think of the board in four left-to-right zones, with the "always-hot 12V" side on the left and the two switched outputs on the right:

| Zone | Approx. columns | Rows | Contents |
|---|---|---|---|
| 1 — Power in | A–D | 1–6 | Battery bus tap (12V+ / GND) terminal pads or a small 2-pin screw terminal footprint |
| 2 — Converter + relay driver | E–L | 1–10 | Matek UBEC DUO (mount off-board or via short flying leads — it's a small potted module, not a THT part — bring its VIN/GND/OUT1/OUT2 leads to pads in this zone); DRV8871 breakout module (mount via its own 0.1" header pins, ~6 columns wide) |
| 3 — Relay + Pi switch | M–S | 1–14 | SRD-12VDC-SL-C relay (give it a generous footprint, ~5-6 columns x 4-5 rows, and confirm pin spacing before drilling in); IRF9540N (TO-220, mount vertically to save row space, bend leads to fit 0.1" spacing) + 2N3904 (TO-92) + R1/R2 (10k, standard axial, bend to 0.1"/0.2" spacing) clustered together just below/right of the relay |
| 4 — Outputs + ESP32 header | T–X | 1–18 | Header pins / pigtail pads for: J1 (Pi wake-trigger GPIO), J2 (Starlink+router 12V), J3 (Pi 5V), plus the ESP32 module itself if it's mounted on this same board (otherwise these are just wire-out points to your existing ESP32 dev board) |

## Practical notes

- Keep the 12V battery-bus trace/wire run (Zone 1 → UBEC VIN → DRV8871 VM → relay COM) short and on generously doubled-up solder-bridged rows if your board supports it — this carries the highest current in the circuit.
- Give the relay its own dedicated footprint area and dry-fit it (push the leads into candidate holes) before soldering anything else nearby — this is the one part most likely to need adjustment.
- The IRF9540N (TO-220) can be mounted flat against the board with its tab up, or vertically with bent legs — vertical saves row space if the board is tight, given you're on 2.5mm/0.1" spacing.
- Route GND as a shared rail/bus row across all four zones if your protoboard has dedicated rail rows along an edge — every part in this design references the same common ground.
- Leave the relay's NC contact pin unconnected (per the schematic) — don't solder anything to it.
- Double-check the Matek UBEC's Out-1 Aux pins are left unconnected/floating — that's intentional (see the schematic notes).

This is a starting point, not a fabrication-quality layout — adjust freely to fit your enclosure's wire entry points and whatever's easiest to hand-solder cleanly.
