# Obsidian Vault Setup Guide — tractor2025
**Created:** 2026-05-20
**Status:** Finalized — ready to implement when convenient
**Supersedes:** Obsidian_Vault_Consolidation.md

---

## Vault Location

The Obsidian vault moves into the tractor2025 GitHub repo as a dedicated folder:

```
/home/albert/tractor2025/obsidian_vault/
```

In Obsidian: Open folder as vault → choose the path above.
The code folders (`tractor_teensy/`, `tractor_rpi/`, etc.) are siblings — untouched.

---

## Final Folder Structure

```
obsidian_vault/
├── 00-dashboard.md               ← single file: current focus, links, blockers
├── 01-plan/
│   └── milestones.md             ← single bulleted doc: all milestones
├── 02-testing/                   ← dated session notes + field test results
├── 03-design/
│   ├── 03-1-rtk-base/            ← Bridgeville RTK base station
│   ├── 03-2-radio-control/       ← handheld RC unit (Teensy 3.2 + NRF24)
│   └── 03-3-tractor/
│       ├── 03-3-1-rpi5/
│       ├── 03-3-2-teensy-4-1/    ← pinout PDF, pin reference, ethernet datalogger
│       ├── 03-3-3-usb-hub/
│       ├── 03-3-4-ibt2/          ← current wiring only (Gen2 → 05-future-ideas)
│       ├── 03-3-5-jrk-g2/
│       ├── 03-3-6-nrf24/
│       ├── 03-3-7-oak-camera/
│       ├── 03-3-8-andon-light/
│       ├── 03-3-9-relay-board/
│       └── 03-3-10-frame-chassis/
├── 04-reference/                 ← cross-cutting: laptop setup, git workflow,
│                                    Ubuntu notes, loose PDFs, ZeroTier setup
└── 05-future-ideas/
    └── IBT-2_Next_Generation.md
```

---

## What Goes Where

### 00-dashboard.md
Single file at vault root. Contains:
- Current milestone focus
- Active blockers
- Quick links to frequently-used notes
- Recent test dates

### 01-plan/milestones.md
One document. Bulleted list of milestones with status:
- A — Radio manual control (steering + transmission) ← IN PROGRESS
- B — Web teleoperation (browser controls + Oak camera)
- C — GPS RTK onboard (position + heading, RTCM live)
- D — Pure Pursuit navigation (waypoint following)

Expand to individual milestone notes only if a milestone grows complex enough to need it.

### 02-testing/
Dated session notes — what was tested, what happened, measurements, serial output, conclusions.
File naming: `YYYYMMDD-topic.md` e.g. `20260518-steering-pid.md`
This replaces both `Daily Notes/` and `Field Testing/` from the old vault.

### 03-design/ — Component folders
**Philosophy: everything about a component lives in its folder.**
Each component folder contains:
- Architecture / design intent notes
- Wiring diagrams (described in Markdown or as images)
- CAD prompts (for 3D printed parts)
- Datasheets / pinout PDFs
- Known-good settings
- Firmware version notes

The pinout PDF for Teensy 4.1 lives in `03-3-2-teensy-4-1/`.
The IBT-2 current wiring lives in `03-3-4-ibt2/`.
No need to split between "design" and "reference" within a component.

### 04-reference/
Cross-cutting material not tied to any single component:
- Laptop setup guide (fresh Ubuntu install → dev environment)
- Git / GitHub workflow notes
- PlatformIO general notes
- ZeroTier network setup
- SSH configuration
- Ubuntu tips for this machine
- Loose PDFs not associated with a specific component

### 05-future-ideas/
Intentionally deferred work. If it's not being acted on in the current milestone, it lives here.
- `IBT-2_Next_Generation.md` ← Gen2 wiring with R_EN/L_EN/IS pins

---

## Content Migration Map

| Current location | New location | Action |
|-----------------|--------------|--------|
| `Daily Notes/` | `02-testing/` | Move + rename files |
| `Field Testing/` | `02-testing/` | Move |
| `Testing/` (old dated folders) | `02-testing/` | Move |
| `Electronics/Teensy 4.1/` | `03-3-2-teensy-4-1/` | Move |
| `Electronics/NRF24_Radio_Configuration.md` | `03-3-6-nrf24/` | Move |
| `Electronics/Teensy41_Ethernet_Datalogger.md` | `03-3-2-teensy-4-1/` | Move |
| `Electronics/IBT-2_Next_Generation.md` | `05-future-ideas/` | Move |
| `Electronics/USB Hub Big-7/` | `03-3-3-usb-hub/` | Move |
| `Steering/` | `03-3-4-ibt2/` + `03-3-2-teensy-4-1/` | Split by content |
| `JRK G2/` | `03-3-5-jrk-g2/` | Move |
| `GPS/` | `03-3-1-rpi5/` or `03-1-rtk-base/` | Move by topic |
| `GPS_BaseStation/` | `03-1-rtk-base/` | Move |
| `Oak-Camera/` | `03-3-7-oak-camera/` | Move |
| `PurePursuit/` | `03-3-1-rpi5/` | Move (RPi runs navigation) |
| `Planning/McMaster Carr Orders/` | `04-reference/` | Move |
| `Planning/Bob's Architecture/` | `03-design/` | Move |
| `Andon Light Options/` | `05-future-ideas/` | Move |
| `References/` (PDFs) | `04-reference/` | Move |
| `Dimensions/` | `03-3-10-frame-chassis/` | Move |
| `attachments/` | component folders or `04-reference/` | Sort by association |
| `Hardware2/` | `04-reference/` | Move |
| `Relay Board - 4 ports/` | `03-3-9-relay-board/` | Move |
| `16 Ch Servo Board/` | `04-reference/` or `05-future-ideas/` | Move |

---

## Migration Approach

**Do not do this all at once.** Gradual migration works fine in Obsidian:

1. Create the new folder structure in `obsidian_vault/`
2. Start all new notes in the new structure immediately
3. Move old content over as you naturally revisit it
4. Old vault (`claude-robot-project/`) stays intact until migration is complete

---

## Obsidian Plugins to Install

| Plugin | Purpose |
|--------|---------|
| Linter (Victor Tao) | Auto-formats YAML frontmatter, dates |
| Dataview (Michael Brenan) | Live queries for dashboard |
| Smart Connections (Brian Petro) | Semantic search across notes |

**Linter ignore list** — critical. Add all code folders so Linter never touches source files:
```
.git
.pio
.obsidian
BridgevilleRTKBase
radiocontrol_nrf24radio
tractor_rpi
tractor_teensy
hardware
```

---

## Notes

