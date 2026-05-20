# Obsidian Vault Setup Guide — tractor2025
**Created:** 2026-05-20
**Status:** Finalized — ready to implement when convenient

---

## Vault Location

The Obsidian vault lives inside the tractor2025 GitHub repo:

```
/home/albert/tractor2025/obsidian_vault/
```

In Obsidian: **Open folder as vault** → choose the path above.
The code folders (`tractor_teensy/`, `tractor_rpi/`, etc.) are siblings — untouched.

---

## Folder Structure

```
obsidian_vault/
├── 00-project-overview.md        ← single file: hardware ref, key files, RPi connection
├── 01-plan/
│   └── milestones.md             ← single bulleted doc: all milestones
├── 02-testing/                   ← dated session notes (YYYYMMDD-topic.md)
├── 03-design/
│   ├── 03-1-rtk-base/
│   ├── 03-2-radio-control/
│   └── 03-3-tractor/
│       ├── 03-3-1-rpi5/
│       ├── 03-3-2-teensy-4-1/
│       ├── 03-3-3-usb-hub/
│       ├── 03-3-4-ibt2/
│       ├── 03-3-5-jrk-g2/
│       ├── 03-3-6-nrf24/
│       ├── 03-3-7-oak-camera/
│       ├── 03-3-8-andon-light/
│       ├── 03-3-9-relay-board/
│       └── 03-3-10-frame-chassis/
├── 04-reference/                 ← cross-cutting: laptop setup, git notes, loose PDFs
└── 05-future-ideas/
```

---

## What Goes Where

### 00-project-overview.md
Single file at vault root. Hardware tables, RPi connection, key production file paths, known issues.

### 01-plan/milestones.md
One document. Bulleted milestones with status. Expand to individual files only if a milestone grows complex enough to need it.

### 02-testing/
Dated session notes. File naming: `YYYYMMDD-topic.md` e.g. `20260518-steering-pid.md`
Captures: what was tested, what happened, measurements, serial output, conclusions, next steps.

### 03-design/ — Component folders
**Philosophy: everything about a component lives in its folder.**
Each folder contains: design intent, wiring notes, CAD prompts, datasheets, pinout PDFs, known-good settings, firmware version notes. No design/reference split within a component.

### 04-reference/
Cross-cutting material not tied to any single component: laptop setup, git workflow, PlatformIO notes, ZeroTier setup, Ubuntu tips, loose PDFs.

### 05-future-ideas/
Intentionally deferred work not in the current milestone.

---

## Frontmatter Schema

Every substantive note starts with YAML frontmatter. This makes notes queryable by Dataview and searchable by Smart Connections.

```yaml
---
title: Human-readable title
created: 2026-05-20
modified: 2026-05-20
type: <see table>
status: <active | blocked | done>
tags:
  - type/<value>
  - status/<value>
  - subsystem/<value>
---
```

**Hard rules:**
- ISO 8601 dates only: `YYYY-MM-DD`
- Block-style tag lists (always `tags:` on its own line, then `  - item`)
- Lowercase field names

| type value | When to use |
|-----------|-------------|
| `log` | Dated session note |
| `design` | Architecture, wiring, component notes |
| `reference` | Pinouts, commands, settings, datasheets |
| `decision` | Why X was chosen over Y |
| `milestone` | Outcome-focused project phase |
| `future-idea` | Deferred / not acting on now |

**Status lifecycle:** `active` → `blocked` → `done`
Use `blocked` when waiting on hardware, parts, or a prerequisite. Makes blocked items visible in Dataview queries.

---

## Tag Taxonomy

Tags use `/` nesting, lowercase, kebab-case. Five tags per note is plenty.

```
type/log          type/design        type/reference
type/decision     type/milestone     type/future-idea

status/active     status/blocked     status/done

subsystem/teensy      subsystem/rpi          subsystem/nrf24
subsystem/ibt2        subsystem/jrk-g2       subsystem/gps
subsystem/oak-camera  subsystem/rtk-base     subsystem/radio-control

phase/design      phase/build        phase/test
```

`subsystem/` tags are the most useful: querying `subsystem/teensy` returns every log, design note, and reference that touches the Teensy 4.1 — regardless of which folder it lives in.

---

## Plugins

Install via: **Settings → Community Plugins → Browse**

### Linter (by Victor Tao)
Auto-formats frontmatter and enforces schema discipline.

After installing, configure:

| Setting | Value |
|---------|-------|
| Lint on Save | ON |
| YAML Timestamp — Date Created Key | `created` |
| YAML Timestamp — Date Modified Key | `modified` |
| YAML Timestamp — Format | `YYYY-MM-DD` |

**Folders to ignore** (critical — Linter must never touch source files):
```
.git
.pio
.obsidian
BridgevilleRTKBase
radiocontrol_nrf24radio
tractor_rpi
tractor_teensy
hardware
build
install
log
node_modules
venv
.venv
```

### Dataview (by Michael Brenan)
Live queries in notes. Example — show all blocked items:
```dataview
TABLE status, tags
FROM "obsidian_vault"
WHERE status = "blocked"
SORT modified DESC
```

### Smart Connections (by Brian Petro)
Local AI-powered semantic search. Useful when you know you wrote something about steering smoothing but can't remember which note it's in. Works fully offline after initial embedding.

---

## CAD Artifact Strategy

**CAD prompts** (design intent, constraints, dimensions) live in the component folder in `03-design/`:
```
03-design/03-3-tractor/03-3-10-frame-chassis/ibt2-holder-cad-prompt.md
```

**Fusion 360 `.f3d` files** go in a `hardware/` folder at the repo root (not inside `obsidian_vault/`):
```
tractor2025/hardware/ibt2-motor-controller-holder.f3d
```

Add to `.gitattributes` at repo root:
```
*.f3d binary
```

Workflow: design in Fusion 360 → export `.f3d` to `hardware/` → write CAD prompt note in component folder → commit both together.

---

## Daily Workflow

**Starting a session:**
Create a note in `02-testing/` named `YYYYMMDD-topic.md`.
Write your intention first — one sentence stating what you're trying to accomplish.

**During the session:**
Document as you go:
- Exact commands run (copy from terminal)
- Measurements (voltages, pot values, serial output)
- What changed and why
- Anything surprising

**End of session:**
Fill in "Next session" before closing. This is what you read at the start of the next session.

**When a decision happens:**
If you chose X over Y for a non-obvious reason, capture it immediately as a note in the relevant component folder with `type: decision`. The decision is freshest now.

---

## Content Migration Map

| Current (claude-robot-project) | New (obsidian_vault) | Action |
|-------------------------------|---------------------|--------|
| `Daily Notes/` | `02-testing/` | Move + rename |
| `Field Testing/` | `02-testing/` | Move |
| `Testing/` (old dated) | `02-testing/` | Move |
| `Electronics/Teensy 4.1/` | `03-3-2-teensy-4-1/` | Move |
| `Electronics/NRF24_Radio_Configuration.md` | `03-3-6-nrf24/` | Move |
| `Electronics/Teensy41_Ethernet_Datalogger.md` | `03-3-2-teensy-4-1/` | Move |
| `Electronics/IBT-2_Next_Generation.md` | `05-future-ideas/` | Move |
| `Steering/` | `03-3-4-ibt2/` + `03-3-2-teensy-4-1/` | Split by content |
| `JRK G2/` | `03-3-5-jrk-g2/` | Move |
| `GPS/` | `03-3-1-rpi5/` or `03-1-rtk-base/` | Move by topic |
| `GPS_BaseStation/` | `03-1-rtk-base/` | Move |
| `Oak-Camera/` | `03-3-7-oak-camera/` | Move |
| `PurePursuit/` | `03-3-1-rpi5/` | Move |
| `Planning/McMaster Carr Orders/` | `04-reference/` | Move |
| `Planning/Bob's Architecture/` | `03-design/` | Move |
| `Andon Light Options/` | `05-future-ideas/` | Move |
| `References/` (PDFs) | `04-reference/` | Move |
| `Dimensions/` | `03-3-10-frame-chassis/` | Move |
| `attachments/` | component folders or `04-reference/` | Sort by association |
| `Hardware2/` | `04-reference/` | Move |
| `Relay Board - 4 ports/` | `03-3-9-relay-board/` | Move |
| `16 Ch Servo Board/` | `04-reference/` or `05-future-ideas/` | Move |

**Migration approach:** gradual is fine. Start all new notes in the new vault immediately. Move old content as you naturally revisit it. Keep the legacy vault intact until migration is complete.

---

## Future Upgrades

Install these only after the core system (Linter + Dataview + Smart Connections) is working well in daily use:

| Upgrade | Why |
|---------|-----|
| Templater plugin | Auto-fills dates and prompts for frontmatter fields at insert time |
| Git plugin for Obsidian | Commit from inside Obsidian without opening a terminal |
| Mermaid diagrams | Inline architecture diagrams; good for ROS node graphs, message flows |
| Smart Chat (Ollama) | Chat with your vault locally — "what did I decide about the steering algorithm?" |
| Dataview advanced queries | Aggregate test results, milestone velocity, blocked-item aging |

---

## Notes
