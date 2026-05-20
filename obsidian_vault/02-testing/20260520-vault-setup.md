---
title: Vault Setup and Migration Planning
created: 2026-05-20
modified: 2026-05-20
type: log
status: active
tags:
  - type/log
  - status/active
  - subsystem/rpi
---

# 2026-05-20 — Vault Setup and Migration Planning

## What Was Done

### New Obsidian Vault Created
- Vault created at `tractor2025/obsidian_vault/` inside the GitHub repo
- Folder structure finalized and committed to GitHub
- `.gitignore` added (excludes workspace state, Smart Connections embeddings)

### 00-project-overview.md
Merged four overlapping reference files into one canonical overview:
- `_Home.md` (legacy vault dashboard)
- `README_tractor2026.md` (hardware reference)
- `Key Files.md` (production file paths — updated to current filenames)
- `CLAUDE.md` (hardware tables, known issues, radio modes)

### 04-reference/ — Three new files
- `laptop-setup-guide.md` — step-by-step Ubuntu dev environment setup built from shell history
- `laptop-setup.sh` — automated setup script
- `obsidian-vault-setup-guide.md` — vault structure, frontmatter schema, tag taxonomy, plugin config, daily workflow, CAD strategy, future upgrades

### IBT-2 Next Generation
- `Electronics/IBT-2_Next_Generation.md` — wiring plan for full 6-pin IBT-2 control (R_EN, L_EN, R_IS, L_IS) with 3.3V safety circuits
- `tractor_rpi/testing/teensy41_ibt2_gen2/` — Gen2 test firmware with stall detection and automatic fault recovery
- Not yet implemented on tractor

### Teensy 4.1 Pin Reference
- Confirmed: Ethernet add-on uses bottom pads only — no edge pin conflicts with NRF24 radio (pins 9–13)
- Corrected earlier wrong analysis that claimed pin 13 was consumed by Ethernet
- Created `Electronics/Teensy 4.1/Teensy41_Pin_Reference.md` and copied pinout PDF

### Vault Migration — Completed
All content migrated from legacy vault (`claude-robot-project/`) to new vault:
- Component notes → `03-design/03-3-tractor/` subfolders
- RTK base station → `03-1-rtk-base/`
- Reference PDFs, McMaster orders, Bob's Architecture → `04-reference/`
- Andon lights, IBT-2 Gen2 → `05-future-ideas/`
- Physical hardware notes → `03-3-10-frame-chassis/`
- 120 images → `attachments/` — all resolving correctly in Obsidian
- GPS component page written: `03-3-1-rpi5/GPS-ZED-F9P.md`

Legacy vault kept as backup — not deleted.

## Next Session
- Field test: transmission neutral calibration using `jrkG2_range_test`
- Steering PID tuning (currently kp=1.0 — jerky)
- Create CLAUDE.md at `~/tractor2025/` repo root for Claude Code context
