# September 23, 2026 Tractor01 mission replay

Open `replay_20260923_124933_master_manual_5hz.html` directly in a modern web
browser. It is a self-contained historical replay—not a simulation—and needs
no internet connection, web server, or connection to Tractor01.

The event search accepts terms such as `RTK`, `heading`, `radio loss`, `Pause`,
`Manual`, `Auto`, `stopped`, `resumed`, `reacquired`, or a mission phase name.
Previous/Next event follow the filtered list when a search is active.

Rebuild and validate it from the repository root with:

```powershell
python field_testing/tools/build_tractor01_mission_replay_20260923.py
```

The builder reads the two original CSV logs and the mission/audit files without
modifying them. It embeds their SHA-256 hashes, plus hashes of the template and
builder sources used to produce the replay.
