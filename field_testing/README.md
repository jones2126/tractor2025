# Field testing

Reusable collection and analysis scripts live in `tools/`. Saved field-test
artifacts live in `sites/`.

See [`tools/README.md`](tools/README.md) for tool selection, repository-local
commands, required inputs, and generated outputs.

Run the Windows menu from the repository root:

```powershell
.\field_testing\tools\field_test_analysis_menu_20260726.ps1
```

The menu lists complete mission-package folders found under
`tractor_rpi/pure-pursuit/missions/`. Choose the site by number, then enter the
run from a numbered list of downloaded run directories. Each run is labeled
as ready for analysis, missing its mission package, or partially downloaded.
Choose `N` only when collecting a run that does not yet exist locally. The
selected mission-folder name is used consistently for the local run directory,
the tractor mission-package path, and analysis filenames.

The menu then offers:

1. Download RTK-base/ESP32 data.
2. Download tractor logs and the mission package.
3. Analyze the run and generate/open its interactive HTML map.

Files are stored under:

```text
%USERPROFILE%\Documents\field_plans\<site>\runs\<run-id>\
```

The analysis option requires Python 3, NumPy, and pandas:

```powershell
python -m pip install -r .\field_testing\tools\requirements_field_testing_analysis_20260726.txt
```

The analysis can also be run without the menu:

```powershell
python .\field_testing\tools\analyze_run_20260726.py `
  --site-name 62_Collins_Dr `
  --run-id 20260724_172543 `
  --open-map
```

The Python script writes the analysis JSON, embedded map-data JSON, and final
standalone UTF-8 HTML map directly into the selected run directory. Node.js is
not required.

The RTK-base option normally retains the verified base-station CSV as a
recovery copy. Starting a new ESP32 download uses `download_delete`, so the
menu asks for confirmation before the ESP32 source log is reset.
