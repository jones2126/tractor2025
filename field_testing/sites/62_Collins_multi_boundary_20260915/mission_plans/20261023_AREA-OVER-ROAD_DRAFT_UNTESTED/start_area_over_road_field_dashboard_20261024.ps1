$ErrorActionPreference = "Stop"
$package = "field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED"
$remote = "cd /home/al/tractor2025 && git pull --ff-only origin main && python3 $package/verify_area_over_road_field_package_20261024.py && bash $package/start_area_over_road_field_dashboard_20261024.sh"

Write-Host "Updating and verifying the AREA-OVER-ROAD package on Tractor01..."
Write-Host "Blades must remain off. Keep the phone in PAUSE until the dashboard says READY FOR GUARDED AUTO."
& ssh -t al@192.168.193.76 $remote
if ($LASTEXITCODE -ne 0) {
    throw "Tractor01 update, verification, or dashboard session exited with status $LASTEXITCODE"
}
