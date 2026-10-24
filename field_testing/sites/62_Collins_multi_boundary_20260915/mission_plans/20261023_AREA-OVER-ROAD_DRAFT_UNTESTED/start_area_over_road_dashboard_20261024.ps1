$ErrorActionPreference = "Stop"
$key = Join-Path $env:USERPROFILE ".ssh\id_ed25519_tractor01"
$remote = "cd /home/al/tractor2025 && bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/start_area_over_road_dashboard_20261024.sh"

Write-Host "Opening the AREA-OVER-ROAD review-only dashboard on Tractor01..."
Write-Host "Live mission start is disabled."
& ssh -t -i $key al@192.168.193.76 $remote
if ($LASTEXITCODE -ne 0) {
    throw "Dashboard session exited with status $LASTEXITCODE"
}
