<#
.SYNOPSIS
Downloads one completed Pure Pursuit field run from the tractor RPi.

.EXAMPLE
.\collect_site_run_20260724.ps1 `
  -RunId 20260724_172543 `
  -SiteName 62_Collins_Dr

.DESCRIPTION
Copies the Pure Pursuit log, field-test logger CSV, and exact archived mission
package used by a run. It verifies that the expected files arrived and writes
SHA-256 and collection-summary files alongside them.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^\d{8}_\d{6}$')]
    [string]$RunId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
    [string]$SiteName,

    [ValidateNotNullOrEmpty()]
    [string]$TractorHost = '192.168.193.76',

    [ValidateNotNullOrEmpty()]
    [string]$TractorUser = 'al',

    [ValidateNotNullOrEmpty()]
    [string]$DestinationRoot = (Join-Path $HOME 'Documents\field_plans'),

    [switch]$SkipMissionPackage
)

$ErrorActionPreference = 'Stop'

foreach ($command in @('ssh', 'scp')) {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) {
        throw "Required OpenSSH command is unavailable: $command"
    }
}

$remote = "${TractorUser}@${TractorHost}"
$destination = Join-Path $DestinationRoot "$SiteName\runs\$RunId"
$pursuitName = "pursuit_log_$RunId.csv"
$fieldName = "field_test_$RunId.csv"
$remotePursuit = "/home/$TractorUser/repos/field-testing-data/$pursuitName"
$remoteField = "/home/$TractorUser/field_logs/$fieldName"
$remoteMission = (
    "/home/$TractorUser/tractor2025/tractor_rpi/pure-pursuit/" +
    "missions/$SiteName"
)

Write-Output "Checking tractor connection: $remote"
& ssh $remote 'hostname'
if ($LASTEXITCODE -ne 0) {
    throw "SSH connection failed: $remote"
}

$requiredRemote = @($remotePursuit, $remoteField)
if (-not $SkipMissionPackage) {
    $requiredRemote += $remoteMission
}

foreach ($remotePath in $requiredRemote) {
    & ssh $remote "test -e '$remotePath'"
    if ($LASTEXITCODE -ne 0) {
        throw "Required tractor file or directory was not found: $remotePath"
    }
}

New-Item -ItemType Directory -Force -Path $destination | Out-Null

Write-Output "Downloading Pure Pursuit log..."
& scp "${remote}:$remotePursuit" $destination
if ($LASTEXITCODE -ne 0) {
    throw "Failed to download $remotePursuit"
}

Write-Output "Downloading field-test logger CSV..."
& scp "${remote}:$remoteField" $destination
if ($LASTEXITCODE -ne 0) {
    throw "Failed to download $remoteField"
}

if (-not $SkipMissionPackage) {
    Write-Output "Downloading exact archived mission package..."
    & scp -r "${remote}:$remoteMission" $destination
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to download $remoteMission"
    }
}

$localPursuit = Join-Path $destination $pursuitName
$localField = Join-Path $destination $fieldName
foreach ($localPath in @($localPursuit, $localField)) {
    if (-not (Test-Path -LiteralPath $localPath -PathType Leaf)) {
        throw "Expected downloaded file is missing: $localPath"
    }
}

$pursuitRows = @(Import-Csv -LiteralPath $localPursuit).Count
$fieldRows = @(Import-Csv -LiteralPath $localField).Count

$manifestPath = Join-Path $destination 'file_hashes.csv'
$summary = [ordered]@{
    run_id                  = $RunId
    site_name               = $SiteName
    tractor                 = $remote
    collected_utc           = [DateTime]::UtcNow.ToString('o')
    destination             = $destination
    pursuit_log             = $pursuitName
    pursuit_data_rows       = $pursuitRows
    field_log               = $fieldName
    field_log_data_rows     = $fieldRows
    mission_package_copied  = -not $SkipMissionPackage
    checksum_manifest       = 'file_hashes.csv'
}
$summaryPath = Join-Path $destination 'collection_summary.json'
$summary | ConvertTo-Json | Set-Content -LiteralPath $summaryPath -Encoding UTF8

$hashRows = Get-ChildItem -LiteralPath $destination -Recurse -File |
    Where-Object { $_.FullName -ne $manifestPath } |
    Sort-Object FullName |
    ForEach-Object {
        [PSCustomObject]@{
            RelativePath = $_.FullName.Substring($destination.Length).TrimStart('\')
            LengthBytes  = $_.Length
            SHA256       = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
        }
    }
$hashRows | Export-Csv -LiteralPath $manifestPath -NoTypeInformation

Write-Output ''
Write-Output 'Collection complete.'
Write-Output "Destination       : $destination"
Write-Output "Pure Pursuit rows : $pursuitRows"
Write-Output "Field logger rows : $fieldRows"
Write-Output "Hash manifest     : $manifestPath"
Write-Output "Summary           : $summaryPath"
