<#
.SYNOPSIS
Downloads ESP32 data through the RTK base station and collects it on Windows.

.EXAMPLE
.\collect_rtkbase_esp32_20260724.ps1 `
  -RunId 20260724_172543 `
  -SiteName 62_Collins_Dr

.EXAMPLE
.\collect_rtkbase_esp32_20260724.ps1 `
  -RunId 20260724_172543 `
  -SiteName 62_Collins_Dr `
  -ExistingBaseFileName esp32_data_20260724_175633.csv

.DESCRIPTION
Run this script on the Windows planning laptop. It connects to the RTK base
station over SSH, runs the production ESP32 downloader in download_delete mode,
copies the resulting CSV into the matching site-run directory, and verifies
the transfer with SHA-256.

The ESP32 source file is deleted only after the base station downloader reports
a successful local save. The saved base-station CSV is retained by default as
a recovery copy. If the named base-station CSV already exists, the script
collects it without running download_delete again.
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
    [string]$BaseHost = '192.168.193.88',

    [ValidateNotNullOrEmpty()]
    [string]$BaseUser = 'al',

    [ValidateNotNullOrEmpty()]
    [string]$DestinationRoot = (Join-Path $HOME 'Documents\field_plans'),

    [ValidatePattern('^esp32_data_\d{8}_\d{6}(?:_rtkbase)?\.csv$')]
    [string]$ExistingBaseFileName,

    [switch]$RemoveBaseCopyAfterVerify
)

$ErrorActionPreference = 'Stop'

foreach ($command in @('ssh', 'scp')) {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) {
        throw "Required OpenSSH command is unavailable: $command"
    }
}

$remote = "${BaseUser}@${BaseHost}"
$destination = Join-Path $DestinationRoot "$SiteName\runs\$RunId"
$downloader = (
    "/home/$BaseUser/tractor2025/RTKBase/Bridgeville/" +
    "esp32_downloader_20260623.py"
)
$espName = if ($ExistingBaseFileName) {
    $ExistingBaseFileName
} else {
    "esp32_data_${RunId}_rtkbase.csv"
}
$remoteEsp = "/home/$BaseUser/esp32_data/$espName"
$localEsp = Join-Path $destination $espName

Write-Output "Checking RTK base connection: $remote"
& ssh $remote 'hostname'
if ($LASTEXITCODE -ne 0) {
    throw "SSH connection failed: $remote"
}

& ssh $remote "test -f '$downloader'"
if ($LASTEXITCODE -ne 0) {
    throw "ESP32 downloader was not found: $downloader"
}

& ssh $remote "test -f '$remoteEsp'"
$remoteFileAlreadyExists = ($LASTEXITCODE -eq 0)

if ($ExistingBaseFileName -and -not $remoteFileAlreadyExists) {
    throw "Requested existing base-station file was not found: $remoteEsp"
}

if ($remoteFileAlreadyExists) {
    Write-Output (
        "Base-station file already exists; skipping ESP32 download_delete: " +
        $remoteEsp
    )
} else {
    Write-Output 'Downloading ESP32 data to the RTK base station...'
    & ssh $remote "mkdir -p '/home/$BaseUser/esp32_data' && python3 '$downloader' download_delete '$remoteEsp'"
    if ($LASTEXITCODE -ne 0) {
        throw 'ESP32 download_delete failed; no laptop transfer was attempted.'
    }
}

& ssh $remote "test -s '$remoteEsp'"
if ($LASTEXITCODE -ne 0) {
    throw "Downloaded base-station CSV is missing or empty: $remoteEsp"
}

$remoteHashOutput = & ssh $remote "sha256sum '$remoteEsp'"
if ($LASTEXITCODE -ne 0 -or -not $remoteHashOutput) {
    throw "Could not calculate the base-station SHA-256: $remoteEsp"
}
$remoteHash = (($remoteHashOutput -split '\s+')[0]).ToUpperInvariant()

New-Item -ItemType Directory -Force -Path $destination | Out-Null

Write-Output 'Copying ESP32 CSV to the Windows run directory...'
& scp "${remote}:$remoteEsp" $destination
if ($LASTEXITCODE -ne 0) {
    throw "Failed to copy $remoteEsp"
}

if (-not (Test-Path -LiteralPath $localEsp -PathType Leaf)) {
    throw "Expected laptop file is missing: $localEsp"
}

$localHash = (Get-FileHash -LiteralPath $localEsp -Algorithm SHA256).Hash
if ($localHash -ne $remoteHash) {
    throw "SHA-256 verification failed: base=$remoteHash laptop=$localHash"
}

$espRows = @(Import-Csv -LiteralPath $localEsp).Count
$summaryPath = Join-Path $destination 'rtkbase_esp32_collection.json'
$summary = [ordered]@{
    run_id                     = $RunId
    site_name                  = $SiteName
    base_station               = $remote
    collected_utc              = [DateTime]::UtcNow.ToString('o')
    remote_file                = $remoteEsp
    local_file                 = $espName
    data_rows                  = $espRows
    sha256                     = $localHash
    reused_existing_base_file  = $remoteFileAlreadyExists
    base_copy_removed          = [bool]$RemoveBaseCopyAfterVerify
}
$summary | ConvertTo-Json | Set-Content -LiteralPath $summaryPath -Encoding UTF8

if ($RemoveBaseCopyAfterVerify) {
    & ssh $remote "rm -- '$remoteEsp'"
    if ($LASTEXITCODE -ne 0) {
        throw (
            'Laptop copy verified, but removal of the base-station recovery ' +
            "copy failed: $remoteEsp"
        )
    }
}

$manifestPath = Join-Path $destination 'file_hashes.csv'
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
Write-Output 'RTK base ESP32 collection complete.'
Write-Output "Destination        : $destination"
Write-Output "ESP32 data rows    : $espRows"
Write-Output "Verified SHA-256   : $localHash"
Write-Output "Base recovery copy : $(-not $RemoveBaseCopyAfterVerify)"
Write-Output "Summary            : $summaryPath"
