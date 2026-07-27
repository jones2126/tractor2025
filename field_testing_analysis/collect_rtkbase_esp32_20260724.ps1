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
Run this script on the Windows analysis computer. It connects to the RTK base
station over SSH, runs the production ESP32 downloader in download_delete mode,
copies the resulting CSV into the matching site-run directory, and verifies
the transfer with SHA-256.

The ESP32 source is reset only after the base-station downloader successfully
saves a recovery copy. PowerShell confirmation is required before initiating
download_delete. The base-station recovery copy is retained by default.
#>

[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'High')]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^\d{8}_\d{6}$')]
    [string]$RunId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
    [string]$SiteName,

    [ValidatePattern('^[A-Za-z0-9.:-]+$')]
    [string]$BaseHost = '192.168.193.88',

    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
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
    throw "Requested base-station file was not found: $remoteEsp"
}

if ($remoteFileAlreadyExists) {
    Write-Output (
        'Base-station file already exists; skipping ESP32 download_delete: ' +
        $remoteEsp
    )
} else {
    $action = (
        'save the ESP32 CSV on the RTK base and reset the ESP32 source log'
    )
    if (-not $PSCmdlet.ShouldProcess($remoteEsp, $action)) {
        Write-Output 'ESP32 collection cancelled; no data was changed.'
        return
    }
    Write-Output 'Downloading ESP32 data to the RTK base station...'
    & ssh $remote (
        "mkdir -p '/home/$BaseUser/esp32_data' && " +
        "python3 '$downloader' download_delete '$remoteEsp'"
    )
    if ($LASTEXITCODE -ne 0) {
        throw 'ESP32 download_delete failed; no laptop transfer was attempted.'
    }
}

& ssh $remote "test -s '$remoteEsp'"
if ($LASTEXITCODE -ne 0) {
    throw "Base-station CSV is missing or empty: $remoteEsp"
}

$remoteHashOutput = & ssh $remote "sha256sum -- '$remoteEsp'"
if ($LASTEXITCODE -ne 0 -or -not $remoteHashOutput) {
    throw "Could not calculate base-station SHA-256: $remoteEsp"
}
$remoteHash = (($remoteHashOutput -split '\s+')[0]).ToUpperInvariant()
if ($remoteHash -notmatch '^[0-9A-F]{64}$') {
    throw "Invalid base-station SHA-256 response: $remoteHashOutput"
}

New-Item -ItemType Directory -Force -Path $destination | Out-Null
$localReused = $false
if (Test-Path -LiteralPath $localEsp -PathType Leaf) {
    $localHash = (
        Get-FileHash -LiteralPath $localEsp -Algorithm SHA256
    ).Hash
    if ($localHash -ne $remoteHash) {
        throw (
            'Existing laptop ESP32 CSV differs from the base-station copy; ' +
            "it was not overwritten: $localEsp"
        )
    }
    $localReused = $true
    Write-Output 'Reusing verified existing laptop ESP32 CSV.'
} else {
    $partial = "$localEsp.partial-$PID"
    try {
        Write-Output 'Copying ESP32 CSV to the Windows run directory...'
        & scp "${remote}:$remoteEsp" $partial
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to copy $remoteEsp"
        }
        $localHash = (
            Get-FileHash -LiteralPath $partial -Algorithm SHA256
        ).Hash
        if ($localHash -ne $remoteHash) {
            throw (
                "SHA-256 verification failed: " +
                "base=$remoteHash laptop=$localHash"
            )
        }
        Move-Item -LiteralPath $partial -Destination $localEsp
    } finally {
        if (Test-Path -LiteralPath $partial) {
            Remove-Item -LiteralPath $partial -Force
        }
    }
}

$baseCopyRemoved = $false
if ($RemoveBaseCopyAfterVerify) {
    if ($PSCmdlet.ShouldProcess(
        $remoteEsp,
        'remove the verified RTK-base recovery copy'
    )) {
        & ssh $remote "rm -- '$remoteEsp'"
        if ($LASTEXITCODE -ne 0) {
            throw (
                'Laptop copy verified, but removal of the base-station ' +
                "recovery copy failed: $remoteEsp"
            )
        }
        $baseCopyRemoved = $true
    }
}

$espRows = @(Import-Csv -LiteralPath $localEsp).Count
$summaryPath = Join-Path $destination 'rtkbase_esp32_collection.json'
$summary = [ordered]@{
    run_id                    = $RunId
    site_name                 = $SiteName
    base_station              = $remote
    collected_utc             = [DateTime]::UtcNow.ToString('o')
    remote_file               = $remoteEsp
    local_file                = $espName
    data_rows                 = $espRows
    sha256                    = $remoteHash
    local_file_reused         = $localReused
    reused_existing_base_file = $remoteFileAlreadyExists
    base_copy_removal_requested = [bool]$RemoveBaseCopyAfterVerify
    base_copy_removed         = $baseCopyRemoved
}
$summary |
    ConvertTo-Json |
    Set-Content -LiteralPath $summaryPath -Encoding UTF8

$manifestPath = Join-Path $destination 'file_hashes.csv'
$hashRows = Get-ChildItem -LiteralPath $destination -Recurse -File |
    Where-Object { $_.FullName -ne $manifestPath } |
    Sort-Object FullName |
    ForEach-Object {
        [PSCustomObject]@{
            RelativePath = (
                $_.FullName.Substring($destination.Length).TrimStart('\')
            )
            LengthBytes  = $_.Length
            SHA256       = (
                Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256
            ).Hash
        }
    }
$hashRows |
    Export-Csv -LiteralPath $manifestPath -NoTypeInformation

Write-Output ''
Write-Output 'RTK base ESP32 collection complete.'
Write-Output "Destination        : $destination"
Write-Output "ESP32 data rows    : $espRows"
Write-Output "Verified SHA-256   : $remoteHash"
Write-Output "Base recovery copy : $(-not $baseCopyRemoved)"
Write-Output "Summary            : $summaryPath"
