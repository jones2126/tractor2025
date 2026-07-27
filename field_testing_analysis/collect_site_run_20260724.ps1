<#
.SYNOPSIS
Downloads and verifies one completed Pure Pursuit field run.

.EXAMPLE
.\collect_site_run_20260724.ps1 `
  -RunId 20260724_172543 `
  -SiteName 62_Collins_Dr

.DESCRIPTION
Run this script on the Windows analysis computer. It copies the Pure Pursuit
log, field-test logger CSV, and the site mission package present on tractor01
at collection time. Remote and local SHA-256 values are compared for both logs,
and every mission-package file is compared against a remote hash manifest.

Existing matching files are reused. Existing files that differ from tractor01
cause the script to stop instead of overwriting or merging data.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^\d{8}_\d{6}$')]
    [string]$RunId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
    [string]$SiteName,

    [ValidatePattern('^[A-Za-z0-9.:-]+$')]
    [string]$TractorHost = '192.168.193.76',

    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
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
$localPursuit = Join-Path $destination $pursuitName
$localField = Join-Path $destination $fieldName
$localMission = Join-Path $destination $SiteName

function Get-RemoteSha256 {
    param([Parameter(Mandatory = $true)][string]$RemotePath)

    $hashOutput = & ssh $remote "sha256sum -- '$RemotePath'"
    if ($LASTEXITCODE -ne 0 -or -not $hashOutput) {
        throw "Could not calculate remote SHA-256: $RemotePath"
    }
    $hash = (($hashOutput -split '\s+')[0]).ToUpperInvariant()
    if ($hash -notmatch '^[0-9A-F]{64}$') {
        throw "Invalid remote SHA-256 response for ${RemotePath}: $hashOutput"
    }
    return $hash
}

function Get-RemoteMissionManifest {
    $manifestOutput = & ssh $remote (
        "cd '$remoteMission' && " +
        "find . -type f -print0 | sort -z | xargs -0 -r sha256sum"
    )
    if ($LASTEXITCODE -ne 0) {
        throw "Could not calculate remote mission manifest: $remoteMission"
    }

    $rows = @()
    foreach ($line in @($manifestOutput)) {
        if ($line -notmatch '^([0-9a-fA-F]{64})\s+\*?(.+)$') {
            throw "Could not parse remote mission hash line: $line"
        }
        $rows += [PSCustomObject]@{
            RelativePath = ($Matches[2] -replace '\\', '/')
            SHA256       = $Matches[1].ToUpperInvariant()
        }
    }
    if ($rows.Count -eq 0) {
        throw "Remote mission package contains no files: $remoteMission"
    }
    return @($rows | Sort-Object RelativePath)
}

function Get-LocalMissionManifest {
    param([Parameter(Mandatory = $true)][string]$Root)

    $rootPath = (Resolve-Path -LiteralPath $Root).Path.TrimEnd('\')
    $rows = Get-ChildItem -LiteralPath $rootPath -Recurse -File |
        ForEach-Object {
            $relative = $_.FullName.Substring($rootPath.Length).TrimStart('\')
            [PSCustomObject]@{
                RelativePath = './' + ($relative -replace '\\', '/')
                SHA256       = (
                    Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256
                ).Hash
            }
        }
    return @($rows | Sort-Object RelativePath)
}

function Assert-MissionManifestsMatch {
    param(
        [Parameter(Mandatory = $true)][object[]]$Expected,
        [Parameter(Mandatory = $true)][object[]]$Actual
    )

    $expectedByPath = @{}
    foreach ($row in $Expected) {
        $expectedByPath[$row.RelativePath] = $row.SHA256
    }
    $actualByPath = @{}
    foreach ($row in $Actual) {
        $actualByPath[$row.RelativePath] = $row.SHA256
    }

    $differences = @()
    foreach ($path in $expectedByPath.Keys) {
        if (-not $actualByPath.ContainsKey($path)) {
            $differences += "missing locally: $path"
        } elseif ($actualByPath[$path] -ne $expectedByPath[$path]) {
            $differences += "hash mismatch: $path"
        }
    }
    foreach ($path in $actualByPath.Keys) {
        if (-not $expectedByPath.ContainsKey($path)) {
            $differences += "unexpected local file: $path"
        }
    }
    if ($differences.Count -gt 0) {
        throw (
            "Mission package does not match tractor01:`n  " +
            ($differences -join "`n  ")
        )
    }
}

function Get-ManifestDigest {
    param([Parameter(Mandatory = $true)][object[]]$Manifest)

    $text = (
        $Manifest |
            Sort-Object RelativePath |
            ForEach-Object { "$($_.SHA256)  $($_.RelativePath)" }
    ) -join "`n"
    $bytes = [Text.Encoding]::UTF8.GetBytes($text)
    $sha = [Security.Cryptography.SHA256]::Create()
    try {
        return (
            [BitConverter]::ToString($sha.ComputeHash($bytes)) -replace '-', ''
        )
    } finally {
        $sha.Dispose()
    }
}

function Copy-VerifiedRemoteFile {
    param(
        [Parameter(Mandatory = $true)][string]$RemotePath,
        [Parameter(Mandatory = $true)][string]$LocalPath,
        [Parameter(Mandatory = $true)][string]$ExpectedHash,
        [Parameter(Mandatory = $true)][string]$Label
    )

    if (Test-Path -LiteralPath $LocalPath -PathType Leaf) {
        $existingHash = (
            Get-FileHash -LiteralPath $LocalPath -Algorithm SHA256
        ).Hash
        if ($existingHash -ne $ExpectedHash) {
            throw (
                "Existing $Label differs from tractor01; it was not " +
                "overwritten: $LocalPath"
            )
        }
        Write-Host "Reusing verified existing $Label."
        return $true
    }

    $partial = "$LocalPath.partial-$PID"
    try {
        Write-Host "Downloading $Label..."
        & scp "${remote}:$RemotePath" $partial | Out-Host
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to download $RemotePath"
        }
        $localHash = (
            Get-FileHash -LiteralPath $partial -Algorithm SHA256
        ).Hash
        if ($localHash -ne $ExpectedHash) {
            throw (
                "SHA-256 verification failed for ${Label}: " +
                "tractor=$ExpectedHash laptop=$localHash"
            )
        }
        Move-Item -LiteralPath $partial -Destination $LocalPath
        return $false
    } finally {
        if (Test-Path -LiteralPath $partial) {
            Remove-Item -LiteralPath $partial -Force
        }
    }
}

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

$remotePursuitHash = Get-RemoteSha256 -RemotePath $remotePursuit
$remoteFieldHash = Get-RemoteSha256 -RemotePath $remoteField
$remoteMissionManifest = @()
if (-not $SkipMissionPackage) {
    $remoteMissionManifest = @(Get-RemoteMissionManifest)
}

New-Item -ItemType Directory -Force -Path $destination | Out-Null
$pursuitReused = Copy-VerifiedRemoteFile `
    -RemotePath $remotePursuit `
    -LocalPath $localPursuit `
    -ExpectedHash $remotePursuitHash `
    -Label 'Pure Pursuit log'
$fieldReused = Copy-VerifiedRemoteFile `
    -RemotePath $remoteField `
    -LocalPath $localField `
    -ExpectedHash $remoteFieldHash `
    -Label 'field-test logger CSV'

$missionReused = $false
$missionManifestDigest = $null
if (-not $SkipMissionPackage) {
    if (Test-Path -LiteralPath $localMission -PathType Container) {
        $localManifest = @(Get-LocalMissionManifest -Root $localMission)
        Assert-MissionManifestsMatch `
            -Expected $remoteMissionManifest `
            -Actual $localManifest
        $missionReused = $true
        Write-Output 'Reusing verified existing mission package.'
    } else {
        $temporaryParent = Join-Path $destination ".mission-collect-$PID"
        try {
            New-Item -ItemType Directory -Force -Path $temporaryParent |
                Out-Null
            Write-Output 'Downloading mission package...'
            & scp -r "${remote}:$remoteMission" $temporaryParent
            if ($LASTEXITCODE -ne 0) {
                throw "Failed to download $remoteMission"
            }
            $temporaryMission = Join-Path $temporaryParent $SiteName
            if (-not (
                Test-Path -LiteralPath $temporaryMission -PathType Container
            )) {
                throw "Downloaded mission directory is missing: $temporaryMission"
            }
            $localManifest = @(
                Get-LocalMissionManifest -Root $temporaryMission
            )
            Assert-MissionManifestsMatch `
                -Expected $remoteMissionManifest `
                -Actual $localManifest
            Move-Item `
                -LiteralPath $temporaryMission `
                -Destination $localMission
        } finally {
            if (Test-Path -LiteralPath $temporaryParent) {
                Remove-Item -LiteralPath $temporaryParent -Recurse -Force
            }
        }
    }
    $missionManifestDigest = Get-ManifestDigest `
        -Manifest $remoteMissionManifest
}

$pursuitRows = @(Import-Csv -LiteralPath $localPursuit).Count
$fieldRows = @(Import-Csv -LiteralPath $localField).Count
$manifestPath = Join-Path $destination 'file_hashes.csv'
$summaryPath = Join-Path $destination 'collection_summary.json'
$summary = [ordered]@{
    run_id                     = $RunId
    site_name                  = $SiteName
    tractor                    = $remote
    collected_utc              = [DateTime]::UtcNow.ToString('o')
    destination                = $destination
    pursuit_log                = $pursuitName
    pursuit_data_rows          = $pursuitRows
    pursuit_sha256             = $remotePursuitHash
    pursuit_reused             = [bool]$pursuitReused
    field_log                  = $fieldName
    field_log_data_rows        = $fieldRows
    field_log_sha256           = $remoteFieldHash
    field_log_reused           = [bool]$fieldReused
    mission_package_copied     = -not $SkipMissionPackage
    mission_package_reused     = [bool]$missionReused
    mission_manifest_sha256    = $missionManifestDigest
    mission_package_note       = (
        'Snapshot of the site mission directory present on tractor01 at ' +
        'collection time; collect immediately after the run.'
    )
    checksum_manifest          = 'file_hashes.csv'
}
$summary |
    ConvertTo-Json |
    Set-Content -LiteralPath $summaryPath -Encoding UTF8

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
Write-Output 'Tractor run collection complete.'
Write-Output "Destination       : $destination"
Write-Output "Pure Pursuit rows : $pursuitRows"
Write-Output "Field logger rows : $fieldRows"
Write-Output "Hash manifest     : $manifestPath"
Write-Output "Summary           : $summaryPath"
