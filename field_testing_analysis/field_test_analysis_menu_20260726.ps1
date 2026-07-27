<#
.SYNOPSIS
Interactive Windows menu for collecting and analyzing tractor field runs.

.EXAMPLE
.\field_test_analysis_menu_20260726.ps1

.EXAMPLE
.\field_test_analysis_menu_20260726.ps1 `
  -SiteName 62_Collins_Dr `
  -RunId 20260724_172543
#>

[CmdletBinding()]
param(
    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
    [string]$SiteName,

    [ValidatePattern('^\d{8}_\d{6}$')]
    [string]$RunId,

    [ValidateNotNullOrEmpty()]
    [string]$DestinationRoot = (Join-Path $HOME 'Documents\field_plans'),

    [ValidatePattern('^[A-Za-z0-9.:-]+$')]
    [string]$TractorHost = '192.168.193.76',

    [ValidatePattern('^[A-Za-z0-9.:-]+$')]
    [string]$BaseHost = '192.168.193.88'
)

$ErrorActionPreference = 'Stop'
$script:CurrentSiteName = $SiteName
$script:CurrentRunId = $RunId
$siteCollector = Join-Path $PSScriptRoot 'collect_site_run_20260724.ps1'
$baseCollector = Join-Path $PSScriptRoot 'collect_rtkbase_esp32_20260724.ps1'
$analyzer = Join-Path $PSScriptRoot 'analyze_run_20260726.py'
$template = Join-Path $PSScriptRoot 'tractor-path-map-template.html'
$repositoryRoot = Split-Path $PSScriptRoot -Parent
$missionsRoot = Join-Path (
    $repositoryRoot
) 'tractor_rpi\pure-pursuit\missions'

function Select-SiteName {
    param([string]$CurrentValue)

    if (-not (Test-Path -LiteralPath $missionsRoot -PathType Container)) {
        throw "Local missions directory was not found: $missionsRoot"
    }

    $siteDirectories = @(
        Get-ChildItem -LiteralPath $missionsRoot -Directory |
            Where-Object {
                $_.Name -match '^[A-Za-z0-9_.-]+$' -and
                (Test-Path -LiteralPath (
                    Join-Path $_.FullName "$($_.Name)_mission.txt"
                ) -PathType Leaf) -and
                (Test-Path -LiteralPath (
                    Join-Path $_.FullName "$($_.Name)_mission_audit.csv"
                ) -PathType Leaf) -and
                (Test-Path -LiteralPath (
                    Join-Path $_.FullName '01_boundary_final.csv'
                ) -PathType Leaf)
            } |
            Sort-Object Name
    )
    if ($siteDirectories.Count -eq 0) {
        throw (
            'No complete mission packages were found under: ' +
            $missionsRoot
        )
    }

    $defaultNumber = 1
    if ($CurrentValue) {
        for ($index = 0; $index -lt $siteDirectories.Count; $index++) {
            if ($siteDirectories[$index].Name -eq $CurrentValue) {
                $defaultNumber = $index + 1
                break
            }
        }
    }

    while ($true) {
        Write-Host ''
        Write-Host "Available mission sites in $missionsRoot"
        for ($index = 0; $index -lt $siteDirectories.Count; $index++) {
            $number = $index + 1
            Write-Host ("{0,2}. {1}" -f $number, $siteDirectories[$index].Name)
        }
        $selection = (
            Read-Host "Select site number [$defaultNumber]"
        ).Trim()
        if (-not $selection) {
            $selection = $defaultNumber.ToString()
        }
        $selectedNumber = 0
        if (
            [int]::TryParse($selection, [ref]$selectedNumber) -and
            $selectedNumber -ge 1 -and
            $selectedNumber -le $siteDirectories.Count
        ) {
            return $siteDirectories[$selectedNumber - 1].Name
        }
        Write-Host 'Enter one of the displayed site numbers.' `
            -ForegroundColor Yellow
    }
}

function Read-RequiredValue {
    param(
        [Parameter(Mandatory = $true)][string]$Prompt,
        [string]$CurrentValue,
        [Parameter(Mandatory = $true)][string]$Pattern,
        [Parameter(Mandatory = $true)][string]$ErrorMessage
    )

    while ($true) {
        $displayPrompt = if ($CurrentValue) {
            "$Prompt [$CurrentValue]"
        } else {
            $Prompt
        }
        $value = (Read-Host $displayPrompt).Trim()
        if (-not $value) {
            $value = $CurrentValue
        }
        if ($value -and $value -match $Pattern) {
            return $value
        }
        Write-Host $ErrorMessage -ForegroundColor Yellow
    }
}

function Select-RunId {
    param(
        [Parameter(Mandatory = $true)][string]$SelectedSiteName,
        [string]$CurrentValue
    )

    $runsRoot = Join-Path $DestinationRoot "$SelectedSiteName\runs"
    $runDirectories = @()
    if (Test-Path -LiteralPath $runsRoot -PathType Container) {
        $runDirectories = @(
            Get-ChildItem -LiteralPath $runsRoot -Directory |
                Where-Object { $_.Name -match '^\d{8}_\d{6}$' } |
                Sort-Object Name -Descending
        )
    }

    if ($runDirectories.Count -eq 0) {
        Write-Host ''
        Write-Host "No downloaded runs were found under $runsRoot"
        return Read-RequiredValue `
            -Prompt 'New run ID (YYYYMMDD_HHMMSS)' `
            -CurrentValue $CurrentValue `
            -Pattern '^\d{8}_\d{6}$' `
            -ErrorMessage 'Use the format YYYYMMDD_HHMMSS.'
    }

    $defaultNumber = 1
    if ($CurrentValue) {
        for ($index = 0; $index -lt $runDirectories.Count; $index++) {
            if ($runDirectories[$index].Name -eq $CurrentValue) {
                $defaultNumber = $index + 1
                break
            }
        }
    }

    while ($true) {
        Write-Host ''
        Write-Host "Available runs for $SelectedSiteName"
        for ($index = 0; $index -lt $runDirectories.Count; $index++) {
            $directory = $runDirectories[$index]
            $runId = $directory.Name
            $friendlyTime = [DateTime]::ParseExact(
                $runId,
                'yyyyMMdd_HHmmss',
                [Globalization.CultureInfo]::InvariantCulture
            ).ToString('yyyy-MM-dd HH:mm:ss')
            $hasPursuit = Test-Path -LiteralPath (
                Join-Path $directory.FullName "pursuit_log_$runId.csv"
            ) -PathType Leaf
            $hasField = Test-Path -LiteralPath (
                Join-Path $directory.FullName "field_test_$runId.csv"
            ) -PathType Leaf
            $hasMission = Test-Path -LiteralPath (
                Join-Path (
                    $directory.FullName
                ) "$SelectedSiteName\$($SelectedSiteName)_mission.txt"
            ) -PathType Leaf
            $status = if ($hasPursuit -and $hasField -and $hasMission) {
                'ready for analysis'
            } elseif ($hasPursuit -and $hasField) {
                'logs downloaded; mission package missing'
            } else {
                'partial download'
            }
            $number = $index + 1
            Write-Host (
                "{0,2}. {1}  ({2})  [{3}]" -f
                $number,
                $runId,
                $friendlyTime,
                $status
            )
        }
        Write-Host ' N. Enter a new run ID for collection'
        $selection = (
            Read-Host "Select run number [$defaultNumber]"
        ).Trim()
        if (-not $selection) {
            $selection = $defaultNumber.ToString()
        }
        if ($selection -match '^(n|new)$') {
            return Read-RequiredValue `
                -Prompt 'New run ID (YYYYMMDD_HHMMSS)' `
                -CurrentValue '' `
                -Pattern '^\d{8}_\d{6}$' `
                -ErrorMessage 'Use the format YYYYMMDD_HHMMSS.'
        }
        $selectedNumber = 0
        if (
            [int]::TryParse($selection, [ref]$selectedNumber) -and
            $selectedNumber -ge 1 -and
            $selectedNumber -le $runDirectories.Count
        ) {
            return $runDirectories[$selectedNumber - 1].Name
        }
        Write-Host 'Enter a displayed run number or N for a new run.' `
            -ForegroundColor Yellow
    }
}

function Read-YesNo {
    param(
        [Parameter(Mandatory = $true)][string]$Prompt,
        [bool]$Default = $false
    )

    $suffix = if ($Default) { '[Y/n]' } else { '[y/N]' }
    while ($true) {
        $answer = (Read-Host "$Prompt $suffix").Trim()
        if (-not $answer) {
            return $Default
        }
        switch -Regex ($answer) {
            '^(y|yes)$' { return $true }
            '^(n|no)$' { return $false }
            default {
                Write-Host 'Enter Y or N.' -ForegroundColor Yellow
            }
        }
    }
}

function Set-RunContext {
    $script:CurrentSiteName = Select-SiteName `
        -CurrentValue $script:CurrentSiteName
    $script:CurrentRunId = Select-RunId `
        -SelectedSiteName $script:CurrentSiteName `
        -CurrentValue $script:CurrentRunId `
}

function Get-PythonInvocation {
    $python = Get-Command python -ErrorAction SilentlyContinue
    if ($python) {
        return [PSCustomObject]@{
            Command = $python.Source
            Prefix  = @()
        }
    }
    $launcher = Get-Command py -ErrorAction SilentlyContinue
    if ($launcher) {
        return [PSCustomObject]@{
            Command = $launcher.Source
            Prefix  = @('-3')
        }
    }
    throw (
        'Python 3 was not found. Install Python or add python.exe to PATH.'
    )
}

function Invoke-TractorCollection {
    & $siteCollector `
        -RunId $script:CurrentRunId `
        -SiteName $script:CurrentSiteName `
        -TractorHost $TractorHost `
        -DestinationRoot $DestinationRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Tractor collection exited with code $LASTEXITCODE"
    }
}

function Invoke-BaseCollection {
    $existingName = (
        Read-Host (
            'Existing base filename, or press Enter to download/reset ESP32'
        )
    ).Trim()
    if (-not $existingName) {
        $approved = Read-YesNo -Prompt (
            'The ESP32 log will be saved on the RTK base, then reset. Continue?'
        )
        if (-not $approved) {
            Write-Host 'RTK-base collection cancelled.'
            return
        }
    }

    $removeRecovery = Read-YesNo -Prompt (
        'Remove the RTK-base recovery copy after laptop verification?'
    )
    $parameters = @{
        RunId          = $script:CurrentRunId
        SiteName       = $script:CurrentSiteName
        BaseHost       = $BaseHost
        DestinationRoot = $DestinationRoot
        Confirm        = $false
    }
    if ($existingName) {
        $parameters.ExistingBaseFileName = $existingName
    }
    if ($removeRecovery) {
        $parameters.RemoveBaseCopyAfterVerify = $true
    }
    & $baseCollector @parameters
    if ($LASTEXITCODE -ne 0) {
        throw "RTK-base collection exited with code $LASTEXITCODE"
    }
}

function Invoke-RunAnalysis {
    $python = Get-PythonInvocation
    $arguments = @()
    $arguments += $python.Prefix
    $arguments += @(
        $analyzer,
        '--site-name', $script:CurrentSiteName,
        '--run-id', $script:CurrentRunId,
        '--field-plans-root', $DestinationRoot,
        '--template', $template,
        '--open-map'
    )
    & $python.Command @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Run analysis exited with code $LASTEXITCODE"
    }
}

foreach ($requiredScript in @(
    $siteCollector,
    $baseCollector,
    $analyzer,
    $template
)) {
    if (-not (Test-Path -LiteralPath $requiredScript -PathType Leaf)) {
        throw "Required workflow file is missing: $requiredScript"
    }
}

Set-RunContext
$quit = $false
while (-not $quit) {
    $runDirectory = Join-Path $DestinationRoot (
        "$($script:CurrentSiteName)\runs\$($script:CurrentRunId)"
    )
    Write-Host ''
    Write-Host '============================================'
    Write-Host ' Tractor field-test collection and analysis'
    Write-Host '============================================'
    Write-Host "Site : $($script:CurrentSiteName)"
    Write-Host "Run  : $($script:CurrentRunId)"
    Write-Host "Files: $runDirectory"
    Write-Host ''
    Write-Host '1. Download RTK-base / ESP32 data'
    Write-Host '2. Download tractor logs and mission package'
    Write-Host '3. Analyze run and generate the HTML map'
    Write-Host 'C. Change site name or run ID'
    Write-Host 'Q. Quit'
    Write-Host ''

    $choice = (Read-Host 'Choose an option').Trim().ToUpperInvariant()
    try {
        switch ($choice) {
            '1' { Invoke-BaseCollection }
            '2' { Invoke-TractorCollection }
            '3' { Invoke-RunAnalysis }
            'C' { Set-RunContext }
            'Q' { $quit = $true }
            default {
                Write-Host 'Choose 1, 2, 3, C, or Q.' -ForegroundColor Yellow
            }
        }
    } catch {
        Write-Host ''
        Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    }

    if (-not $quit -and $choice -ne 'C') {
        Write-Host ''
        Read-Host 'Press Enter to return to the menu' | Out-Null
    }
}
