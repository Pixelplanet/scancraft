# ScanCraft per-user installer (no administrator rights needed).
# Run by ScanCraft-Setup.exe, or directly: powershell -ExecutionPolicy Bypass -File install.ps1
# Installs to %LOCALAPPDATA%\Programs\ScanCraft, adds Start menu shortcuts and an entry under
# Settings > Apps so it can be uninstalled there.
param(
    [string]$Payload = (Join-Path $PSScriptRoot 'payload.zip'),
    [string]$Target = (Join-Path $env:LOCALAPPDATA 'Programs\ScanCraft'),
    [switch]$NoDesktopShortcut,
    [switch]$Quiet
)
$ErrorActionPreference = 'Stop'

function Show-Message($text, $title = 'ScanCraft Setup', $icon = 'Information') {
    if ($Quiet) { Write-Host $text; return }
    Add-Type -AssemblyName System.Windows.Forms
    [void][System.Windows.Forms.MessageBox]::Show($text, $title, 'OK', $icon)
}

try {
    if (-not (Test-Path $Payload)) { throw "payload.zip not found next to the installer ($Payload)." }

    $running = Get-Process -Name scancraft -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -and $_.Path.StartsWith($Target, [StringComparison]::OrdinalIgnoreCase) }
    if ($running) {
        throw "ScanCraft is running from $Target. Stop any scan with Stop, close ScanCraft and run Setup again."
    }

    # Keep user data that lives next to the program (recordings) when upgrading.
    if (Test-Path $Target) {
        Get-ChildItem $Target -Force | Where-Object { $_.Name -ne 'recordings' } |
            Remove-Item -Recurse -Force
    } else {
        New-Item -ItemType Directory -Force $Target | Out-Null
    }
    Expand-Archive -Path $Payload -DestinationPath $Target -Force
    # The zip holds one top-level folder; flatten it.
    $inner = Get-ChildItem $Target -Directory | Where-Object { Test-Path (Join-Path $_.FullName 'scancraft.exe') } | Select-Object -First 1
    if ($inner) {
        Get-ChildItem $inner.FullName -Force | Move-Item -Destination $Target -Force
        Remove-Item $inner.FullName -Recurse -Force
    }
    Copy-Item (Join-Path $PSScriptRoot 'uninstall.ps1') $Target -Force

    $exe = Join-Path $Target 'scancraft.exe'
    if (-not (Test-Path $exe)) { throw "scancraft.exe missing after unpacking." }
    $version = (Get-Content (Join-Path $Target 'VERSION.txt') -ErrorAction SilentlyContinue | Select-Object -First 1)
    if (-not $version) { $version = '0.0.0' }

    $shell = New-Object -ComObject WScript.Shell
    $menu = Join-Path ([Environment]::GetFolderPath('Programs')) 'ScanCraft'
    New-Item -ItemType Directory -Force $menu | Out-Null
    function New-Shortcut($path, $targetPath, $arguments, $description) {
        $s = $shell.CreateShortcut($path)
        $s.TargetPath = $targetPath
        $s.Arguments = $arguments
        $s.WorkingDirectory = $Target
        $s.IconLocation = "$exe,0"
        $s.Description = $description
        $s.Save()
    }
    New-Shortcut (Join-Path $menu 'ScanCraft.lnk') $exe '' 'Open 3D scanner software'
    New-Shortcut (Join-Path $menu 'ScanCraft (simulated scanner).lnk') $exe '--mock' 'ScanCraft with a built-in simulated scanner'
    New-Shortcut (Join-Path $menu 'ScanCraft (record raw streams).lnk') (Join-Path $Target 'ScanCraft (record).bat') '' 'ScanCraft, saving every raw scan stream to the recordings folder'
    New-Shortcut (Join-Path $menu 'Uninstall ScanCraft.lnk') "$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe" "-NoProfile -ExecutionPolicy Bypass -File `"$Target\uninstall.ps1`"" 'Remove ScanCraft'
    if (-not $NoDesktopShortcut) {
        New-Shortcut (Join-Path ([Environment]::GetFolderPath('Desktop')) 'ScanCraft.lnk') $exe '' 'Open 3D scanner software'
    }

    $key = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\ScanCraft'
    New-Item -Path $key -Force | Out-Null
    $size = [int]((Get-ChildItem $Target -Recurse -File | Measure-Object Length -Sum).Sum / 1KB)
    $values = @{
        DisplayName = 'ScanCraft'; DisplayVersion = $version; Publisher = 'ScanCraft contributors'
        InstallLocation = $Target; DisplayIcon = "$exe,0"
        UninstallString = "`"$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe`" -NoProfile -ExecutionPolicy Bypass -File `"$Target\uninstall.ps1`""
        URLInfoAbout = 'https://github.com/Pixelplanet/scancraft'
    }
    foreach ($k in $values.Keys) { Set-ItemProperty -Path $key -Name $k -Value $values[$k] }
    Set-ItemProperty -Path $key -Name EstimatedSize -Value $size -Type DWord
    Set-ItemProperty -Path $key -Name NoModify -Value 1 -Type DWord
    Set-ItemProperty -Path $key -Name NoRepair -Value 1 -Type DWord

    Show-Message "ScanCraft $version is installed in`n$Target`n`nStart it from the Start menu (ScanCraft). Close Revo Scan before connecting to a scanner."
} catch {
    Show-Message "Installation failed: $($_.Exception.Message)" 'ScanCraft Setup' 'Error'
    exit 1
}
