# Removes a per-user ScanCraft installation made by install.ps1. Keeps the "recordings" folder
# (raw scan streams) unless -RemoveRecordings is given. Settings and caches in
# %LOCALAPPDATA%\ScanCraft (laser calibration, AI models) are kept unless -RemoveData is given.
param([switch]$RemoveRecordings, [switch]$RemoveData, [switch]$Quiet)
$ErrorActionPreference = 'Stop'
$target = $PSScriptRoot
Add-Type -AssemblyName System.Windows.Forms

function Ask($text) {
    if ($Quiet) { return $true }
    return [System.Windows.Forms.MessageBox]::Show($text, 'Uninstall ScanCraft', 'YesNo', 'Question') -eq 'Yes'
}

if (-not (Test-Path (Join-Path $target 'scancraft.exe'))) {
    if (-not $Quiet) { [void][System.Windows.Forms.MessageBox]::Show("No ScanCraft installation in $target.", 'Uninstall ScanCraft') }
    exit 1
}
if (Get-Process -Name scancraft -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($target, [StringComparison]::OrdinalIgnoreCase) }) {
    if (-not $Quiet) { [void][System.Windows.Forms.MessageBox]::Show('ScanCraft is still running. Stop any scan, close ScanCraft and try again.', 'Uninstall ScanCraft') }
    exit 1
}
if (-not (Ask "Remove ScanCraft from`n$target ?")) { exit 0 }

$menu = Join-Path ([Environment]::GetFolderPath('Programs')) 'ScanCraft'
Remove-Item $menu -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item (Join-Path ([Environment]::GetFolderPath('Desktop')) 'ScanCraft.lnk') -Force -ErrorAction SilentlyContinue
Remove-Item 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\ScanCraft' -Recurse -Force -ErrorAction SilentlyContinue

$keepRecordings = (Test-Path (Join-Path $target 'recordings')) -and -not $RemoveRecordings
Get-ChildItem $target -Force | Where-Object { -not ($keepRecordings -and $_.Name -eq 'recordings') -and $_.Name -ne 'uninstall.ps1' } |
    Remove-Item -Recurse -Force
if ($RemoveData) { Remove-Item (Join-Path $env:LOCALAPPDATA 'ScanCraft') -Recurse -Force -ErrorAction SilentlyContinue }

# The script cannot delete itself while running: remove it (and the folder if empty) afterwards.
$cleanup = "Start-Sleep -Seconds 2; Remove-Item -LiteralPath '$target\uninstall.ps1' -Force; " +
    "if (-not (Get-ChildItem -LiteralPath '$target' -Force)) { Remove-Item -LiteralPath '$target' -Force }"
Start-Process powershell.exe -WindowStyle Hidden -ArgumentList '-NoProfile', '-Command', $cleanup

if (-not $Quiet) {
    $note = if ($keepRecordings) { "`n`nYour recordings were kept in $target\recordings." } else { '' }
    [void][System.Windows.Forms.MessageBox]::Show("ScanCraft was removed.$note", 'Uninstall ScanCraft')
}
