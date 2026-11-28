# Removes a per-user ScanCraft installation made by install.ps1, with its file association
# (.scancraft, .scancraftz). Keeps the "recordings" folder (raw scan streams) unless
# -RemoveRecordings is given. Settings and caches in %LOCALAPPDATA%\ScanCraft (laser calibration,
# AI models, settings.json, the working folders of opened bundles under "work") are kept unless
# -RemoveData is given; even then working folders are only removed after asking, since one may
# hold unsaved changes of a bundle (ScanCraft offers to recover them at its next start). With
# -Quiet nobody is asked: they are kept unless -RemoveWorkFolders is given as well. They are always
# kept while a ScanCraft (from any folder) is running.
param([switch]$RemoveRecordings, [switch]$RemoveData, [switch]$RemoveWorkFolders, [switch]$Quiet)
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

# The file association: the ProgIDs, and the extensions where they still point at them.
$classes = 'HKCU:\Software\Classes'
foreach ($type in @(@('.scancraft', 'ScanCraft.Project'), @('.scancraftz', 'ScanCraft.Bundle'))) {
    $ext = Join-Path $classes $type[0]
    Remove-Item (Join-Path $classes $type[1]) -Recurse -Force -ErrorAction SilentlyContinue
    if (Test-Path "$ext\OpenWithProgids") { Remove-ItemProperty -Path "$ext\OpenWithProgids" -Name $type[1] -ErrorAction SilentlyContinue }
    if ((Get-ItemProperty -Path $ext -Name '(default)' -ErrorAction SilentlyContinue).'(default)' -eq $type[1]) {
        Remove-Item $ext -Recurse -Force -ErrorAction SilentlyContinue
    }
}
try {
    Add-Type -Namespace ScanCraftSetup -Name Shell -MemberDefinition '[System.Runtime.InteropServices.DllImport("shell32.dll")] public static extern void SHChangeNotify(int eventId, uint flags, System.IntPtr item1, System.IntPtr item2);'
    [ScanCraftSetup.Shell]::SHChangeNotify(0x08000000, 0, [IntPtr]::Zero, [IntPtr]::Zero)
} catch { }

$keepRecordings = (Test-Path (Join-Path $target 'recordings')) -and -not $RemoveRecordings
Get-ChildItem $target -Force | Where-Object { -not ($keepRecordings -and $_.Name -eq 'recordings') -and $_.Name -ne 'uninstall.ps1' } |
    Remove-Item -Recurse -Force
if ($RemoveData) {
    $data = Join-Path $env:LOCALAPPDATA 'ScanCraft'
    $work = Join-Path $data 'work'
    $openBundles = (Test-Path $work) -and @(Get-ChildItem $work -Directory -ErrorAction SilentlyContinue).Count -gt 0
    # Never answered for the user: a quiet uninstall keeps them unless told otherwise. Never
    # while a ScanCraft started from elsewhere (a dev build, the release zip) may be using them.
    $otherRunning = [bool](Get-Process -Name scancraft -ErrorAction SilentlyContinue)
    $removeWork = -not $otherRunning -and ($RemoveWorkFolders -or ($openBundles -and -not $Quiet -and [System.Windows.Forms.MessageBox]::Show("$work holds the working folders of opened bundles, which may have unsaved changes.`n`nRemove them too?", 'Uninstall ScanCraft', 'YesNo', 'Question') -eq 'Yes'))
    if (($openBundles -or $otherRunning) -and -not $removeWork) {
        Get-ChildItem $data -Force -ErrorAction SilentlyContinue | Where-Object { $_.Name -ne 'work' } | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
    } else {
        Remove-Item $data -Recurse -Force -ErrorAction SilentlyContinue
    }
}

# The script cannot delete itself while running: remove it (and the folder if empty) afterwards.
$cleanup = "Start-Sleep -Seconds 2; Remove-Item -LiteralPath '$target\uninstall.ps1' -Force; " +
    "if (-not (Get-ChildItem -LiteralPath '$target' -Force)) { Remove-Item -LiteralPath '$target' -Force }"
Start-Process powershell.exe -WindowStyle Hidden -ArgumentList '-NoProfile', '-Command', $cleanup

if (-not $Quiet) {
    $note = if ($keepRecordings) { "`n`nYour recordings were kept in $target\recordings." } else { '' }
    [void][System.Windows.Forms.MessageBox]::Show("ScanCraft was removed.$note", 'Uninstall ScanCraft')
}
