# ScanCraft per-user installer (no administrator rights needed).
# Run by ScanCraft-Setup.exe, or directly: powershell -ExecutionPolicy Bypass -File install.ps1
# Installs to %LOCALAPPDATA%\Programs\ScanCraft, adds Start menu shortcuts and an entry under
# Settings > Apps so it can be uninstalled there, and registers .scancraft project files and
# .scancraftz bundles for this user, so a double-click opens them (in the running ScanCraft as a
# new tab, if one runs).
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

    # File association, per user (HKCU\Software\Classes): ProgID with icon and open command, and
    # the extension pointing at it. uninstall.ps1 removes both.
    $classes = 'HKCU:\Software\Classes'
    function Register-FileType($extension, $progId, $description) {
        $prog = Join-Path $classes $progId
        New-Item -Path "$prog\DefaultIcon" -Force | Out-Null
        New-Item -Path "$prog\shell\open\command" -Force | Out-Null
        Set-ItemProperty -Path $prog -Name '(default)' -Value $description
        Set-ItemProperty -Path $prog -Name 'FriendlyTypeName' -Value $description
        Set-ItemProperty -Path "$prog\DefaultIcon" -Name '(default)' -Value "`"$exe`",0"
        Set-ItemProperty -Path "$prog\shell\open\command" -Name '(default)' -Value "`"$exe`" `"%1`""
        $ext = Join-Path $classes $extension
        New-Item -Path "$ext\OpenWithProgids" -Force | Out-Null
        Set-ItemProperty -Path $ext -Name '(default)' -Value $progId
        New-ItemProperty -Path "$ext\OpenWithProgids" -Name $progId -Value '' -PropertyType String -Force | Out-Null
    }
    Register-FileType '.scancraft' 'ScanCraft.Project' 'ScanCraft project'
    Register-FileType '.scancraftz' 'ScanCraft.Bundle' 'ScanCraft project bundle'
    # Tell Explorer, so the icons and the double-click work without signing out.
    try {
        Add-Type -Namespace ScanCraftSetup -Name Shell -MemberDefinition '[System.Runtime.InteropServices.DllImport("shell32.dll")] public static extern void SHChangeNotify(int eventId, uint flags, System.IntPtr item1, System.IntPtr item2);'
        [ScanCraftSetup.Shell]::SHChangeNotify(0x08000000, 0, [IntPtr]::Zero, [IntPtr]::Zero)
    } catch { }

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

    Show-Message "ScanCraft $version is installed in`n$Target`n`nStart it from the Start menu (ScanCraft), or double-click a .scancraft project or .scancraftz bundle. Close Revo Scan before connecting to a scanner."
} catch {
    Show-Message "Installation failed: $($_.Exception.Message)" 'ScanCraft Setup' 'Error'
    exit 1
}
