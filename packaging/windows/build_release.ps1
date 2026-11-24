# Builds the Windows release: a portable zip and ScanCraft-Setup-<version>.exe (per-user installer
# made with Windows' built-in IExpress; no extra tools needed).
#
#   powershell -ExecutionPolicy Bypass -File packaging\windows\build_release.ps1 -Out D:\scancraft-dist
#
# -Out       folder for the results (default: <repo>\target\dist). Keep it off small drives.
# -TargetDir CARGO_TARGET_DIR for the release build (default: <repo>\target\release-dist).
#            A full build needs several GB.
# -SkipBuild package the binaries already in -TargetDir.
param(
    [string]$Out,
    [string]$TargetDir,
    [switch]$SkipBuild
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if (-not $Out) { $Out = Join-Path $repo 'target\dist' }
if (-not $TargetDir) { $TargetDir = Join-Path $repo 'target\release-dist' }
$here = $PSScriptRoot

$version = (Select-String -Path (Join-Path $repo 'Cargo.toml') -Pattern '^version\s*=\s*"([^"]+)"' |
    Select-Object -First 1).Matches[0].Groups[1].Value
$date = Get-Date -Format 'yyyy-MM-dd'
$name = "scancraft-$version-windows-x64"
Write-Host "ScanCraft $version ($date) -> $Out"

$tools = @(
    @{ Package = 'scancraft-driver-revopoint'; Example = 'mode_check' },
    @{ Package = 'scancraft-driver-revopoint'; Example = 'live_scan' },
    @{ Package = 'scancraft-driver-revopoint'; Example = 'stream_probe' },
    @{ Package = 'scancraft-align'; Example = 'merge_ply' }
)
if (Test-Path (Join-Path $repo 'crates\scancraft-mesh\examples\solid_ply.rs')) {
    $tools += @{ Package = 'scancraft-mesh'; Example = 'solid_ply' }
}

if (-not $SkipBuild) {
    $env:CARGO_TARGET_DIR = $TargetDir
    $env:CARGO_INCREMENTAL = '0'
    # Static C runtime: the exe runs without the Visual C++ redistributable.
    $env:RUSTFLAGS = '-C target-feature=+crt-static'
    Push-Location $repo
    try {
        cargo build --release -p scancraft
        if ($LASTEXITCODE -ne 0) { throw 'cargo build of the app failed' }
        foreach ($t in $tools) {
            cargo build --release -p $t.Package --example $t.Example
            if ($LASTEXITCODE -ne 0) { throw "cargo build of $($t.Example) failed" }
        }
    } finally { Pop-Location }
}

$stage = Join-Path $Out $name
if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
New-Item -ItemType Directory -Force (Join-Path $stage 'tools') | Out-Null
Copy-Item (Join-Path $TargetDir 'release\scancraft.exe') $stage
foreach ($t in $tools) {
    Copy-Item (Join-Path $TargetDir "release\examples\$($t.Example).exe") (Join-Path $stage 'tools')
}
Copy-Item (Join-Path $here 'ScanCraft (record).bat') $stage
Copy-Item (Join-Path $repo 'apps\scancraft\ATTRIBUTION.md') $stage
Copy-Item (Join-Path $repo 'LICENSE-MIT'), (Join-Path $repo 'LICENSE-APACHE') $stage
(Get-Content (Join-Path $here 'README.txt') -Raw).Replace('@VERSION@', $version).Replace('@DATE@', $date) |
    Set-Content (Join-Path $stage 'README.txt') -Encoding utf8
Set-Content (Join-Path $stage 'VERSION.txt') "$version`n$date" -Encoding ascii

$zip = Join-Path $Out "$name.zip"
if (Test-Path $zip) { Remove-Item $zip -Force }
Compress-Archive -Path $stage -DestinationPath $zip -CompressionLevel Optimal

# Installer: IExpress packs payload.zip + install scripts into one self-extracting exe that runs
# install.cmd (per-user install, no elevation).
$work = Join-Path $Out 'installer-work'
if (Test-Path $work) { Remove-Item $work -Recurse -Force }
New-Item -ItemType Directory -Force $work | Out-Null
Copy-Item $zip (Join-Path $work 'payload.zip')
Copy-Item (Join-Path $here 'install.ps1'), (Join-Path $here 'uninstall.ps1') $work
Set-Content (Join-Path $work 'install.cmd') "@echo off`r`npowershell.exe -NoProfile -ExecutionPolicy Bypass -File `"%~dp0install.ps1`"`r`n" -Encoding ascii

$setup = Join-Path $Out "ScanCraft-Setup-$version.exe"
if (Test-Path $setup) { Remove-Item $setup -Force }
$files = 'payload.zip', 'install.ps1', 'uninstall.ps1', 'install.cmd'
$sed = @"
[Version]
Class=IEXPRESS
SEDVersion=3
[Options]
PackagePurpose=InstallApp
ShowInstallProgramWindow=1
HideExtractAnimation=0
UseLongFileName=1
InsideCompressed=0
CAB_FixedSize=0
CAB_ResvCodeSigning=0
RebootMode=N
InstallPrompt=Install ScanCraft $version for the current user?
DisplayLicense=
FinishMessage=
TargetName=$setup
FriendlyName=ScanCraft $version Setup
AppLaunched=cmd.exe /c install.cmd
PostInstallCmd=<None>
AdminQuietInstCmd=
UserQuietInstCmd=
SourceFiles=SourceFiles
$(for ($i = 0; $i -lt $files.Count; $i++) { "FILE$i=`"$($files[$i])`"`r`n" })[SourceFiles]
SourceFiles0=$work\
[SourceFiles0]
$(for ($i = 0; $i -lt $files.Count; $i++) { "%FILE$i%=`r`n" })
"@
$sedPath = Join-Path $work 'scancraft.sed'
Set-Content $sedPath $sed -Encoding ascii
$p = Start-Process "$env:SystemRoot\System32\iexpress.exe" -ArgumentList '/N', '/Q', $sedPath -Wait -PassThru
if (-not (Test-Path $setup)) { throw "IExpress did not produce $setup (exit $($p.ExitCode))" }
Remove-Item $work -Recurse -Force

foreach ($f in $zip, $setup) {
    $h = (Get-FileHash $f -Algorithm SHA256).Hash
    Write-Host ("{0}  {1:N1} MB  SHA-256 {2}" -f (Split-Path $f -Leaf), ((Get-Item $f).Length / 1MB), $h)
}
