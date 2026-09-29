# Installs the complete research toolchain (Ubuntu 24.04 + ns-3.47 + SUMO 1.18)
# into ONE self-contained folder so it can be removed later without leaving
# anything behind on the Windows system.
#
#   C:\ns3-uav-lab\
#       downloads\   Ubuntu root filesystem image
#       wsl\         WSL2 virtual disk (ext4.vhdx) holding Ubuntu, ns-3, SUMO, Python venv
#
# Usage (PowerShell):
#   .\scripts\setup\install_toolchain_windows.ps1
#   .\scripts\setup\install_toolchain_windows.ps1 -LabDir D:\my-lab -Jobs 8
#
# Removal: .\scripts\setup\remove_toolchain_windows.ps1

param(
    [string]$LabDir = "C:\ns3-uav-lab",
    [string]$DistroName = "ns3-uav-lab",
    [int]$Jobs = 8
)

$ErrorActionPreference = "Stop"
$rootfsUrl = "https://cloud-images.ubuntu.com/wsl/releases/24.04/current/ubuntu-noble-wsl-amd64-wsl.rootfs.tar.gz"
$rootfs = Join-Path $LabDir "downloads\ubuntu-noble-wsl-amd64.rootfs.tar.gz"

New-Item -ItemType Directory -Force (Join-Path $LabDir "downloads"), (Join-Path $LabDir "wsl") | Out-Null

if (-not (Test-Path $rootfs)) {
    Write-Host "Downloading Ubuntu 24.04 WSL root filesystem..."
    curl.exe -L --fail -o $rootfs $rootfsUrl
}

$existing = (wsl.exe --list --quiet) -replace "`0", "" | Where-Object { $_.Trim() -eq $DistroName }
if (-not $existing) {
    Write-Host "Importing WSL distro '$DistroName' into $LabDir\wsl ..."
    wsl.exe --import $DistroName (Join-Path $LabDir "wsl") $rootfs --version 2
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$drive = $repoRoot.Substring(0, 1).ToLower()
$wslRepo = "/mnt/$drive" + ($repoRoot.Substring(2) -replace "\\", "/")

Write-Host "Bootstrapping Ubuntu (apt packages, SUMO, Python venv, ns-3.47 build)..."
# tr strips CR characters in case the repo was checked out with Windows line endings.
wsl.exe -d $DistroName -u root -- bash -c "tr -d '\r' < '$wslRepo/scripts/setup/bootstrap_ubuntu.sh' | bash -s -- $Jobs"

Write-Host "Building the NetAnim viewer (Qt5)..."
wsl.exe -d $DistroName -u root -- bash -c "tr -d '\r' < '$wslRepo/scripts/setup/build_netanim.sh' | bash -s -- $Jobs"
