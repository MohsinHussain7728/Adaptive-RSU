# Removes everything installed by install_toolchain_windows.ps1.
# Unregistering the distro deletes its virtual disk (Ubuntu, ns-3, SUMO, venv).
# The research repository itself (and its results/) is NOT touched.

param(
    [string]$LabDir = "C:\ns3-uav-lab",
    [string]$DistroName = "ns3-uav-lab"
)

wsl.exe --terminate $DistroName 2>$null
wsl.exe --unregister $DistroName
if (Test-Path $LabDir) { Remove-Item -Recurse -Force $LabDir }
Write-Host "Toolchain removed ($DistroName, $LabDir)."
