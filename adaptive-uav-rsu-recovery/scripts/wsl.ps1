# Runs a repository bash script inside the ns3-uav-lab WSL distro from Windows.
#   .\scripts\wsl.ps1 scripts/build.sh
#   .\scripts\wsl.ps1 scripts/run_sim.sh --mobility=static --strategy=reactive-gap
param(
    [Parameter(Mandatory = $true, Position = 0)][string]$Script,
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments
)
$DistroName = if ($env:NS3_UAV_DISTRO) { $env:NS3_UAV_DISTRO } else { "ns3-uav-lab" }
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$drive = $repoRoot.Substring(0, 1).ToLower()
$wslRepo = "/mnt/$drive" + ($repoRoot.Substring(2) -replace "\\", "/")
$argString = ""
if ($Arguments) {
    $argString = ($Arguments | ForEach-Object { "'" + ($_ -replace "'", "'\''") + "'" }) -join " "
}
# Windows editors may save CRLF; bash scripts must be LF (.gitattributes enforces LF in git).
$fixEol = "find scripts scratch -name '*.sh' -exec grep -l `$'\r' {} + 2>/dev/null | xargs -r sed -i 's/\r`$//'"
wsl.exe -d $DistroName --cd $wslRepo -- bash -lc "$fixEol; bash '$Script' $argString"
