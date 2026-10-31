# Copies the robot's logs to this laptop so they can be checked.
#   1. Connect the laptop to the Control Hub's Wi-Fi.
#   2. From the repo folder:   .\tools\pull_robot_logs.ps1          (add -Clear to delete them from the hub afterwards)
# Files land in robot_logs\<date_time>\ :
#   BioBuzzLogs\*.csv       - (from /sdcard/BioBuzzLogs on the hub) diag_teleop_*, events_teleop_*, auto_*, systemcheck_*, vision_*, shots_*
#   robotControllerLog.txt  - the Robot Controller app's own log (crashes, configuration errors)
param([switch]$Clear, [string]$Address = "192.168.43.1:5555")

$adb = (Get-Command adb -ErrorAction SilentlyContinue).Source
if (-not $adb) { $adb = Join-Path $env:LOCALAPPDATA "Android\Sdk\platform-tools\adb.exe" }
if (-not (Test-Path $adb)) { Write-Error "adb not found. Install Android platform-tools (see README_2.md)."; exit 1 }

& $adb connect $Address | Out-Host
$devices = (& $adb devices) -match ("^" + [regex]::Escape($Address) + "\s+device$")
if (-not $devices) { Write-Error "No Control Hub connected. Is the laptop on the Control Hub Wi-Fi?"; exit 1 }
function hub { & $adb -s $Address @args }                           # always the Control Hub, even if other devices are listed

$repo = Split-Path $PSScriptRoot -Parent
$dest = Join-Path $repo ("robot_logs\" + (Get-Date -Format "yyyyMMdd_HHmmss"))
New-Item -ItemType Directory -Force $dest | Out-Null

hub pull /sdcard/BioBuzzLogs $dest | Out-Host                  # AppUtil.ROOT_FOLDER is /sdcard (not /sdcard/FIRST)
hub pull /sdcard/biobuzz_arm.txt $dest 2>$null | Out-Host        # saved flower-arm calibration (BioBuzz Arm Setup)
hub pull /sdcard/biobuzz_pinpoint.txt $dest 2>$null | Out-Host   # saved Pinpoint pod setup
hub pull /sdcard/biobuzz_teleop.txt $dest 2>$null | Out-Host     # saved alliance + SHOT SPOT
hub pull /sdcard/robotControllerLog.txt $dest 2>$null | Out-Host
hub pull /sdcard/FIRST/matchlogs $dest 2>$null | Out-Host

$files = Get-ChildItem $dest -Recurse -File
"`nCopied $($files.Count) file(s) to $dest"
$files | Sort-Object LastWriteTime | Select-Object Name, @{n = 'KB'; e = { [math]::Round($_.Length / 1KB, 1) } }, LastWriteTime | Format-Table -AutoSize

if ($Clear -and $files.Count -gt 0) {
    hub shell "rm -f /sdcard/BioBuzzLogs/*.csv"
    "Deleted the CSV logs from the Control Hub."
}
"Disconnect the laptop from the Control Hub Wi-Fi so the Driver Hub can connect."

