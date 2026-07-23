param(
    [int]$Port = 1237
)

$ErrorActionPreference = "Stop"
$webotsHome = if ($env:WEBOTS_HOME) { $env:WEBOTS_HOME } else { "C:\Program Files\Webots" }
$candidates = @(
    (Join-Path $webotsHome "msys64\mingw64\bin\webots.exe"),
    (Join-Path $webotsHome "webots.exe")
)
$webotsExe = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $webotsExe) {
    throw "Webots was not found. Set WEBOTS_HOME to the Windows Webots installation directory."
}

$world = Join-Path $PSScriptRoot "worlds\legacy_hexapod_ros2_bridge_test.wbt"
Write-Host "Starting Windows Webots on TCP port $Port"
Write-Host "World: $world"
& $webotsExe "--port=$Port" "--mode=realtime" $world
