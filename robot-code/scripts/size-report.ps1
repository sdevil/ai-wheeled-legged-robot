$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectRoot ".pio\build\esp32dev"
$versionSource = Join-Path $projectRoot "src\WebController.cpp"
$versionText = Get-Content -Raw -Encoding UTF8 -LiteralPath $versionSource
if ($versionText -notmatch 'ROBOT_FIRMWARE_VERSION\[\]\s*=\s*"([^"]+)"') {
  throw "ROBOT_FIRMWARE_VERSION not found in WebController.cpp."
}
$firmwareVersion = $matches[1]
$firmware = Join-Path $buildDir "wrobot_firmware_$firmwareVersion.bin"
$elf = Join-Path $buildDir "wrobot_firmware_$firmwareVersion.elf"
$webBundle = Join-Path $projectRoot "src\generated\WebUiBundle.h"
$sizeTool = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-xtensa-esp32\bin\xtensa-esp32-elf-size.exe"

if (!(Test-Path $firmware)) {
  throw "wrobot_firmware_$firmwareVersion.bin not found. Run PlatformIO build first."
}

$firmwareInfo = Get-Item $firmware
Write-Host "Firmware bin: $($firmwareInfo.Length) bytes"

if ((Test-Path $elf) -and (Test-Path $sizeTool)) {
  Write-Host "Sections:"
  & $sizeTool -A $elf | ForEach-Object {
    if ($_ -match '^\.(iram0\.text|dram0\.data|dram0\.bss|flash\.rodata|flash\.text)\s+(\d+)\s+') {
      '{0,-16} {1,10} bytes' -f ".$($matches[1])", [int]$matches[2]
    }
  }
}

if (Test-Path $webBundle) {
  $content = Get-Content -Raw -Encoding UTF8 -LiteralPath $webBundle
  $gz = [regex]::Match($content, 'WEB_UI_APP_JS_GZ_LEN = (\d+)')
  $raw = [regex]::Match($content, 'WEB_UI_APP_JS_RAW_LEN = (\d+)')
  if ($gz.Success) { Write-Host "Web UI JS gzip: $($gz.Groups[1].Value) bytes" }
  if ($raw.Success) { Write-Host "Web UI JS raw:  $($raw.Groups[1].Value) bytes" }
}
