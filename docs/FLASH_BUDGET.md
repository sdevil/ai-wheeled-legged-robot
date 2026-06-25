# Flash and Memory Budget

[中文](FLASH_BUDGET.zh-CN.md) | English

## What Is Stored on the ESP32

Only compiled firmware content consumes the ESP32 application partition:

- C/C++ code and linked library code
- Constant strings and embedded HTML in `WebController.cpp`
- Static data explicitly referenced by the firmware

Repository documentation such as `README.md`, files under `docs/`, Word/PDF
documents, and source comments are not uploaded to the ESP32 by PlatformIO.

## Partition Layout

The firmware uses Arduino ESP32 `min_spiffs.csv`:

| Partition | Size | Purpose |
| --- | ---: | --- |
| App slot 0 | 1,966,080 bytes | Current or OTA firmware |
| App slot 1 | 1,966,080 bytes | Alternate OTA firmware |
| SPIFFS | 131,072 bytes | Optional small data/assets |
| NVS and system data | Reserved | Preferences and ESP32 metadata |

This preserves future OTA support while providing 50% more application space
than the default 1,310,720-byte app slot.

## Current Baseline

Current robot firmware measurement with OTA dual app slots, Web UI, WiFi, WebSocket, MaixCam communication, and Gamepad Bluetooth code enabled:

```text
Firmware bin:     1,400,048 bytes
App partition:    1,393,473 / 1,966,080 bytes (70.9%)
Remaining app:    approximately 559KB
RAM:              63,132 / 327,680 bytes (19.3%)
React Web UI JS:  148,153 bytes gzip / 476,501 bytes raw
```

The old embedded HTML page has been removed. Markdown docs, `docs/`, `node_modules/`, and `.pio/` are not pushed or flashed to the ESP32.

Local size report command:

```powershell
powershell -ExecutionPolicy Bypass -File robot-code\scripts\size-report.ps1
```

## Budget Policy

1. Keep release firmware below 70% of the app partition when practical.
2. Record Flash and RAM numbers for every major feature.
3. Reuse existing libraries before adding dependencies.
4. Keep AI models, images, audio, and large UI assets on MaixCam or an external
   agent unless the ESP32 must access them directly.
5. Give every optional feature a compile-time flag when its cost is meaningful.
6. Store only a compact control page on the ESP32; host rich dashboards on a
   companion device.
7. Replace verbose serial logging with build-level logging before release.
8. Review the linker map and largest symbols whenever growth exceeds 20KB.

## Optimization Priorities

The largest project-owned static object is now `WEB_UI_APP_JS_GZ`, approximately 146KB. Future optimization priority:

1. Keep actions, PID, and motion behavior on the new controller path and remove conflicting legacy logic.
2. Continue shrinking the Web UI: reduce MUI/MUI Icons usage and move toward lightweight CSS plus inline icons if needed.
3. Put optional features behind compile-time switches, such as WiFi-only, Gamepad-enabled, and maintenance/diagnostic builds.
4. Compile verbose serial logging by log level and disable detailed logs in release firmware.
5. Keep AI models, images, audio, and complex dashboards out of the ESP32 firmware.
6. Review `firmware.map` and largest symbols whenever a feature grows firmware by more than 20KB.

Do not compile out Gamepad Bluetooth by default yet, because the project still needs runtime switching between WiFi and controller operation. If space gets tighter, create separate `robot_wifi` and `robot_gamepad` firmware environments.