# Project Structure

[中文](PROJECT_STRUCTURE.zh-CN.md) | English

## Ownership boundaries

| Path | Ownership |
| --- | --- |
| robot-code/src | ESP32 product code |
| robot-code/src/generated | Generated Web UI firmware assets; do not edit manually |
| robot-code/scripts/build_web_ui.py | PlatformIO pre-build hook that refreshes the embedded Web UI only when its sources changed |
| robot-code/lib/SCServo | Vendored FEETECH smart-servo library |
| robot-code/lib/GamepadController | Vendored Gamepad BLE support |
| robot-code/scripts | Build and size-report utilities |
| maixcam-code | MaixCam application code deployed together as one app |
| servo-code | One-purpose STS3032 UART setup bridge |
| web-ui-react/src | React control UI source |
| docs | Project documentation; never flashed to either device |

## Naming rules

- Python modules and functions use snake_case.
- C++ classes and modules use PascalCase; functions and variables follow the existing lower camel case.
- Generated artifacts live in a generated directory.
- Upstream vendor filenames remain unchanged so their origin and update path stay traceable.
- Experimental builds and local tools do not belong in production source directories.

## Consolidation decisions

The duplicate MaixCam autostart helper was removed because startup is handled by main.py. The unused legacy scripted-action engine and inactive diagnostic stages were removed from the ESP32 production source. Small modules with clear hardware ownership remain separate; combining Web, voltage, RGB, preferences, and serial parsing would make the code harder to test and review without reducing firmware size.
