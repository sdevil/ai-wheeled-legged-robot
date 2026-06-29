# ESP32 Mainboard Firmware

English | [??](README.zh-CN.md)

This PlatformIO project contains the real-time firmware for the ESP32-WROOM-32 mainboard. It owns balance control, FOC, smart servos, actions, safety, Web/WiFi control, gamepad input, settings, and OTA.

Gamepad mapping: RB stands, LB sits, D-pad up/down extends or retracts both legs while held, and D-pad left/right leans the body while held. Releasing the D-pad preserves the current leg height. The left stick drives at high speed, and the right stick controls camera pitch plus chassis yaw. Web and Gamepad movement share the same normalized motion-command path. In the Web UI, leg height is an absolute synchronized slider and body lean uses hold-to-lean buttons that return to neutral on release.

The RGB status LEDs stay off during startup, flash green rapidly three times after the battery, IMU, both smart servos, motion core, and control path pass readiness checks, and breathe red slowly only when battery percentage is 10% or lower. The discrete green GPIO13 LED stays on in Gamepad Bluetooth mode and stays off in WiFi/Web mode.

## Build and upload

Open this directory in VS Code with PlatformIO, then use Build or Upload, or run pio run and pio run --target upload.

Disconnect the MaixCam UART TX/RX wires before USB upload. The shared serial path can otherwise interrupt flashing with Serial data stream stopped or Failed to write to target RAM.

The board has 4 MB Flash. min_spiffs.csv provides two OTA application slots of about 1.875 MB each plus a small SPIFFS partition. Use scripts/size-report.ps1 to report current firmware and embedded UI size.

Before changing LED, OTA, Web control, camera preview, or motion-core integration code, read `../docs/ENGINEERING_NOTES.md`. It records hardware-specific stability lessons from bring-up, including the RMT-only WS2812 LED rule and the current OTA safety assumptions.
