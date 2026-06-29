# Firmware Version Notes

## Stable release 2026-06-29

First stable public baseline for this repository:

- Robot mainboard firmware: `3.2.111`
- Robot firmware file: `robot-code/.pio/build/esp32dev/wrobot_firmware_3.2.111.bin`
- MaixCam application firmware: `3.2.29`
- MaixCam app source: `maixcam-code/`
- Git tag: `stable-2026-06-29`

This stable point keeps the current Web UI, WiFi/captive portal, OTA/settings,
gamepad switching, MaixCam video preview, object-label scan/lock UI, camera
pitch tracking, and MotionCore integration together as one tested baseline.

Notes:

- Runtime WiFi credentials are not stored in source control.
- The default gamepad MAC is intentionally blank; set the controller MAC from
  the Web UI settings page.
- Generated robot OTA binaries are ignored by Git. Rebuild from source when a
  binary is needed.

## Robot firmware 3.2.82 source rollback build

- File: `robot-code/.pio/build/esp32dev/wrobot_firmware_3.2.82.bin`
- Source behavior baseline: robot motion source restored to the 3.2.73 behavior from the Codex patch log
- New reported version: `3.2.82`

This is a source rollback, not a patched binary. The tracking and motion adapter
changes introduced after 3.2.73 were removed from source, then the firmware was
rebuilt as a new version so the original 3.2.73 version remains untouched.
