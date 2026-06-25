# Firmware Version Notes

## Robot firmware 3.2.82 source rollback build

- File: `robot-code/.pio/build/esp32dev/wrobot_firmware_3.2.82.bin`
- Source behavior baseline: robot motion source restored to the 3.2.73 behavior from the Codex patch log
- New reported version: `3.2.82`

This is a source rollback, not a patched binary. The tracking and motion adapter
changes introduced after 3.2.73 were removed from source, then the firmware was
rebuilt as a new version so the original 3.2.73 version remains untouched.
