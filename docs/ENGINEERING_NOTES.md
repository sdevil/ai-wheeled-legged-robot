# Engineering Notes

These notes capture hardware-specific lessons from the WRobot - sdevil Enhanced bring-up. Treat them as regression guards when changing firmware, Web UI, OTA, camera, or motion-core code.

## Status LEDs

- The RGB status LEDs are WS2812-style addressable LEDs on `RGB_PIN 21`.
- Do not drive WS2812 LEDs with hand-written GPIO bit-banging. That implementation was unreliable once WiFi, WebSocket, camera status polling, and real-time motion tasks were active. It could leave one green LED latched on even though firmware state said every RGB value was black.
- Use ESP32 RMT for RGB LED output. RMT provides hardware-timed pulses and reliably sends the black/off frame.
- It is safe to transmit more RGB pixels than are physically present. The firmware currently sends 8 pixels to ensure every LED on the chain receives off frames.
- Ready indication should flash green 6 times after self-check succeeds, then turn off.
- Low battery warning should be red breathing only when battery percent is below the configured low-battery threshold.
- GPIO13 is a discrete green control-mode indicator, not the RGB ready LED. In Web/WiFi mode it should stay off; in Gamepad/Bluetooth mode it should stay on.
- Keep `led` fields in `/api/diagnostics` while LED behavior is still being validated. They distinguish GPIO13 from RGB chain state.

## OTA Stability

- Recent OTA builds became stable after pausing motion-side interference and relying on the current Web UI upload flow. Do not reintroduce background operations during OTA without testing on battery and USB power.
- OTA must run only in maintenance mode.
- Motion and action controls must be disabled during OTA.
- The OTA minimum battery threshold is 30%, unless the user explicitly confirms stable external power.
- The ESP32 uses dual OTA application slots with rollback support. New firmware must call OTA confirmation after boot so a valid image is marked good.
- Web UI upload progress is local XHR upload progress; do not show stale firmware status text such as `Idle` during active upload.

## Web Control

- Web and Gamepad control must both enter the same `MotionCommand` path. Differences in behavior are usually bugs in input mapping or transport handling.
- WebSocket is preferred for control, but HTTP fallback must send non-zero drive commands too. A previous bug only sent stop commands through HTTP fallback, causing Web UI controls to appear dead when WebSocket failed.
- Do not let status polling or video refresh reset movement commands while the joystick is held.

## Camera Preview

- MJPEG streams can appear black in mobile browsers even when the camera server is alive. The Web UI should automatically reconnect the stream on failed load, stale image state, focus regain, or foreground resume.
- If video is absent, show an explicit "No Video Signal" state instead of a broken image.
- Camera firmware and robot firmware have independent version numbers.

## Motion Core

- Keep `MotionCommand` as the stable boundary. Web UI, Gamepad, Camera, UART, OTA, and settings code must not directly call integrated motion controller internals.
- Real-time sensor/control/motor tasks should stay on the motion core to avoid WiFi/Web workload interrupting motor output.
- When changing stand/sit behavior, preserve smooth transitions and avoid hard-coded compensation that depends on floor friction, battery voltage, payload, or mechanical wear.
- Sit-down must first crouch under balance, then retract gently. Do not add wheel power that can pitch the robot forward.
