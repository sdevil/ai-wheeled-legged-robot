# WiFi Web Control

[中文](WEB_CONTROL.zh-CN.md) | English

## Connection

```text
SSID: sdevil-WRobot
Password: wrobot123
URL: http://192.168.8.1
Local name: http://wrobot.local
```

The ESP32 keeps its access point active while it may also join home WiFi in the background. Connecting to the robot AP triggers a captive portal. You can also use http://wrobot.local; use http://192.168.8.1 if the client does not support mDNS.

## Available Controls

- Stand and sit
- Reset body pose adjustments
- Forward, backward, left, and right movement
- Speed limit from 10 to 100
- Immediate stop
- Robot, Gamepad, balance angle, battery percent, battery voltage, maintenance mode, and OTA state
- Basic settings for robot name, control mode, home WiFi, camera stream URL, and resolution
- OTA firmware selection, upload progress, and rollback controls in Advanced Maintenance
- Standing, forward, backward, left, and right jumps

## WebSocket Real-Time Control

The Web UI automatically connects to `ws://<robot-host>:81/ws`. Joystick, gimbal, and action commands prefer WebSocket and fall back to ordered HTTP commands while the socket is unavailable. The Network status card shows the active transport. A second controller that cannot obtain the single real-time socket falls back after a 1.5-second connection timeout instead of hanging.

Client message formats:

```json
{"type":"drive","x":25,"y":60}
{"type":"gimbal","yaw":20,"pitchDelta":-2}
{"type":"action","name":"stand"}
```

Pointer movement sends an immediate command and a held joystick refreshes its current value every 32ms. Release sends zero immediately. The ESP32 clears motion after 1 second without a refresh and also clears motion as soon as the WebSocket disconnects. The wider watchdog prevents HTTP fallback jitter from pulsing the motors while preserving explicit stop behavior. Status and settings continue to use HTTP.

Drive and gimbal controls start in the high-speed position. Web and gamepad inputs use the same normalized axis range and enter the same `MotionCommand` path.

Hold the up/down buttons between the joysticks to extend or retract both legs; releasing preserves the current height. Hold the left/right buttons below the drive joystick to lean the body, and release to return to neutral. These controls refresh every 100ms and are cleared after 350ms without a refresh, on WebSocket disconnect, or when the page is hidden.

## Camera Preview

The ESP32 does not proxy video. MaixCam serves these endpoints on port 8080:

- /stream.mjpg: low-latency continuous MJPEG
- /snapshot.jpg: single-frame fallback
- 160x120 remote control: 40ms encode interval, JPEG quality 62, minimum bandwidth and latency
- 320x240 low latency: 45ms encode interval, JPEG quality 80, target approximately 22 FPS
- 640x480 (default): 75ms encode interval, JPEG quality 86, target approximately 13 FPS
- 1280x720: 140ms encode interval, JPEG quality 82, target approximately 7 FPS

Streaming uses non-blocking partial writes and keeps only the newest queued frame when the network slows down. This prevents stale frames from accumulating into increasing latency. Actual FPS still depends on detector load and WiFi conditions.

AI processing uses the selected camera frame as its source. Each detector is resized to its model-defined input tensor size, with detections mapped back to the source frame. Tracking is category-based: face, animal, or ping-pong ball. Unsupported arbitrary-object tracking is intentionally disabled.

After a changed resolution is saved, the ESP32 sends it over UART. MaixCam persists the setting and automatically restarts only when the value changed, then reconnects to home WiFi and resumes video at the new resolution.

## WiFi Privacy

- Keep WIFI_SSID and WIFI_PASSWORD in maixcam-code/config.py as empty strings. Never store live credentials in source control.
- Save home WiFi once in the Web UI; the ESP32 stores it locally and synchronizes it to MaixCam.
- MaixCam stores live credentials on the device at `/root/wrobot_wifi.json`.
- MaixCam writes that file atomically with owner-only permissions and redacts the password from WiFi command logs.
- That device file is outside the repository and should not be exported, bundled, or committed.
- The ESP32 web UI may keep the camera `SSID`, but the real password should be treated as runtime device data rather than repository content.

## OTA Safety Model

### OTA update flow

1. Build the mainboard firmware with PlatformIO.
2. Use `robot-code/.pio/build/esp32dev/wrobot_firmware_<version>.bin` as
   the OTA file, for example `wrobot_firmware_3.2.51.bin`.
3. Open the robot Web UI, go to **Settings**, and enter **Maintenance Mode**.
4. Confirm the robot is sitting and disabled.
5. Confirm battery is at least `30%`, or confirm USB/external stable power.
6. Select the generated `wrobot_firmware_<version>.bin` in
   **Advanced Maintenance** and upload.
7. Do not close or refresh the page while OTA is running. The UI stays locked
   until the ESP32 reboots, reports a new boot ID, and becomes reachable again.
8. After reconnect, verify the firmware version at the bottom of the settings
   panel.

- OTA is only allowed in maintenance mode
- Firmware selection and every OTA control remain disabled until maintenance mode is active; the maintenance controls are grouped in a visually separate settings section
- The robot must already be sitting and disabled before upload
- The displayed battery percentage must be at or above the configured threshold, default `30%`
- Drive and action commands are locked during OTA
- The browser sends the exact firmware byte size before upload. The ESP32 uses that size for progress reporting and rejects incomplete images before rebooting.
- If the HTTP response is interrupted by reboot, the UI verifies a changed `boot_id` instead of immediately reporting a connection failure.
- After upload reaches 100%, the UI remains locked until a new device boot ID is observed and the robot is reachable again
- After the new boot ID is confirmed, the browser reloads the whole page so it cannot continue running the old in-memory Web UI bundle
- `/api/status` exposes the mainboard and MaixCam version/build metadata; the bottom of the settings panel displays only `Firmware <version>` and `Camera <version>`
- The camera version is reported to the mainboard over UART and remains `unknown` until both updated firmwares are running
- The firmware uses dual OTA application slots, so uploads target the inactive slot
- If power is lost during upload, the previous bootable firmware remains intact
- On first boot after update, the firmware confirms the new image; if the first boot fails, the ESP32 rollback path can return to the previous image
- The settings API returns only whether a WiFi password exists; it never returns the stored password

## HTTP API

| Method | Endpoint | Description |
| --- | --- | --- |
| `GET` | `/` | Embedded control page |
| `GET` | `/api/status` | Robot status JSON |
| `GET` | `/api/settings` | Current settings JSON |
| `POST` | `/api/settings` | Save basic settings |
| `POST` | `/api/drive?x=-100..100&y=-100..100` | Drive command |
| `POST` | `/api/legs/height?direction=-1..1` | Hold/release both-leg height direction |
| `POST` | `/api/legs/lean?percent=-100..100` | Hold/release body lean |
| `POST` | `/api/action?name=stand` | Stand up |
| `POST` | `/api/action?name=sit` | Sit down |
| `POST` | `/api/action?name=reset` | Reset pose adjustments |
| `POST` | `/api/action?name=maintenance_on` | Enter maintenance mode |
| `POST` | `/api/action?name=maintenance_off` | Exit maintenance mode |
| `POST` | `/api/ota/upload` | Upload firmware |
| `POST` | `/api/ota/rollback` | Roll back to the previous image |

Example status response:

```json
{"enabled":true,"sitting":false,"gamepad":true,"maintenance":false,"ota_running":false,"angle":1.5,"battery_voltage":8.12,"battery_percent":83,"ota_progress":0,"ota_message":"idle","clients":1}
```

## Extension Guidance

Add new UI actions through a validated API route and a high-level action enum.
Consume actions from the ESP32 main loop. Do not call motor or servo drivers
from an HTTP callback.

The current firmware uses approximately 90% of the default application flash
partition. Future large web assets should be compressed, moved to a filesystem,
or served by a companion device after reviewing the partition layout.

## Category tracking

1. Make sure live camera video is visible.
2. Press **Track** in the Actions section.
3. Select **Face**, **Animal**, or **Ping-pong ball**.
4. Drag a rectangle around the target directly on the video and release.
5. Press **Track** again to stop tracking.

The selected category must be detected inside the selected box before tracking starts. If the requested category is not detected, MaixCam rejects the selection and the robot remains still.

The browser excludes letterbox areas and sends a normalized target rectangle, so selection remains correct across camera resolutions and phone layouts. The ESP32 forwards TRACKROI coordinates to MaixCam over UART. Canceling sends TRACKSTOP. Movement remains disabled until MaixCam confirms a valid semantic target.

Detector frames are converted to each model's required RGB/BGR format. A transient low-confidence frame coasts without chassis search; reacquisition starts only after consecutive misses.
