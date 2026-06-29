# WRobot - sdevil Enhanced

**AI-Enhanced Self-Balancing Wheeled-Legged Robot**

[中文文档](README.zh-CN.md) | English

![WRobot - sdevil Enhanced prototype](docs/assets/wrobot-sdevil-enhanced.jpg)

WRobot - sdevil Enhanced is an ESP32-based wheeled-legged robot platform for self-balancing
motion, articulated leg actions, embedded vision, and future AI agent control.
The ESP32 remains the real-time motion controller, while MaixCam or another AI
module handles perception and sends high-level commands.

> [!WARNING]
> This is an experimental robot. Test new firmware with the wheels lifted and
> keep a physical power cutoff within reach. WiFi, vision, and AI commands must
> never bypass the ESP32 motion-safety layer.

## Current Features

- ESP32-WROOM-32 real-time motion controller
- Dual BLDC wheel control with SimpleFOC
- MPU6050-based LQR balance and yaw control
- Two STS3032 smart bus servos for the legs
- Gamepad Bluetooth controller support
- Sitting, standing, stable multi-direction jumping, and obstacle traversal
- MaixCam category tracking for faces, animals, and ping-pong balls, UART target offsets, and MJPEG web video
- WebSocket control with explicit release stop, a 1s lost-command safety timeout, mDNS, and captive portal
- Camera pitch and front mechanism PWM servo control
- RGB and battery-voltage status handling

## System Architecture

```text
MaixCam / future AI module / Web UI / gamepad
                         |
             high-level commands and intent
                         v
                  ESP32 motion controller
               balance, safety, legs, wheels
                         |
          +--------------+---------------+
          |              |               |
      BLDC wheels   STS3032 legs   PWM accessories
```

The AI or vision module does not directly drive motors. See
[Architecture](docs/ARCHITECTURE.md) for control ownership and data flow.

## Repository Layout

| Directory | Purpose |
| --- | --- |
| `robot-code/` | Main ESP32 PlatformIO firmware for motion and web control |
| `maixcam-code/` | MaixCam vision application and UART command output |
| `servo-code/` | Temporary ESP32 serial bridge used during STS3032 setup |
| `web-ui-react/` | React/TypeScript source for the embedded control UI |
| `docs/` | English and Chinese project documentation |

## Quick Start

### Build the robot firmware

```powershell
cd robot-code
pio run
```

Upload only when the robot is mechanically secured:

```powershell
pio run -t upload
```

> [!IMPORTANT]
> Before USB flashing the `robot-code` main firmware, disconnect the **MaixCam
> UART connection to the board `TX/RX` header**.
> In practice, leaving the camera UART attached can interfere with ESP32
> download mode and cause upload failures such as `Serial data stream stopped`
> or `Failed to write to target RAM`.
> This is not primarily an upload-speed issue; it is a hardware serial conflict.
>
> Current rule:
> - **Before USB flashing the main firmware: disconnect camera UART**
> - **After flashing the main firmware: reconnect camera UART**
> - **MaixCam firmware updates are not affected by this rule**

### Use the web controller

After flashing the current robot firmware:

```text
WiFi SSID: sdevil-WRobot
Password:  wrobot123
URL:       http://192.168.8.1
Local name: http://wrobot.local
```

Detailed instructions and API endpoints are documented in
[Web Control](docs/WEB_CONTROL.md).

### Update firmware over OTA

The robot mainboard can update firmware from the Web UI:

1. Build `robot-code/.pio/build/esp32dev/wrobot_firmware_<version>.bin`
   (for example `wrobot_firmware_3.2.51.bin`).
2. Open the robot Web UI and go to **Settings**.
3. Enter **Maintenance Mode**.
4. Make sure the robot is sitting and disabled.
5. Confirm battery is at least `30%`, or enable the USB power override when the
   robot is powered from USB.
6. Select the generated `wrobot_firmware_<version>.bin` file in
   **Advanced Maintenance** and upload it.
7. Keep the page open until the robot reboots and the Web UI reports the new
   firmware version.

OTA writes to the inactive ESP32 application slot. If upload is interrupted or
power is lost during upload, the previous bootable firmware remains available.
If OTA is not reachable, the mainboard can still be recovered by USB flashing.

The detector-assisted chase and reacquisition design is documented in
[Predictive Tracking Architecture](docs/TRACKING_ARCHITECTURE.md).
The camera network boundary is documented in [Camera Privacy Audit](docs/CAMERA_PRIVACY.md).

Flash usage, partition choices, and optimization rules are documented in
[Flash and Memory Budget](docs/FLASH_BUDGET.md).
Repository ownership and generated-file rules are documented in [Project Structure](docs/PROJECT_STRUCTURE.md).
Hardware-specific stability lessons are documented in
[Engineering Notes](docs/ENGINEERING_NOTES.md). Read them before changing LED,
OTA, Web control, camera preview, or motion-core integration code.

## Development Roadmap

- V1: vision tracking and a stable mobile web controller
- V2: voice commands and expressive display integration
- V3: external AI agent and LLM command integration
- V4: expanded wheeled-legged actions and autonomy

## Origins and Acknowledgements

This project is based on the original publicly shared design:

1. [MuShibo/Micro-Wheeled_leg-Robot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot)
   by Mu Shibo, with Li Yufeng listed as a contributor.

The current project is maintained by **sdevil**. See
[Acknowledgements](ACKNOWLEDGEMENTS.md) and [NOTICE](NOTICE.md) for full
attribution and provenance.

## License Status

No project-wide license has been selected yet. At the time of review, neither
upstream repository contained a root license file. Public source availability
alone does not grant redistribution or relicensing rights. Upstream permission
and licensing must be clarified before a public release of derived code.

Bundled third-party libraries remain subject to their own license files. See
[NOTICE](NOTICE.md) before redistributing this repository.

This repository is intended for public source sharing, learning, and
collaboration. It is not currently presented as an OSI-licensed open-source
distribution because the upstream licensing terms still need clarification.
