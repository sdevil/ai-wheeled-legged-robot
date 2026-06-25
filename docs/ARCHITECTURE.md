# WRobot - sdevil Enhanced Architecture

[中文](ARCHITECTURE.zh-CN.md) | English

## Design Principle

The ESP32 owns every real-time and safety-critical actuator decision. Vision,
web, gamepad, voice, and future LLM agents provide intent, never raw motor PWM
or phase-voltage commands.

## Components

| Component | Responsibility |
| --- | --- |
| ESP32-WROOM-32 | Balance, FOC, yaw, legs, actions, safety, command arbitration |
| MPU6050 | Body attitude and angular velocity |
| AS5600 x2 | Wheel position and velocity feedback |
| BLDC motors x2 | Wheel actuation through SimpleFOC |
| STS3032 x2 | Articulated leg positioning over the smart-servo bus |
| MaixCam | Vision processing and UART target-offset output |
| gamepad | Manual Bluetooth input |
| Web controller | Hotspot or home-WiFi control with WebSocket, wrobot.local, and captive portal |

## Runtime Data Flow

```text
Gamepad notification ---------+
WebSocket/HTTP command -----+--> command arbitration --> Wrobot targets
MaixCam DX/DY over UART ----+                              |
                                                          v
MPU6050 + AS5600 feedback --> LQR/yaw/leg control --> motor and servo output
```

Web drive commands temporarily override Gamepad joystick axes. They expire after
500ms without refresh or immediately when the WebSocket disconnects. The longer fallback window prevents HTTP round-trip jitter from repeatedly zeroing a held command; pointer release still sends zero immediately. MaixCam sends stateful `TRK:` observations for target tracking.

## Task and Core Ownership

- The Arduino `loop()` performs sensor updates, balance, yaw, leg logic, action
  sequencing, and FOC updates.
- HTTP, WebSocket, mDNS, and captive DNS run in a FreeRTOS task pinned to Core 0.
- Shared web state is protected by an ESP32 critical-section mutex.
- Network handlers only update high-level command state; they never call motor
  drivers directly.

## Safety Rules for Future Features

1. Every moving command must have a timeout or explicit release condition.
2. AI and network inputs must be range-limited and validated by the ESP32.
3. Stop, sitting, fall recovery, and physical power cutoff take precedence.
4. Network parsing and model inference must not run in the balance loop.
5. New control sources should enter through command arbitration, not modify
   `motor.target` directly.
6. Web actions and camera-pitch commands cross task boundaries through queues;
   network callbacks never call the motion core directly.
7. Disabling the wheel motors explicitly writes zero phase voltage instead of
   leaving the previous PWM duty active.
8. gamepad action buttons are edge-triggered, and controller disconnect forces an
   immediate motion stop.

## Servo calibration policy

The robot must not run any boot-time calibration that writes servo center or offset values. STS3032 center calibration is a maintenance operation and only makes sense when the robot is held in a known mechanical reference pose. Running it automatically at every boot can save a fallen, crouched, blocked, or misaligned pose as the new zero point, making the mechanism progressively worse and adding unnecessary persistent writes.

The recommended boot behavior is a read-only health check: read available servo position, load, temperature, and voltage telemetry, compare it with the expected standby pose, and report offset, load imbalance, or wear trends. The current integrated motion adapter has no verified write-based zero-point calibration, so the Web UI does not expose a non-functional control. A future implementation should enter through the maintenance UI and the shared `MotionCommand` contract only after the calibration procedure is validated.

## Motion Core Boundary

Motion is now isolated behind `MotionCommand` and `MotionCore`:

```text
Web UI / WebSocket ----+
Gamepad adapter ----------+--> MotionCommand --> MotionCore --> integrated controller/devices/system
MaixCam UART ----------+
```

The default implementation is `MotionCoreAdapter`, which wraps the
integrated `controller`, `devices`, and `system/task` code currently maintained
in this repository. Future motion-core upgrades should add or replace an adapter
that implements `MotionCore` and maps the stable `MotionCommand` contract to
that core's internal API. Web, WiFi, OTA, camera, settings, and UI code must not
include or call motion-controller internals directly.

The internal Gamepad driver from imported motion code is not used. Bluetooth,
Web, and camera inputs are normalized outside the core and injected through
`host_data` by the adapter. This keeps the motion core replaceable and avoids
UART/Bluetooth/WiFi ownership conflicts.

## Firmware Size Policy

The firmware uses a local `SimpleFOCMinimal` library containing only the FOC
pieces required by this robot: BLDC motor control, ESP32 3PWM driver, AS5600 I2C
sensor support, PID, and low-pass filtering. Unused SimpleFOC current-sensing and
extra hardware backends are excluded to reduce build time and flash pressure.
FastLED was replaced with a small local WS2812 status LED driver for the same
reason.

The WS2812 status LEDs must be driven through ESP32 RMT, not hand-written GPIO
bit-banging. The bit-banged implementation was unstable under WiFi/WebSocket and
motion-task load and could leave a green LED latched on after the ready blink.
See [Engineering Notes](ENGINEERING_NOTES.md) before changing LED, OTA, Web
control, camera preview, or motion-core integration code.

## Soft Stand

The integrated motion core uses a feedback-aware stand-up transition to reduce chassis displacement. The normal seated posture is rear-supported with retracted legs and its centre of mass behind the wheel axle. The controller therefore keeps the legs retracted while wheel balance first moves the axle under the centre of mass. After the pitch and pitch rate remain upright for a short ready window, it hands over to normal balance without applying a fixed forward correction.

Wheel balance output is blended in during the recovery window. Wheel odometry is recorded for diagnostics, but it is not used as a hard closed-loop position reference during stand-up because wheel slip can make the measured displacement misleading. Recovery is time-bounded to prevent an unstable stand attempt from continuously driving backward. Invalid or unavailable servo feedback uses a conservative fallback.

Soft Stand intentionally does not apply a fixed forward correction. Such compensation is sensitive to floor friction, battery voltage, payload, and mechanical wear and can increase fall risk.
