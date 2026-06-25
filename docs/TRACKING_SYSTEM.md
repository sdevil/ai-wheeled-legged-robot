# Tracking System

WRobot uses a state-driven visual tracking pipeline inspired by camera-gimbal and autonomous vehicle tracking systems. The design separates visual confidence from robot motion so that a weak or missing detection cannot directly command the chassis.

## Pipeline

1. The user enables Track and selects a target category: Face, Animal, or Ping-pong ball.
2. The user draws a region around the target in the Web UI.
3. MaixCam confirms that the requested category is detected inside the selected region.
4. MaixCam reports tracking state, image-center error, size error, confidence, target class, and bounding box over UART.
5. ESP32 applies smoothing, motion limits, command timeouts, and recovery behavior through the unified `MotionCommand` interface.

Unsupported arbitrary-object tracking is disabled in this version. The firmware does not use NanoTrack or MixFormer as a fallback when the selected category cannot be detected.

## States

- `IDLE`: tracking is disabled or waiting for a new selection.
- `ACQUIRING`: the selected category must remain stable for multiple frames. The robot does not move.
- `LOCKED`: confidence and geometry are valid. Normal gimbal, yaw, and distance control are enabled.
- `COASTING`: a short detection interruption is treated as temporary occlusion. Chassis motion stops immediately.
- `REACQUIRING`: the detector tries to reacquire the same category near the last known target region. The robot does not perform blind search turns.
- `LOST`: the recovery deadline expired. Motion stops and the camera returns to its standby angle.

## Safety Rules

- Only `LOCKED` observations may command normal chassis movement.
- Missing UART tracking updates for 400 ms trigger recovery and stop forward motion.
- Reacquisition never commands forward or backward travel.
- Before the first successful lock, failed detections remain in `ACQUIRING` and never rotate the chassis.
- Camera corrections are rate-limited to prevent accumulated pitch motion.
- Target boxes are rejected when their center or scale changes implausibly between frames.
- A selected target must keep matching the requested semantic category. If it drifts to another object, MaixCam reports occlusion/reacquisition rather than a false lock.

## Default Tuning

Camera parameters are in `maixcam-code/config.py`. Chassis and gimbal parameters are in `robot-code/src/motion/MotionCoreAdapter.cpp`.

Start real-world tuning with the robot supported so its wheels can move freely. Validate stationary acquisition first, then lateral motion, distance changes, short occlusion, long occlusion, UART disconnect, and mode cancellation.

## Protocol

The current UART observation format is:

```text
TRK:LOCKED,DX:12,DY:-4,DZ:8,Q:935;
```

`Q` is confidence scaled from 0 to 1000. Legacy `DX/DY/DZ` commands remain accepted for compatibility.

## Predictive Carrier Control

Camera observations include resolution-independent `EX`, `EY`, and `EZ` errors. A filtered target velocity predicts slightly more than one frame ahead, with prediction bounded by the target box size. Adaptive box smoothing responds faster to rapid target motion while remaining stable for slow motion.

The chassis uses separate yaw engage and release thresholds. A direction must remain consistent for multiple observations before a bounded turn starts or reverses, so delayed camera observations cannot cause left-right oscillation. Camera errors are converted to the same chassis yaw and forward/reverse polarity used by Web and Gamepad control. IMU yaw-rate feedback damps the command, stopping uses a faster slew rate than acceleration, and residual tracking yaw integral decays after centering. Distance movement requires consistent size-error observations and starts only after the initial distance baseline has settled.
