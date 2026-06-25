# Predictive Tracking Architecture

WRobot uses detector-led category tracking. The chassis is never driven directly from an unverified visual box.

## Pipeline

1. The user selects a target category in the Web UI.
2. The user draws a target box on the live video.
3. MaixCam confirms the selected box with the matching detector: face, animal, or ping-pong ball.
4. A filtered target position and velocity estimate predict bearing during brief visual gaps.
5. Class-specific detection performs local/global reacquisition only after consecutive misses.
6. ESP32 receives normalized bearing, distance error, confidence, state, and tracking profile over UART.
7. The motion core runs its own rate-limited yaw, pitch, and distance loops.

## Tracking Profiles

- `1`: face; smooth follow response
- `2`: animal; faster follow response
- `3`: ping-pong ball; fastest safe chase response

Profile `0` is reserved for idle/unsupported targets and should not command motion.

Every target type preserves its initially locked distance. The camera stabilizes the target-box height over the first locked frames before enabling distance motion. Manual distance adjustment replaces that baseline.

The UART observation includes `P:<profile>` in each `TRK` frame. During `COASTING`, forward motion is disabled. `REACQUIRING` is legal only after a target has previously reached `LOCKED`.

## Ping-Pong Model

The built-in COCO detector falls back to the `sports ball` class. Reliable small-ball chase requires a custom one-class model installed as one of:

- `/root/models/wrobot_pingpong.mud` for YOLO11
- `/root/models/wrobot_pingpong_yolov8.mud` for YOLOv8

The camera defaults to 640x480 and provides 320x240 low-latency and 160x120 remote-control profiles. Inference frames are converted and resized to each model's native tensor format; MJPEG resolution is independent of model tensor size.

## Safety

- Predictions never command forward motion without a confirmed frame.
- Confidence gates motor commands.
- Yaw and drive commands use acceleration and deceleration limits.
- A lost target returns the camera to standby and stops the robot.
