# MaixCam Vision Models

[中文](VISION_MODELS.zh-CN.md) | English

## Category Tracking Engine

Current MaixCam firmware intentionally supports category-based tracking only:

- Face: try `yolov8n_face.mud`, then `retinaface.mud`, then `face_detector.mud`.
- Animals: use the installed COCO detector, preferring `yolo11n.mud` and falling back to `yolov8n.mud`. Supported animal labels include bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, and giraffe.
- Ping-pong ball / small ball: use a dedicated ping-pong model when installed, otherwise use the COCO `sports ball` class and report it as `PING_PONG_BALL`.

The selected Web UI target box must be confirmed by the requested semantic detector before tracking starts. If the user selects Face and no face is detected inside the selected region, MaixCam rejects the selection instead of falling back to a generic visual tracker.

## Why Arbitrary-Object Tracking Is Disabled

The current MaixCam hardware supports NanoTrack, but project testing showed that it can drift from the selected target to nearby background objects. Sipeed documentation also shows that MixFormerV2 is not supported on MaixCAM / MaixCAM-Pro; it is a MaixCAM2 path.

For this project version, arbitrary-object tracking is disabled to avoid presenting an unreliable control mode. Future arbitrary-object support should use either a validated custom detector for the target category or hardware that supports a stronger tracker.

## Runtime Behavior

The detector is not run on every frame. MaixCam periodically validates the selected target category and sends normalized target offset, distance error, confidence, and label data to the ESP32. The ESP32 remains responsible for motion safety and converts target errors into bounded high-level motion commands.
