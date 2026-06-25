"""Runtime configuration for the WRobot MaixCam application."""

import os

from maix import err, gpio, pinmap, time

LED_PIN_NAME = "A14"
LED_GPIO_NAME = "GPIOA14"
BLINK_INTERVAL = 500

led = None
previous_millis = 0

WEB_PREVIEW_ENABLED = True
WEB_PREVIEW_HOST = "0.0.0.0"
WEB_PREVIEW_PORT = 8080
WEB_PREVIEW_SNAPSHOT_PATH = "/snapshot.jpg"
WEB_PREVIEW_STREAM_PATH = "/stream.mjpg"
WEB_PREVIEW_REFRESH_MS = 80
WEB_PREVIEW_JPEG_QUALITY = 95
WEB_PREVIEW_ENCODE_INTERVAL_MS = 70
WEB_PREVIEW_PROFILES = {
    "160x120": {"interval_ms": 40, "jpeg_quality": 62},
    "320x240": {"interval_ms": 45, "jpeg_quality": 80},
    "640x480": {"interval_ms": 60, "jpeg_quality": 82},
    "1280x720": {"interval_ms": 140, "jpeg_quality": 82},
}
WEB_PREVIEW_TEMP_FILE = "/root/wrobot_snapshot.jpg"

DEFAULT_CAMERA_RESOLUTION = "640x480"
CAMERA_RESOLUTION_PRESETS = {
    "160x120": (160, 120),
    "320x240": (320, 240),
    "640x480": (640, 480),
    "1280x720": (1280, 720),
}
CAMERA_FRAME_WIDTH = CAMERA_RESOLUTION_PRESETS[DEFAULT_CAMERA_RESOLUTION][0]
CAMERA_FRAME_HEIGHT = CAMERA_RESOLUTION_PRESETS[DEFAULT_CAMERA_RESOLUTION][1]

def _load_app_version():
    """Read the MaixVision app version from app.yaml.

    Keep the version number in app.yaml only; runtime status reports reuse it
    so the flashed app and Web UI never drift apart.
    """
    candidates = []
    try:
        candidates.append(os.path.join(os.path.dirname(__file__), "app.yaml"))
    except Exception:
        pass
    candidates.append("app.yaml")

    for path in candidates:
        try:
            with open(path, "r", encoding="utf-8") as file:
                for line in file:
                    stripped = line.strip()
                    if stripped.startswith("version:"):
                        version = stripped.split(":", 1)[1].strip().strip("'\"")
                        return version or "unknown"
        except Exception:
            pass
    return "unknown"


APP_VERSION = _load_app_version()
BUILD_ID = "2026-06-25-track-fast-response-01"

WIFI_AUTO_CONNECT = True
FORCE_ROBOT_AP_MODE = False
ROBOT_AP_SSID = "sdevil-WRobot"
ROBOT_AP_PASSWORD = "wrobot123"
WIFI_SSID = ""
WIFI_PASSWORD = ""
WIFI_CONNECT_TIMEOUT_SEC = 20

FACE_MODEL_CANDIDATES = (
    ("/root/models/yolov8n_face.mud", "YOLOv8"),
    ("/root/models/yolov8n_face.cvimodel", "YOLOv8"),
    ("/root/models/retinaface.mud", "Retinaface"),
    ("/root/models/retinaface.cvimodel", "Retinaface"),
    ("/root/models/face_detector.mud", "FaceDetector"),
    ("/root/models/face_detector.cvimodel", "FaceDetector"),
)
OBJECT_MODEL_CANDIDATES = (
    ("/root/models/yolo11n.mud", "YOLO11"),
    ("/root/models/yolo11n.cvimodel", "YOLO11"),
    ("/root/models/yolov8n.mud", "YOLOv8"),
    ("/root/models/yolov8n.cvimodel", "YOLOv8"),
)
BALL_MODEL_CANDIDATES = (
    ("/root/models/wrobot_pingpong.mud", "YOLO11"),
    ("/root/models/wrobot_pingpong_yolov8.mud", "YOLOv8"),
)
HYBRID_DETECTION_ENABLED = True
HYBRID_DETECT_EVERY_N_FRAMES = 6
HYBRID_DETECT_CONFIDENCE = 0.18
HYBRID_DETECT_IOU = 0.45
HYBRID_ROI_MIN_IOU = 0.015
HYBRID_INITIAL_MIN_AREA_RATIO = 0.03
HYBRID_INITIAL_MAX_AREA_RATIO = 9.0
HYBRID_REACQUIRE_MIN_IOU = 0.0
HYBRID_REINIT_TRACKER_ON_DETECT = True
TRACK_SEMANTIC_VERIFY_EVERY_N_FRAMES = 1
TRACK_SEMANTIC_VERIFY_MISSES = 10
TRACK_SEMANTIC_VALID_MS = 4500
DETECTOR_SCORE_THRESHOLD = 0.58
DETECTOR_REACQUIRE_SCORE_THRESHOLD = 0.30
TRACK_ACQUIRE_CONFIRM_FRAMES = 1
TRACK_REACQUIRE_CONFIRM_FRAMES = 1
TRACK_COAST_FRAMES = 10
TRACK_REACQUIRE_TIMEOUT_MS = 3500
TRACK_BBOX_SMOOTH_ALPHA = 0.96
TRACK_MAX_CENTER_JUMP_RATIO = 0.42
TRACK_MAX_SCALE_CHANGE = 0.20
TRACK_DX_DEADBAND = 4
TRACK_DY_DEADBAND = 4
TRACK_VELOCITY_ALPHA = 0.50
TRACK_PREDICTION_FRAMES = 0.8
TRACK_MAX_PREDICTION_BOX_RATIO = 0.18
TRACK_DISTANCE_ADJUST_STEP_PX = 4.0
TRACK_DISTANCE_BASELINE_FRAMES = 12
TRACK_DISTANCE_MIN_HEIGHT_RATIO = 0.08
TRACK_DISTANCE_MAX_HEIGHT_RATIO = 0.82


def init_led():
    """Initialize the MaixCam status LED."""
    global led
    try:
        err.check_raise(
            pinmap.set_pin_function(LED_PIN_NAME, LED_GPIO_NAME),
            f"Failed to configure LED pin {LED_PIN_NAME}",
        )
        led = gpio.GPIO(LED_GPIO_NAME, gpio.Mode.OUT)
        led.value(0)
        print(f"[WROBOT] LED ready: {LED_PIN_NAME} ({LED_GPIO_NAME})")
    except Exception as exc:
        print(f"[WROBOT] LED initialization failed: {exc}")
        led = None


def update_led():
    """Toggle the status LED at a fixed interval."""
    global previous_millis
    if led is None:
        return

    current_millis = time.time_ms()
    if current_millis - previous_millis >= BLINK_INTERVAL:
        previous_millis = current_millis
        led.toggle()

