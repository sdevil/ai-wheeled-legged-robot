import os

from maix import app, image, nn, time

from camera_control import handle_incoming_camera_commands, report_camera_detection, report_camera_status
from config import (
    DETECTOR_REACQUIRE_SCORE_THRESHOLD,
    DETECTOR_SCORE_THRESHOLD,
    FACE_MODEL_CANDIDATES, HYBRID_DETECT_CONFIDENCE,
    HYBRID_DETECT_EVERY_N_FRAMES, HYBRID_DETECT_IOU,
    HYBRID_DETECTION_ENABLED, HYBRID_REACQUIRE_MIN_IOU,
    HYBRID_INITIAL_MIN_AREA_RATIO, HYBRID_INITIAL_MAX_AREA_RATIO,
    HYBRID_REINIT_TRACKER_ON_DETECT, HYBRID_ROI_MIN_IOU,
    TRACK_ACQUIRE_CONFIRM_FRAMES,
    TRACK_BBOX_SMOOTH_ALPHA, TRACK_COAST_FRAMES, TRACK_DX_DEADBAND,
    TRACK_DY_DEADBAND, TRACK_MAX_CENTER_JUMP_RATIO, TRACK_MAX_SCALE_CHANGE,
    BALL_MODEL_CANDIDATES,
    OBJECT_MODEL_CANDIDATES,
    TRACK_REACQUIRE_CONFIRM_FRAMES, TRACK_REACQUIRE_TIMEOUT_MS,
    TRACK_VELOCITY_ALPHA, TRACK_PREDICTION_FRAMES,
    TRACK_MAX_PREDICTION_BOX_RATIO, TRACK_DISTANCE_ADJUST_STEP_PX,
    TRACK_DISTANCE_BASELINE_FRAMES,
    TRACK_DISTANCE_MIN_HEIGHT_RATIO, TRACK_DISTANCE_MAX_HEIGHT_RATIO,
    TRACK_SEMANTIC_VALID_MS, TRACK_SEMANTIC_VERIFY_EVERY_N_FRAMES,
    TRACK_SEMANTIC_VERIFY_MISSES,
)

NORMALIZED_ROI_MAX = 10000
MIN_TARGET_SIZE_PX = 12
IDLE = "IDLE"
ACQUIRING = "ACQUIRING"
LOCKED = "LOCKED"
COASTING = "COASTING"
REACQUIRING = "REACQUIRING"
LOST = "LOST"
COCO_LABELS = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)
ANIMAL_LABELS = {
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
    "zebra", "giraffe",
}
BALL_LABELS = {"sports ball"}
PERSON_LABELS = {"person"}


class Detection:
    def __init__(self, box, score, label, family):
        self.box = box
        self.score = score
        self.label = label
        self.family = family


def create_hybrid_detector():
    return HybridDetector() if HYBRID_DETECTION_ENABLED else None


class HybridDetector:
    def __init__(self):
        self.face = None
        self.face_kind = ""
        self.object_detector = None
        self.object_kind = ""
        self.object_labels = COCO_LABELS
        self.ball_detector = None
        self.ball_kind = ""
        self._load_face_detector()
        self._load_ball_detector()
        self._load_object_detector()

    def ready(self):
        return (
            self.face is not None or self.ball_detector is not None or
            self.object_detector is not None
        )

    def _load_model(self, path, kind):
        try:
            if kind == "YOLO11" and hasattr(nn, "YOLO11"):
                return nn.YOLO11(model=path, dual_buff=True)
            if kind == "YOLOv8" and hasattr(nn, "YOLOv8"):
                return nn.YOLOv8(model=path, dual_buff=True)
            if kind == "Retinaface" and hasattr(nn, "Retinaface"):
                return nn.Retinaface(model=path)
            if kind == "FaceDetector" and hasattr(nn, "FaceDetector"):
                return nn.FaceDetector(model=path)
        except Exception as exc:
            print(f"[WROBOT] detector load failed: {kind} {path}: {exc}")
        return None

    def _load_face_detector(self):
        for path, kind in FACE_MODEL_CANDIDATES:
            if not os.path.exists(path):
                continue
            detector = self._load_model(path, kind)
            if detector is not None:
                self.face = detector
                self.face_kind = kind
                print(
                    f"[WROBOT] face detector: {kind}({path}) "
                    f"input={_detector_input_size(detector)}"
                )
                return
        print("[WROBOT] face detector unavailable")

    def _load_ball_detector(self):
        for path, kind in BALL_MODEL_CANDIDATES:
            if not os.path.exists(path):
                continue
            detector = self._load_model(path, kind)
            if detector is not None:
                self.ball_detector = detector
                self.ball_kind = kind
                print(
                    f"[WROBOT] ping-pong detector: {kind}({path}) "
                    f"input={_detector_input_size(detector)}"
                )
                return
        print("[WROBOT] dedicated ping-pong detector unavailable; using generic detector")

    def _load_object_detector(self):
        for path, kind in OBJECT_MODEL_CANDIDATES:
            if not os.path.exists(path):
                continue
            detector = self._load_model(path, kind)
            if detector is not None:
                self.object_detector = detector
                self.object_kind = kind
                labels = getattr(detector, "labels", None)
                if labels:
                    self.object_labels = labels
                print(f"[WROBOT] object detector: {kind}({path})")
                return
        print("[WROBOT] object detector unavailable")

    def _collect_candidates(self, img, target_family=""):
        candidates = []
        target_family = "" if target_family == "generic" else target_family
        if self.face is not None and target_family in ("", "face"):
            candidates.extend(self._detect_faces(img))
        if self.ball_detector is not None and target_family in ("", "ball"):
            candidates.extend(self._detect_balls(img))
        if (
            self.object_detector is not None and target_family != "face" and
            not (target_family == "ball" and self.ball_detector is not None)
        ):
            candidates.extend(self._detect_objects(img))
        return candidates

    def detect(
        self, img, roi=None, target_family="", target_label="",
        preserve_roi_scale=False, loose_roi=False,
    ):
        candidates = self._collect_candidates(img, target_family)
        target_family = "" if target_family == "generic" else target_family
        if target_family:
            candidates = [
                item for item in candidates
                if item.family == target_family and
                (not target_label or item.label == target_label)
            ]
        semantic_candidates = list(candidates)
        if roi is not None:
            candidates = [
                item for item in candidates
                if _iou(item.box, roi) >= HYBRID_ROI_MIN_IOU or
                _center_inside(item.box, roi) or
                _center_inside(roi, item.box)
            ]
        if loose_roi and roi is not None and target_family == "face":
            roi_center = _center(roi)
            roi_diagonal = max(1.0, (roi[2] * roi[2] + roi[3] * roi[3]) ** 0.5)

            def loose_distance(item):
                item_center = _center(item.box)
                return (
                    (item_center[0] - roi_center[0]) ** 2 +
                    (item_center[1] - roi_center[1]) ** 2
                ) ** 0.5

            loose_faces = [
                item for item in semantic_candidates
                if loose_distance(item) <= roi_diagonal * 2.8
            ]
            if loose_faces:
                candidates = loose_faces
        if roi is not None and preserve_roi_scale:
            roi_area = max(1.0, roi[2] * roi[3])
            candidates = [
                item for item in candidates
                if HYBRID_INITIAL_MIN_AREA_RATIO <=
                (item.box[2] * item.box[3]) / roi_area <=
                HYBRID_INITIAL_MAX_AREA_RATIO
            ]
        if not candidates and roi is not None and target_family == "face":
            roi_center = _center(roi)
            roi_diagonal = max(1.0, (roi[2] * roi[2] + roi[3] * roi[3]) ** 0.5)

            def near_roi_score(item):
                item_center = _center(item.box)
                center_distance = (
                    (item_center[0] - roi_center[0]) ** 2 +
                    (item_center[1] - roi_center[1]) ** 2
                ) ** 0.5
                return center_distance

            near_faces = [
                item for item in semantic_candidates
                if near_roi_score(item) <= roi_diagonal * 2.2
            ]
            if near_faces:
                candidates = near_faces
        if not candidates:
            return None
        if roi is None:
            return max(candidates, key=lambda item: item.score)

        roi_area = max(1.0, roi[2] * roi[3])
        roi_center = _center(roi)
        roi_diagonal = max(1.0, (roi[2] * roi[2] + roi[3] * roi[3]) ** 0.5)

        def association_score(item):
            item_area = max(1.0, item.box[2] * item.box[3])
            area_ratio = item_area / roi_area
            scale_score = min(area_ratio, 1.0 / area_ratio)
            item_center = _center(item.box)
            center_distance = (
                (item_center[0] - roi_center[0]) ** 2 +
                (item_center[1] - roi_center[1]) ** 2
            ) ** 0.5
            center_score = max(0.0, 1.0 - center_distance / roi_diagonal)
            return (
                item.score * 0.30 +
                _iou(item.box, roi) * 0.40 +
                scale_score * 0.20 +
                center_score * 0.10
            )

        return max(candidates, key=association_score)

    def reacquire_global(self, img, predicted_box, target_family, target_label):
        candidates = self._collect_candidates(img, target_family)
        if target_family and target_family != "generic":
            candidates = [item for item in candidates if item.family == target_family]
        if target_label:
            candidates = [item for item in candidates if item.label == target_label]
        if not candidates:
            return None
        if predicted_box is None:
            return max(candidates, key=lambda item: item.score)
        image_width, image_height = _image_size(img)
        if target_family == "face":
            candidates = [
                item for item in candidates
                if _valid_geometry(item.box, predicted_box, image_width, image_height)
            ]
            if not candidates:
                return None
        diagonal = max(1.0, (image_width * image_width + image_height * image_height) ** 0.5)
        predicted_center = _center(predicted_box)

        def association_score(item):
            center = _center(item.box)
            distance = (
                (center[0] - predicted_center[0]) ** 2 +
                (center[1] - predicted_center[1]) ** 2
            ) ** 0.5
            proximity = max(0.0, 1.0 - distance / diagonal)
            return item.score * 0.40 + proximity * 0.60

        return max(candidates, key=association_score)

    def reacquire(self, img, previous_box, target_family, target_label):
        candidate = self.detect(img, previous_box, target_family, target_label)
        if candidate is None:
            return self.reacquire_global(img, previous_box, target_family, target_label)
        if previous_box is None:
            return candidate
        if _iou(candidate.box, previous_box) >= HYBRID_REACQUIRE_MIN_IOU:
            return candidate
        if _center_inside(candidate.box, previous_box) or _center_inside(previous_box, candidate.box):
            return candidate
        return self.reacquire_global(img, previous_box, target_family, target_label)

    def _detect_faces(self, img):
        detect_img, scale_x, scale_y = _image_for_detector(self.face, img)
        try:
            if self.face_kind == "YOLOv8":
                objs = self.face.detect(detect_img, conf_th=HYBRID_DETECT_CONFIDENCE, iou_th=HYBRID_DETECT_IOU, keypoint_th=0.2)
            else:
                objs = self.face.detect(detect_img, conf_th=HYBRID_DETECT_CONFIDENCE, iou_th=HYBRID_DETECT_IOU)
        except TypeError:
            objs = self.face.detect(detect_img, conf_th=HYBRID_DETECT_CONFIDENCE, iou_th=HYBRID_DETECT_IOU)
        except Exception as exc:
            print(f"[WROBOT] face detect failed: {exc}")
            return []
        detections = [
            Detection(_box_scaled(obj, scale_x, scale_y), _score(obj), "face", "face")
            for obj in objs
        ]
        if detections:
            print(f"[WROBOT] face detections: {len(detections)} best={max(item.score for item in detections):.2f}")
        return detections

    def _detect_objects(self, img):
        detect_img, scale_x, scale_y = _image_for_detector(self.object_detector, img)
        try:
            objs = self.object_detector.detect(
                detect_img, conf_th=HYBRID_DETECT_CONFIDENCE, iou_th=HYBRID_DETECT_IOU
            )
        except Exception as exc:
            print(f"[WROBOT] object detect failed: {exc}")
            return []

        detections = []
        for obj in objs:
            class_id = int(_call_number(obj, "class_id", -1))
            label = self._label_for_class(class_id)
            family = self._family_for_label(label)
            if not family:
                continue
            detections.append(Detection(_box_scaled(obj, scale_x, scale_y), _score(obj), label, family))
        return detections

    def _detect_balls(self, img):
        detect_img, scale_x, scale_y = _image_for_detector(self.ball_detector, img)
        try:
            objs = self.ball_detector.detect(
                detect_img,
                conf_th=HYBRID_DETECT_CONFIDENCE,
                iou_th=HYBRID_DETECT_IOU,
            )
        except Exception as exc:
            print(f"[WROBOT] ping-pong detect failed: {exc}")
            return []
        return [
            Detection(
                _box_scaled(obj, scale_x, scale_y),
                _score(obj),
                "ping pong ball",
                "ball",
            )
            for obj in objs
        ]

    def _label_for_class(self, class_id):
        try:
            label = self.object_labels[class_id]
            return str(label).strip().lower()
        except Exception:
            return ""

    def _family_for_label(self, label):
        if label in ANIMAL_LABELS:
            return "animal"
        if label in BALL_LABELS:
            return "ball"
        if label in PERSON_LABELS:
            return "person"
        return ""


def _parse_values(command):
    values = {}
    _, _, payload = command.partition(":")
    for item in payload.split(","):
        key, separator, value = item.partition("=")
        if separator:
            values[key.strip()] = value.strip()
    return values


def _roi_to_pixels(command, image_width, image_height):
    values = _parse_values(command)
    x = max(0, min(9999, int(values.get("x", "0"))))
    y = max(0, min(9999, int(values.get("y", "0"))))
    w = max(1, min(10000 - x, int(values.get("w", "0"))))
    h = max(1, min(10000 - y, int(values.get("h", "0"))))
    px = x * image_width // NORMALIZED_ROI_MAX
    py = y * image_height // NORMALIZED_ROI_MAX
    pw = min(max(MIN_TARGET_SIZE_PX, w * image_width // NORMALIZED_ROI_MAX), image_width - px)
    ph = min(max(MIN_TARGET_SIZE_PX, h * image_height // NORMALIZED_ROI_MAX), image_height - py)
    if pw < MIN_TARGET_SIZE_PX or ph < MIN_TARGET_SIZE_PX:
        raise ValueError("Selected target is too small")
    return px, py, pw, ph


def _call_number(obj, name, fallback=0):
    value = getattr(obj, name, fallback)
    try:
        return value() if callable(value) else value
    except Exception:
        return fallback


def _image_size(img):
    width = _call_number(img, "width", 0)
    height = _call_number(img, "height", 0)
    return int(width or 0), int(height or 0)


def _detector_input_size(detector):
    try:
        width = int(detector.input_width())
        height = int(detector.input_height())
        return width, height
    except Exception:
        return 0, 0


def _image_for_detector(detector, img):
    image_width, image_height = _image_size(img)
    input_width, input_height = _detector_input_size(detector)
    prepared = img
    try:
        input_format = detector.input_format()
        if prepared.format() != input_format:
            prepared = prepared.to_format(input_format)
    except Exception as exc:
        print(f"[WROBOT] detector format conversion failed: {exc}")
    if (
        input_width > 0 and input_height > 0 and
        image_width > 0 and image_height > 0 and
        (input_width != image_width or input_height != image_height) and
        hasattr(prepared, "resize")
    ):
        try:
            return prepared.resize(input_width, input_height), image_width / input_width, image_height / input_height
        except Exception as exc:
            print(f"[WROBOT] detector resize failed: {exc}")
    return prepared, 1.0, 1.0


def _box(result):
    return [
        float(_call_number(result, "x", 0)),
        float(_call_number(result, "y", 0)),
        float(_call_number(result, "w", 0)),
        float(_call_number(result, "h", 0)),
    ]


def _box_scaled(result, scale_x, scale_y):
    box = _box(result)
    return [box[0] * scale_x, box[1] * scale_y, box[2] * scale_x, box[3] * scale_y]


def _score(result):
    for name in ("score", "prob", "confidence"):
        value = getattr(result, name, None)
        if value is not None:
            try:
                return float(value() if callable(value) else value)
            except Exception:
                pass
    return 1.0


def _valid_geometry(box, previous, image_width, image_height):
    x, y, w, h = box
    if w < MIN_TARGET_SIZE_PX or h < MIN_TARGET_SIZE_PX:
        return False
    if x + w <= 0 or y + h <= 0 or x >= image_width or y >= image_height:
        return False
    if previous is None:
        return True
    px, py, pw, ph = previous
    distance = (((x + w * 0.5) - (px + pw * 0.5)) ** 2 + ((y + h * 0.5) - (py + ph * 0.5)) ** 2) ** 0.5
    allowed = max(28.0, ((pw * pw + ph * ph) ** 0.5) * TRACK_MAX_CENTER_JUMP_RATIO)
    width_change = abs(w - pw) / max(pw, 1.0)
    height_change = abs(h - ph) / max(ph, 1.0)
    return distance <= allowed and width_change <= TRACK_MAX_SCALE_CHANGE and height_change <= TRACK_MAX_SCALE_CHANGE


def _center(box):
    return box[0] + box[2] * 0.5, box[1] + box[3] * 0.5


def _iou(a, b):
    if a is None or b is None:
        return 0.0
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    left = max(ax, bx)
    top = max(ay, by)
    right = min(ax + aw, bx + bw)
    bottom = min(ay + ah, by + bh)
    inter = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = max(1.0, aw * ah)
    area_b = max(1.0, bw * bh)
    return inter / max(1.0, area_a + area_b - inter)


def _center_inside(box, roi):
    if box is None or roi is None:
        return False
    cx, cy = _center(box)
    x, y, w, h = roi
    return x <= cx <= x + w and y <= cy <= y + h


def _semantic_label(family, label, state):
    if state == ACQUIRING:
        return "TRACK_ACQUIRING"
    if state in (COASTING, REACQUIRING):
        return "TRACK_REACQUIRING"
    if family == "face":
        return "FACE_LOCKED"
    if family == "animal":
        return f"ANIMAL_{label.upper()}"
    if family == "ball":
        return "PING_PONG_BALL"
    if family == "person":
        return "PERSON_LOCKED"
    return "TARGET_UNSUPPORTED"


def _semantic_display_label(family, label):
    if family == "face":
        return "face"
    if family == "animal":
        return label or "animal"
    if family == "ball":
        return "ping pong ball"
    if family == "person":
        return "person"
    return "target"


def _tracking_profile(family):
    if family == "ball":
        return 3
    if family == "animal":
        return 2
    if family in ("face", "person"):
        return 1
    return 0


def _smooth(previous, current):
    if previous is None:
        return current
    previous_x, previous_y = _center(previous)
    current_x, current_y = _center(current)
    distance = ((current_x - previous_x) ** 2 + (current_y - previous_y) ** 2) ** 0.5
    diagonal = max(1.0, (previous[2] ** 2 + previous[3] ** 2) ** 0.5)
    movement_ratio = distance / diagonal
    if movement_ratio < 0.05:
        adaptive_alpha = TRACK_BBOX_SMOOTH_ALPHA
    elif movement_ratio < 0.15:
        adaptive_alpha = 0.70
    else:
        adaptive_alpha = 0.45
    return [previous[i] + (current[i] - previous[i]) * adaptive_alpha for i in range(4)]


def _predict_box(box, velocity_x, velocity_y, steps, image_width, image_height):
    if box is None:
        return None
    x, y, w, h = box
    predicted_x = max(0.0, min(image_width - w, x + velocity_x * steps))
    predicted_y = max(0.0, min(image_height - h, y + velocity_y * steps))
    return [predicted_x, predicted_y, w, h]


def _errors(box, image_center, target_height, velocity_x, velocity_y):
    x, y, w, h = box
    center_x, center_y = _center(box)
    predicted_x = center_x + max(-w * TRACK_MAX_PREDICTION_BOX_RATIO, min(w * TRACK_MAX_PREDICTION_BOX_RATIO, velocity_x * TRACK_PREDICTION_FRAMES))
    predicted_y = center_y + max(-h * TRACK_MAX_PREDICTION_BOX_RATIO, min(h * TRACK_MAX_PREDICTION_BOX_RATIO, velocity_y * TRACK_PREDICTION_FRAMES))
    dx = int(predicted_x) - image_center[0]
    dy = int(predicted_y) - image_center[1]
    dz = target_height - int(h)
    if abs(dx) <= TRACK_DX_DEADBAND:
        dx = 0
    if abs(dy) <= TRACK_DY_DEADBAND:
        dy = 0

    ex = max(-1000, min(1000, int(dx * 1000 / max(1, image_center[0]))))
    ey = max(-1000, min(1000, int(dy * 1000 / max(1, image_center[1]))))
    ez = max(-1000, min(1000, int(dz * 1000 / max(1, target_height))))
    return dx, dy, dz, ex, ey, ez


def _command(
    state, dx=9999, dy=9999, dz=9999, ex=9999, ey=9999, ez=9999,
    score=0.0, profile=0, box=None, frame_size=None, target_height=0,
    missed_frames=0, stable_frames=0, velocity_x=0.0, velocity_y=0.0,
):
    confidence = max(0, min(1000, int(score * 1000)))
    parts = [
        f"TRK:{state}", f"DX:{dx}", f"DY:{dy}", f"DZ:{dz}",
        f"EX:{ex}", f"EY:{ey}", f"EZ:{ez}", f"Q:{confidence}",
        f"P:{max(0, min(3, int(profile)))}",
    ]
    if box is not None:
        bx, by, bw, bh = [int(value) for value in box]
        parts += [f"BX:{bx}", f"BY:{by}", f"BW:{bw}", f"BH:{bh}"]
    if frame_size is not None:
        fw, fh = frame_size
        parts += [f"FW:{int(fw)}", f"FH:{int(fh)}"]
    if target_height:
        parts.append(f"TH:{int(target_height)}")
    parts += [
        f"MF:{int(missed_frames)}", f"SF:{int(stable_frames)}",
        f"RS:{confidence}", f"VX:{int(velocity_x)}", f"VY:{int(velocity_y)}",
    ]
    return ",".join(parts) + ";"


def _track_distance_delta(command):
    values = _parse_values(command)
    try:
        return max(-100, min(100, int(values.get("v", "0"))))
    except Exception:
        return 0

def run_object_tracking(resources):
    cam = resources["cam"]
    disp = resources["disp"]
    serial_dev = resources["serial_dev"]
    preview_server = resources.get("preview_server")
    image_center = resources["image_center"]
    image_width = resources["img_width"]
    image_height = resources["img_height"]
    active_resolution = resources.get("active_resolution")
    hybrid_detector = resources.get("hybrid_detector")

    pending_command = None
    tracking = False
    ever_locked = False
    state = IDLE
    target_height = 0
    target_distance_locked = False
    distance_baseline_frames = 0
    missed_frames = 0
    stable_frames = 0
    lost_since_ms = 0
    filtered_box = None
    target_family = ""
    target_label = ""
    requested_profile = 0
    classify_frames = 0
    semantic_miss_frames = 0
    semantic_valid_until_ms = 0
    velocity_x = 0.0
    velocity_y = 0.0
    frame_count = 0
    last_label = ""
    last_count = -1

    def receive_track_command(command):
        nonlocal pending_command
        pending_command = command

    detector_status = "hybrid" if hybrid_detector and hybrid_detector.ready() else "detector_unavailable"
    report_camera_status(
        serial_dev,
        "READY",
        resources.get("device_ip", ""),
        f"Tracking ready: detector={detector_status}",
    )
    report_camera_detection(serial_dev, "NO_TARGET", 0)

    while not app.need_exit():
        img = cam.read()
        new_ip = handle_incoming_camera_commands(
            serial_dev,
            preview_server,
            receive_track_command,
            active_resolution,
        )
        now_ms = time.time_ms()
        frame_count += 1

        if pending_command:
            command = pending_command
            pending_command = None
            if command.startswith("TRACKSTOP"):
                tracking, state, target_height = False, IDLE, 0
                ever_locked = False
                target_distance_locked = False
                distance_baseline_frames = 0
                missed_frames, stable_frames, lost_since_ms = 0, 0, 0
                filtered_box = None
                target_family, target_label = "", ""
                requested_profile = 0
                classify_frames = 0
                semantic_miss_frames = 0
                semantic_valid_until_ms = 0
                velocity_x, velocity_y = 0.0, 0.0
                report_camera_detection(serial_dev, "NO_TARGET", 0)
            elif command.startswith("TRACKROI:"):
                try:
                    requested_profile = max(
                        0, min(3, int(_parse_values(command).get("p", "0")))
                    )
                    x, y, w, h = _roi_to_pixels(command, image_width, image_height)
                    roi = [float(x), float(y), float(w), float(h)]
                    requested_family = "face" if requested_profile == 1 else ("animal" if requested_profile == 2 else ("ball" if requested_profile == 3 else ""))
                    if not requested_family:
                        raise RuntimeError("Unsupported tracking target profile")
                    semantic = (
                        hybrid_detector.detect(
                            img, roi, requested_family,
                            preserve_roi_scale=False,
                            loose_roi=True,
                        )
                        if hybrid_detector and hybrid_detector.ready() else None
                    )
                    if semantic is not None:
                        target_family, target_label = semantic.family, semantic.label
                        semantic_valid_until_ms = now_ms + TRACK_SEMANTIC_VALID_MS
                        print(f"[WROBOT] semantic target: {target_family}/{target_label} score={semantic.score:.2f}")
                    else:
                        raise RuntimeError(f"No {requested_family or 'supported'} target detected inside selected box")
                    classify_frames = 0
                    semantic_miss_frames = 0
                    initial_target_height = max(MIN_TARGET_SIZE_PX, int(semantic.box[3]))
                    tracking, state, target_height = True, LOCKED, initial_target_height
                    ever_locked = True
                    # Distance follows the detected semantic target, not the
                    # user's selection rectangle. The selection rectangle can be
                    # loose; using it as distance baseline makes follow distance
                    # unstable.
                    target_distance_locked = True
                    distance_baseline_frames = TRACK_DISTANCE_BASELINE_FRAMES
                    missed_frames, stable_frames, lost_since_ms = 0, TRACK_ACQUIRE_CONFIRM_FRAMES, 0
                    filtered_box = [float(value) for value in semantic.box]
                    velocity_x, velocity_y = 0.0, 0.0
                    report_camera_detection(serial_dev, _semantic_label(target_family, target_label, LOCKED), 1)
                    print(f"[WROBOT] target locked from selection: x={x}, y={y}, w={w}, h={h}, target_h={target_height}, family={target_family}")
                except Exception as exc:
                    tracking, state, filtered_box = False, IDLE, None
                    ever_locked = False
                    target_family, target_label = "", ""
                    classify_frames = 0
                    semantic_miss_frames = 0
                    semantic_valid_until_ms = 0
                    report_camera_detection(serial_dev, "TARGET_INVALID", 0)
                    print(f"[WROBOT] target selection failed: {exc}")
            elif command.startswith("TRACKDIST:"):
                delta = _track_distance_delta(command)
                if tracking and ever_locked and filtered_box is not None and delta != 0:
                    min_height = max(MIN_TARGET_SIZE_PX, int(image_height * TRACK_DISTANCE_MIN_HEIGHT_RATIO))
                    max_height = max(min_height, int(image_height * TRACK_DISTANCE_MAX_HEIGHT_RATIO))
                    if not target_distance_locked:
                        target_height = max(MIN_TARGET_SIZE_PX, int(filtered_box[3]))
                        target_distance_locked = True
                    distance_baseline_frames = 0
                    target_height = max(
                        min_height,
                        min(max_height, int(target_height + delta * TRACK_DISTANCE_ADJUST_STEP_PX)),
                    )
                    print(f"[WROBOT] target distance adjusted: h={target_height}, delta={delta}")

        score = 0.0
        active_profile = requested_profile or _tracking_profile(target_family)
        movement_command = _command(state, profile=active_profile)
        detection_label, detection_count = "NO_TARGET", 0

        if tracking:
            semantic_identity_required = bool(target_family)
            threshold = DETECTOR_REACQUIRE_SCORE_THRESHOLD if state in (COASTING, REACQUIRING) else DETECTOR_SCORE_THRESHOLD
            candidate = None
            semantic_candidate = None
            semantic_checked = False
            semantic_valid_match = False

            recovery_detection_due = (
                state in (ACQUIRING, COASTING, REACQUIRING) and
                frame_count % 2 == 0
            )
            locked_detection_due = (
                semantic_identity_required and
                frame_count % max(1, TRACK_SEMANTIC_VERIFY_EVERY_N_FRAMES) == 0
            )
            should_detect = (
                hybrid_detector and hybrid_detector.ready() and
                (recovery_detection_due or locked_detection_due)
            )
            if should_detect:
                semantic_checked = semantic_identity_required
                if state == REACQUIRING:
                    predicted_box = _predict_box(
                        filtered_box, velocity_x, velocity_y,
                        min(missed_frames, TRACK_COAST_FRAMES),
                        image_width, image_height,
                    )
                    semantic_candidate = hybrid_detector.reacquire_global(
                        img, predicted_box, target_family, target_label
                    )
                elif target_family:
                    semantic_candidate = hybrid_detector.reacquire(
                        img, filtered_box, target_family, target_label
                    )
                else:
                    semantic_candidate = None
                if semantic_candidate is not None:
                    semantic_box = semantic_candidate.box
                    semantic_valid = (
                        state in (ACQUIRING, COASTING, REACQUIRING) or
                        _valid_geometry(semantic_box, filtered_box, image_width, image_height)
                    )
                    if semantic_valid:
                        semantic_valid_match = True
                        if semantic_identity_required:
                            candidate = semantic_box
                            score = max(score, semantic_candidate.score, threshold)
                            semantic_miss_frames = 0
                            semantic_valid_until_ms = now_ms + TRACK_SEMANTIC_VALID_MS
                        elif candidate is None or semantic_candidate.score >= score:
                            candidate = semantic_box
                            score = max(score, semantic_candidate.score, threshold)
                if semantic_checked and state == LOCKED and not semantic_valid_match:
                    semantic_miss_frames += 1
                    print(
                        "[WROBOT] semantic identity miss: "
                        f"family={target_family}, misses={semantic_miss_frames}"
                    )
                    if semantic_miss_frames >= TRACK_SEMANTIC_VERIFY_MISSES:
                        candidate = None
                        score = 0.0
                        state = COASTING
                if (
                    semantic_identity_required and
                    state == LOCKED and
                    (semantic_valid_until_ms == 0 or now_ms > semantic_valid_until_ms)
                ):
                    print(
                        "[WROBOT] semantic identity expired: "
                        f"family={target_family}"
                    )
                    candidate = None
                    score = 0.0
                    state = COASTING
            if (
                candidate is None and semantic_identity_required and
                filtered_box is not None and state == LOCKED and
                semantic_valid_until_ms != 0 and now_ms <= semantic_valid_until_ms
            ):
                candidate = filtered_box
                score = max(score, DETECTOR_SCORE_THRESHOLD)

            if candidate is not None:
                semantic_ready = (
                    (not semantic_identity_required) or
                    (semantic_valid_until_ms != 0 and now_ms <= semantic_valid_until_ms)
                )
                if semantic_identity_required and not semantic_ready:
                    candidate = None
                    score = 0.0
                    state = COASTING
                    stable_frames = 0
                    missed_frames += 1
                else:
                    previous_box = filtered_box
                    filtered_box = _smooth(filtered_box, candidate)
                    if previous_box is not None:
                        previous_center = _center(previous_box)
                        current_center = _center(filtered_box)
                        instant_velocity_x = current_center[0] - previous_center[0]
                        instant_velocity_y = current_center[1] - previous_center[1]
                        velocity_x += (instant_velocity_x - velocity_x) * TRACK_VELOCITY_ALPHA
                        velocity_y += (instant_velocity_y - velocity_y) * TRACK_VELOCITY_ALPHA
                    missed_frames = 0
                    lost_since_ms = 0
                    stable_frames = stable_frames + 1 if score >= threshold else 0
                    if state == ACQUIRING and stable_frames >= TRACK_ACQUIRE_CONFIRM_FRAMES and semantic_ready:
                        state = LOCKED
                        ever_locked = True
                        if not target_distance_locked and filtered_box is not None:
                            target_height = max(MIN_TARGET_SIZE_PX, int(filtered_box[3]))
                            target_distance_locked = True
                            distance_baseline_frames = 0
                            print(f"[WROBOT] target distance baseline: h={target_height}")
                        print("[WROBOT] target locked")
                    elif state in (COASTING, REACQUIRING):
                        if semantic_ready and stable_frames >= TRACK_REACQUIRE_CONFIRM_FRAMES:
                            state = LOCKED
                            ever_locked = True
                            if not target_distance_locked and filtered_box is not None:
                                target_height = max(MIN_TARGET_SIZE_PX, int(filtered_box[3]))
                                target_distance_locked = True
                                print(f"[WROBOT] target distance baseline: h={target_height}")
                            print("[WROBOT] target reacquired")
                        else:
                            state = REACQUIRING
                    elif state != ACQUIRING and semantic_ready:
                        state = LOCKED
                        ever_locked = True

                    if state == LOCKED:
                        if distance_baseline_frames > 0 and filtered_box is not None:
                            target_height = max(
                                MIN_TARGET_SIZE_PX,
                                int(target_height * 0.72 + filtered_box[3] * 0.28),
                            )
                            distance_baseline_frames -= 1
                        dx, dy, dz, ex, ey, ez = _errors(
                            filtered_box, image_center, target_height, velocity_x, velocity_y
                        )
                        movement_command = _command(
                            state, dx, dy, dz, ex, ey, ez, score,
                            active_profile, filtered_box,
                            (image_width, image_height), target_height,
                            missed_frames, stable_frames, velocity_x, velocity_y,
                        )
                        detection_label = _semantic_label(target_family, target_label, state)
                    elif state == ACQUIRING:
                        movement_command = _command(
                            state, score=score, profile=active_profile,
                            box=filtered_box, frame_size=(image_width, image_height),
                            target_height=target_height, missed_frames=missed_frames,
                            stable_frames=stable_frames, velocity_x=velocity_x,
                            velocity_y=velocity_y,
                        )
                        detection_label = "TRACK_CLASSIFYING"
                    else:
                        movement_command = _command(
                            COASTING, score=score,
                            profile=active_profile, box=filtered_box,
                            frame_size=(image_width, image_height),
                            target_height=target_height, missed_frames=missed_frames,
                            stable_frames=stable_frames, velocity_x=velocity_x,
                            velocity_y=velocity_y,
                        )
                        detection_label = "TARGET_OCCLUDED" if semantic_identity_required else _semantic_label(target_family, target_label, state)
                    detection_count = 1

            if candidate is not None:
                pass
            else:
                stable_frames = 0
                velocity_x *= 0.82
                velocity_y *= 0.82
                missed_frames += 1
                if state == LOCKED or lost_since_ms == 0:
                    lost_since_ms = now_ms
                if not ever_locked:
                    state, detection_label = ACQUIRING, "TRACK_ACQUIRING"
                elif missed_frames <= TRACK_COAST_FRAMES and state != ACQUIRING:
                    state, detection_label = COASTING, "TARGET_OCCLUDED"
                else:
                    # Hold posture and re-detect locally. The robot chassis must not
                    # perform blind search turns after a target is lost.
                    state, detection_label = COASTING, "TARGET_OCCLUDED"
                detection_count = 1
                if state == COASTING and filtered_box is not None:
                    predicted_box = _predict_box(
                        filtered_box, velocity_x, velocity_y,
                        min(missed_frames, TRACK_COAST_FRAMES),
                        image_width, image_height,
                    )
                    movement_command = _command(
                        state, score=max(0.0, score - missed_frames * 0.02),
                        profile=active_profile,
                        frame_size=(image_width, image_height),
                        target_height=target_height, missed_frames=missed_frames,
                        stable_frames=stable_frames, velocity_x=velocity_x,
                        velocity_y=velocity_y,
                    )
                else:
                    movement_command = _command(
                        state, score=score,
                        profile=active_profile, box=filtered_box,
                        frame_size=(image_width, image_height),
                        target_height=target_height, missed_frames=missed_frames,
                        stable_frames=stable_frames, velocity_x=velocity_x,
                        velocity_y=velocity_y,
                    )

                if now_ms - lost_since_ms >= TRACK_REACQUIRE_TIMEOUT_MS:
                    tracking, state, target_height = False, LOST, 0
                    ever_locked = False
                    target_distance_locked = False
                    distance_baseline_frames = 0
                    filtered_box = None
                    detection_label, detection_count = "TARGET_LOST", 0
                    movement_command = _command(
                        state, profile=active_profile
                    )
                    report_camera_status(serial_dev, "READY", resources.get("device_ip", ""), "Target lost - select again")
                    print("[WROBOT] target lost after reacquire timeout")

            if filtered_box is not None and state in (LOCKED, COASTING, REACQUIRING):
                x, y, w, h = [int(value) for value in filtered_box]
                color = image.COLOR_GREEN if state == LOCKED else image.COLOR_BLUE
                img.draw_rect(x, y, w, h, color, 3)
                display_label = _semantic_display_label(target_family, target_label)
                img.draw_string(x, max(2, y - 18), f"{state} {display_label} {score:.2f}", color)

        if detection_label != last_label or detection_count != last_count:
            report_camera_detection(serial_dev, detection_label, detection_count)
            last_label, last_count = detection_label, detection_count

        serial_dev.write_str(movement_command)
        if new_ip:
            if preview_server and preview_server.is_ready():
                report_camera_status(serial_dev, "CONNECTED", new_ip, "Camera WiFi connected")
            else:
                report_camera_status(serial_dev, "PREVIEW_FAILED", new_ip, "Camera preview server failed")
        if preview_server:
            preview_server.update_frame(img)
            preview_server.poll()
        disp.show(img)
        time.sleep_ms(1)


