"""WRobot MaixCam application entry point."""

import os
import traceback

from maix import app, camera, display, err, pinmap, time, uart

from camera_control import (
    load_saved_camera_resolution,
    load_saved_wifi_config,
    report_camera_status,
)
from config import (
    APP_VERSION,
    BUILD_ID,
    CAMERA_FRAME_HEIGHT,
    CAMERA_FRAME_WIDTH,
    CAMERA_RESOLUTION_PRESETS,
    FORCE_ROBOT_AP_MODE,
    WIFI_SSID,
    init_led,
    update_led,
)
from object_tracking import create_hybrid_detector, run_object_tracking
from web_stream import SnapshotServer
from wifi_manager import ensure_wifi_connected

AUTO_START_FILE = "/maixapp/auto_start.txt"
AUTO_START_APP_ID = "wrobot_sdevil"


def ensure_autostart():
    try:
        current = ""
        if os.path.exists(AUTO_START_FILE):
            with open(AUTO_START_FILE, "r", encoding="utf-8") as file:
                current = file.readline().strip()
        if current == AUTO_START_APP_ID:
            print(f"[WROBOT] autostart already set: {current}")
            return

        with open(AUTO_START_FILE, "w", encoding="utf-8") as file:
            file.write(AUTO_START_APP_ID)
        os.sync()
        print(f"[WROBOT] autostart set to: {AUTO_START_APP_ID}")
    except Exception as exc:
        print(f"[WROBOT] autostart setup failed: {exc}")


def init_system():
    """Initialize camera, display, UART, WiFi, and preview services."""
    init_led()
    ensure_autostart()
    print(f"[WROBOT] version: {APP_VERSION}")
    print(f"[WROBOT] build: {BUILD_ID}")
    print(f"[WROBOT] force_robot_ap_mode: {FORCE_ROBOT_AP_MODE}")
    print(f"[WROBOT] target_ssid: {WIFI_SSID}")

    active_resolution = load_saved_camera_resolution()
    img_width, img_height = CAMERA_RESOLUTION_PRESETS.get(
        active_resolution,
        (CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT),
    )
    print(f"[WROBOT] camera_resolution: {active_resolution}")
    print(f"[WROBOT] image_size: {img_width}x{img_height}")
    image_center = (img_width // 2, img_height // 2)

    hybrid_detector = create_hybrid_detector()
    cam = camera.Camera(img_width, img_height)
    disp = display.Display()

    pin_functions = {"A19": "UART1_TX", "A18": "UART1_RX"}
    for pin, function in pin_functions.items():
        err.check_raise(
            pinmap.set_pin_function(pin, function),
            f"Failed to set {pin} function to {function}",
        )
    serial_dev = uart.UART("/dev/ttyS1", 115200)

    initial_command = "DX:0,DY:0;"
    serial_dev.write_str(initial_command)
    print(f"[WROBOT] initial_offset: {initial_command}")
    time.sleep(1)

    wifi_ssid, wifi_password = load_saved_wifi_config()
    print(f"[WROBOT] effective_ssid: {wifi_ssid}")
    device_ip = ensure_wifi_connected(wifi_ssid, wifi_password)
    print(f"[WROBOT] device_ip: {device_ip}")

    preview_server = SnapshotServer(active_resolution)
    preview_ready = preview_server.start(device_ip)
    preview_url = f"http://{device_ip}:8080/snapshot.jpg" if device_ip else ""
    print(f"[WROBOT] preview_ready: {preview_ready}")
    if preview_url:
        print(f"[WROBOT] preview_url: {preview_url}")
        print(f"[WROBOT] preview_page: http://{device_ip}:8080/")
    else:
        print("[WROBOT] preview_url: unavailable")

    if device_ip and preview_ready:
        report_camera_status(
            serial_dev, "CONNECTED", device_ip, "Camera WiFi connected", active_resolution
        )
    elif device_ip:
        report_camera_status(
            serial_dev,
            "PREVIEW_FAILED",
            device_ip,
            "Camera preview server failed",
            active_resolution,
        )
    else:
        report_camera_status(
            serial_dev, "FAILED", "", "Camera WiFi unavailable", active_resolution
        )

    return {
        "cam": cam,
        "hybrid_detector": hybrid_detector,
        "disp": disp,
        "serial_dev": serial_dev,
        "preview_server": preview_server,
        "device_ip": device_ip,
        "active_resolution": active_resolution,
        "img_width": img_width,
        "img_height": img_height,
        "image_center": image_center,
    }


system_resources = None
try:
    system_resources = init_system()
    update_led()
    run_object_tracking(system_resources)
except Exception as exc:
    print(f"[WROBOT] fatal_error: {exc}")
    traceback.print_exc()
finally:
    if system_resources:
        preview_server = system_resources.get("preview_server")
        if preview_server:
            preview_server.stop()
        for resource_name in ("serial_dev", "cam", "disp"):
            resource = system_resources.get(resource_name)
            close = getattr(resource, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
