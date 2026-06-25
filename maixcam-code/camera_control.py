import json
import os
import traceback

from maix import app, time

import config
from wifi_manager import ensure_wifi_connected

WIFI_CONFIG_FILE = "/root/wrobot_wifi.json"
CAMERA_CONFIG_FILE = "/root/wrobot_camera.json"
CAMERA_CONFIG_VERSION = 3
_uart_buffer = ""
FORCE_ROBOT_AP_MODE = getattr(config, "FORCE_ROBOT_AP_MODE", False)
ROBOT_AP_SSID = getattr(config, "ROBOT_AP_SSID", "sdevil-WRobot")
ROBOT_AP_PASSWORD = getattr(config, "ROBOT_AP_PASSWORD", "wrobot123")
WIFI_SSID = getattr(config, "WIFI_SSID", "")
WIFI_PASSWORD = getattr(config, "WIFI_PASSWORD", "")
DEFAULT_CAMERA_RESOLUTION = getattr(config, "DEFAULT_CAMERA_RESOLUTION", "640x480")
CAMERA_RESOLUTION_PRESETS = getattr(
    config,
    "CAMERA_RESOLUTION_PRESETS",
    {"160x120": (160, 120), "320x240": (320, 240), "640x480": (640, 480), "1280x720": (1280, 720)},
)


def _percent_decode(text):
    result = bytearray()
    i = 0
    while i < len(text):
        if text[i] == "%" and i + 2 < len(text):
            try:
                result.append(int(text[i + 1:i + 3], 16))
                i += 3
                continue
            except Exception:
                pass
        result.extend(text[i].encode("utf-8"))
        i += 1
    return bytes(result).decode("utf-8", "replace")


def _atomic_write_json(path, data):
    temporary_path = path + ".tmp"
    with open(temporary_path, "w", encoding="utf-8") as file:
        json.dump(data, file)
        file.flush()
        os.fsync(file.fileno())
    os.chmod(temporary_path, 0o600)
    os.replace(temporary_path, path)
    os.sync()


def load_saved_wifi_config():
    if FORCE_ROBOT_AP_MODE:
        return ROBOT_AP_SSID, ROBOT_AP_PASSWORD
    try:
        with open(WIFI_CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            return data.get("ssid", WIFI_SSID), data.get("password", WIFI_PASSWORD)
    except Exception:
        return WIFI_SSID, WIFI_PASSWORD


def save_wifi_config(ssid, password):
    if FORCE_ROBOT_AP_MODE:
        return
    _atomic_write_json(WIFI_CONFIG_FILE, {"ssid": ssid, "password": password})


def normalize_camera_resolution(value):
    resolution = str(value or "").strip()
    if resolution in CAMERA_RESOLUTION_PRESETS:
        return resolution
    return DEFAULT_CAMERA_RESOLUTION


def load_saved_camera_resolution():
    try:
        with open(CAMERA_CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            if data.get("version") != CAMERA_CONFIG_VERSION:
                save_camera_resolution(DEFAULT_CAMERA_RESOLUTION)
                return DEFAULT_CAMERA_RESOLUTION
            return normalize_camera_resolution(data.get("resolution", DEFAULT_CAMERA_RESOLUTION))
    except Exception:
        return DEFAULT_CAMERA_RESOLUTION


def save_camera_resolution(resolution):
    normalized = normalize_camera_resolution(resolution)
    _atomic_write_json(
        CAMERA_CONFIG_FILE,
        {"version": CAMERA_CONFIG_VERSION, "resolution": normalized},
    )
    return normalized


def request_app_restart():
    os.sync()
    for command in ("reboot -f", "reboot"):
        try:
            result = os.system(command)
            print(f"[CAMCTRL] restart command '{command}' returned: {result}")
            if result in (None, 0):
                return
        except Exception as exc:
            print(f"[CAMCTRL] restart command '{command}' failed: {exc}")

    try:
        app.set_exit_flag(True)
        print("[CAMCTRL] requested Maix app exit for restart")
        return
    except Exception as exc:
        print(f"[CAMCTRL] app exit request unavailable: {exc}")

    raise RuntimeError("unable to request camera application restart")


def report_camera_status(serial_dev, state, ip="", message="", resolution=""):
    app_version = str(getattr(config, "APP_VERSION", "unknown")).replace(",", "_").replace(";", "_")
    build_id = str(getattr(config, "BUILD_ID", "")).replace(",", "_").replace(";", "_")
    parts = [f"CAMSTAT:{state}", f"VER:{app_version}"]
    if build_id:
        parts.append(f"BUILD:{build_id}")
    normalized_resolution = str(resolution or "").strip()
    if normalized_resolution in CAMERA_RESOLUTION_PRESETS:
        parts.append(f"RES:{normalized_resolution}")
    if ip:
        parts.append(f"IP:{ip}")
    if message:
        safe_message = message.replace(",", " ").replace(";", " ")
        parts.append(f"MSG:{safe_message}")
    payload = ",".join(parts) + ";"
    try:
        serial_dev.write_str(payload)
    except Exception as e:
        print(f"[CAMCTRL] status send failed: {e}")


def report_camera_detection(serial_dev, label, count=0):
    safe_label = str(label or "NO_TARGET").replace(",", "_").replace(";", "_").replace(" ", "_")
    payload = f"CAMDET:{safe_label},COUNT:{int(count)};"
    try:
        serial_dev.write_str(payload)
    except Exception as e:
        print(f"[CAMCTRL] detection send failed: {e}")


def _read_serial_chunk(serial_dev):
    available = getattr(serial_dev, "available", None)
    pending = None
    if callable(available):
        try:
            pending = available()
        except Exception:
            pending = None

    if pending is not None and pending <= 0:
        return ""

    read = getattr(serial_dev, "read", None)
    if pending is not None and callable(read):
        try:
            data = read(min(int(pending), 1024))
            if isinstance(data, bytes):
                return data.decode("utf-8", "ignore")
            return data
        except Exception:
            pass

    if pending is None:
        return ""

    for method_name in ("read_str", "readline"):
        method = getattr(serial_dev, method_name, None)
        if callable(method):
            try:
                data = method()
                if data:
                    return data
            except TypeError:
                try:
                    data = method(min(pending, 256))
                    if data:
                        return data
                except Exception:
                    pass
            except Exception:
                pass
    return ""


def handle_incoming_camera_commands(
    serial_dev,
    preview_server=None,
    track_command_handler=None,
    active_resolution=None,
):
    global _uart_buffer
    chunk = _read_serial_chunk(serial_dev)
    if not chunk and not _uart_buffer:
        return None

    if isinstance(chunk, bytes):
        chunk = chunk.decode("utf-8", "ignore")
    _uart_buffer += chunk
    if len(_uart_buffer) > 4096 and not any(
        delimiter in _uart_buffer for delimiter in (";", "\n", "\r")
    ):
        print("[CAMCTRL] discarded oversized UART frame")
        _uart_buffer = ""
        return None

    while True:
        split_positions = [pos for pos in (_uart_buffer.find(";"), _uart_buffer.find("\n"), _uart_buffer.find("\r")) if pos >= 0]
        if not split_positions:
            break
        end = min(split_positions)
        command = _uart_buffer[:end].strip()
        _uart_buffer = _uart_buffer[end + 1:]
        if len(command) > 1024:
            print("[CAMCTRL] discarded oversized UART command")
            continue
        if not command:
            continue
        if command.startswith("WIFI:") or command.startswith("CAMWIFI:"):
            return apply_wifi_command(serial_dev, command, preview_server)
        if command.startswith("CAMCFG:"):
            apply_camera_config_command(serial_dev, command, active_resolution)
            continue
        if (
            command.startswith("TRACKROI:")
            or command.startswith("TRACKDIST:")
            or command == "TRACKSTOP"
        ):
            if track_command_handler:
                track_command_handler(command)
    return None


def apply_wifi_command(serial_dev, command, preview_server=None):
    try:
        payload = command.split(":", 1)[1]
        values = {}
        for item in payload.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            values[key.strip().lower()] = _percent_decode(value.strip())

        ssid = values.get("ssid", "")
        password = values.get("pwd", "")
        if FORCE_ROBOT_AP_MODE:
            ssid = ROBOT_AP_SSID
            password = ROBOT_AP_PASSWORD
        if not ssid:
            report_camera_status(serial_dev, "INVALID", "", "SSID is empty")
            return None

        save_wifi_config(ssid, password)
        report_camera_status(serial_dev, "CONNECTING", "", "Applying camera WiFi")
        ip = ensure_wifi_connected(ssid, password)
        if ip:
            preview_ready = True
            if preview_server is not None:
                preview_ready = preview_server.start(ip)
            if preview_ready:
                report_camera_status(serial_dev, "CONNECTED", ip, "Camera WiFi connected")
            else:
                report_camera_status(serial_dev, "PREVIEW_FAILED", ip, "Camera preview server failed")
            return ip

        report_camera_status(serial_dev, "FAILED", "", "Camera WiFi connect failed")
    except Exception as e:
        print(f"[CAMCTRL] command failed: {e}")
        traceback.print_exc()
        report_camera_status(serial_dev, "FAILED", "", "Camera WiFi command error")
    return None


def apply_camera_config_command(serial_dev, command, active_resolution=None):
    try:
        payload = command[7:]
        values = {}
        for item in payload.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            values[key.strip().lower()] = _percent_decode(value.strip())

        resolution = normalize_camera_resolution(
            values.get("res", DEFAULT_CAMERA_RESOLUTION)
        )
        previous_resolution = load_saved_camera_resolution()
        save_camera_resolution(resolution)
        current_resolution = normalize_camera_resolution(active_resolution)

        if resolution == previous_resolution and resolution == current_resolution:
            report_camera_status(
                serial_dev,
                "CONFIGURED",
                "",
                f"Camera resolution already active: {resolution}",
                current_resolution,
            )
            return

        report_camera_status(
            serial_dev,
            "RESTARTING",
            "",
            f"Applying camera resolution: {resolution}",
        )
        print(
            "[CAMCTRL] resolution apply requested: "
            f"saved={previous_resolution}, active={current_resolution}, target={resolution}"
        )
        time.sleep_ms(350)
        request_app_restart()
    except Exception as e:
        print(f"[CAMCTRL] camera config failed: {e}")
        traceback.print_exc()
        report_camera_status(serial_dev, "FAILED", "", "Camera config error")
