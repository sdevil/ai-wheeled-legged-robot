import os
import socket
import traceback

from maix import time

from config import (
    FORCE_ROBOT_AP_MODE,
    WIFI_AUTO_CONNECT,
    WIFI_CONNECT_TIMEOUT_SEC,
    WIFI_PASSWORD,
    WIFI_SSID,
)


def _current_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.8.1", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip and not ip.startswith("127."):
            return ip
    except Exception:
        pass

    try:
        hostname = socket.gethostname()
        for item in socket.getaddrinfo(hostname, None):
            ip = item[4][0]
            if "." in ip and not ip.startswith("127."):
                return ip
    except Exception:
        pass
    return ""


def _wait_for_ip(timeout_sec):
    start = time.time_ms()
    timeout_ms = int(timeout_sec * 1000)
    while time.time_ms() - start < timeout_ms:
        ip = _current_ip()
        if ip:
            return ip
        time.sleep_ms(500)
    return ""


def _shell_quote(value):
    return "'" + str(value).replace("'", "'\\''") + "'"


def _run_shell(command, log_command=None):
    try:
        print(f"[WIFI] shell: {log_command or command}")
        output = os.popen(command).read().strip()
        if output:
            print(output)
        return output
    except Exception as e:
        print(f"[WIFI] shell failed: {e}")
        return ""

def _current_ssid():
    commands = [
        "iwgetid -r",
        "wpa_cli -i wlan0 status | awk -F= '$1 == \"ssid\" {print substr($0, 6)}'",
    ]
    for command in commands:
        output = _run_shell(command)
        if not output:
            continue
        return output.splitlines()[0].strip()
    return ""

def _disconnect_existing_wifi():
    try:
        from maix import network as maix_network
        wifi = getattr(maix_network, "wifi", None)
        if wifi and hasattr(wifi, "disconnect"):
            try:
                wifi.disconnect()
                print("[WIFI] disconnected existing maix.network connection")
            except Exception:
                pass
    except Exception:
        pass

    try:
        import network
        if hasattr(network, "STA_IF"):
            sta_if = network.WLAN(network.STA_IF)
        else:
            sta_if = network.WLAN(network.WLAN.IF_STA)
        if hasattr(sta_if, "disconnect"):
            try:
                sta_if.disconnect()
                print("[WIFI] disconnected existing network.WLAN connection")
            except Exception:
                pass
    except Exception:
        pass

    _run_shell("nmcli dev disconnect wlan0")
    _run_shell("wpa_cli -i wlan0 disconnect")
    time.sleep_ms(1000)


def _connect_with_maix_network(ssid, password, timeout_sec):
    try:
        from maix import network as maix_network
    except Exception:
        return ""

    try:
        wifi = getattr(maix_network, "wifi", None)
        if wifi and hasattr(wifi, "connect"):
            print(f"[WIFI] connecting via maix.network to {ssid}")
            try:
                wifi.connect(ssid, password)
            except TypeError:
                wifi.connect(ssid=ssid, password=password)

            start = time.time_ms()
            timeout_ms = int(timeout_sec * 1000)
            while time.time_ms() - start < timeout_ms:
                is_connected = False
                if hasattr(wifi, "is_connected"):
                    is_connected = bool(wifi.is_connected())
                elif hasattr(wifi, "connected"):
                    is_connected = bool(wifi.connected())
                if is_connected:
                    if hasattr(wifi, "get_ip"):
                        ip = wifi.get_ip()
                        if ip:
                            return ip
                    return _wait_for_ip(3)
                time.sleep_ms(500)
    except Exception as e:
        print(f"[WIFI] maix.network connect failed: {e}")
        traceback.print_exc()
    return ""


def _connect_with_micropython_network(ssid, password, timeout_sec):
    try:
        import network
    except Exception:
        return ""

    try:
        if hasattr(network, "STA_IF"):
            sta_if = network.WLAN(network.STA_IF)
        else:
            sta_if = network.WLAN(network.WLAN.IF_STA)
        sta_if.active(True)
        if not sta_if.isconnected():
            print(f"[WIFI] connecting via network.WLAN to {ssid}")
            sta_if.connect(ssid, password)
            start = time.time_ms()
            timeout_ms = int(timeout_sec * 1000)
            while time.time_ms() - start < timeout_ms:
                if sta_if.isconnected():
                    break
                time.sleep_ms(500)
        if sta_if.isconnected():
            config = sta_if.ifconfig()
            if config and len(config) > 0:
                return config[0]
    except Exception as e:
        print(f"[WIFI] network.WLAN connect failed: {e}")
        traceback.print_exc()
    return ""


def _connect_with_linux_commands(ssid, password):
    if not ssid:
        return ""

    quoted_ssid = _shell_quote(ssid)
    quoted_password = _shell_quote(password)
    nmcli_command = f"nmcli dev wifi connect {quoted_ssid}"
    wpa_security_command = "wpa_cli -i wlan0 set_network \"$network_id\" key_mgmt NONE"
    if password:
        nmcli_command += f" password {quoted_password}"
        wpa_security_command = (
            "wpa_cli -i wlan0 set_network \"$network_id\" psk "
            + _shell_quote(chr(34) + password + chr(34))
        )
    commands = [
        (nmcli_command, f"nmcli dev wifi connect {quoted_ssid} password <redacted>"),
        (
            "network_id=$(wpa_cli -i wlan0 add_network | tail -n 1) && "
            f"wpa_cli -i wlan0 set_network \"$network_id\" ssid {_shell_quote(chr(34) + ssid + chr(34))} && "
            f"{wpa_security_command} && "
            "wpa_cli -i wlan0 enable_network \"$network_id\" && "
            "wpa_cli -i wlan0 save_config && wpa_cli -i wlan0 reconnect",
            "wpa_cli -i wlan0 configure network <ssid> <redacted>",
        ),
    ]

    for command, safe_log in commands:
        output = _run_shell(command, safe_log)
        ip = _wait_for_ip(6)
        if ip:
            return ip
        if output:
            print(f"[WIFI] command completed without IP: {safe_log}")
    return ""


def ensure_wifi_connected(ssid=None, password=None):
    ssid = WIFI_SSID if ssid is None else ssid
    password = WIFI_PASSWORD if password is None else password

    if not WIFI_AUTO_CONNECT:
        print("[WIFI] auto connect disabled")
        return _current_ip()

    if not ssid:
        print("[WIFI] WIFI_SSID is empty, skip auto connect")
        return _current_ip()

    existing_ip = _current_ip()
    existing_ssid = _current_ssid()
    if existing_ip:
        if not existing_ssid:
            print(f"[WIFI] active connection has IP {existing_ip}; SSID unavailable, preserving connection")
            return existing_ip
        if existing_ssid == ssid:
            print(f"[WIFI] already connected to target SSID {ssid}: {existing_ip}")
            return existing_ip
        print(f"[WIFI] connected to different SSID '{existing_ssid}' at {existing_ip}, reconnecting to {ssid}")
        _disconnect_existing_wifi()

    print(f"[WIFI] trying to connect to {ssid}")
    strategies = (
        lambda: _connect_with_maix_network(ssid, password, WIFI_CONNECT_TIMEOUT_SEC),
        lambda: _connect_with_micropython_network(ssid, password, WIFI_CONNECT_TIMEOUT_SEC),
        lambda: _connect_with_linux_commands(ssid, password),
    )
    for strategy in strategies:
        ip = strategy()
        if ip:
            print(f"[WIFI] connected: {ip}")
            return ip

    print("[WIFI] failed to get WiFi connection")
    return ""
