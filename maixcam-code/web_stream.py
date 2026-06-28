import socket
import traceback
import time

from config import (
    WEB_PREVIEW_ENCODE_INTERVAL_MS,
    WEB_PREVIEW_ENABLED,
    WEB_PREVIEW_HOST,
    WEB_PREVIEW_JPEG_QUALITY,
    WEB_PREVIEW_PORT,
    WEB_PREVIEW_PROFILES,
    WEB_PREVIEW_SNAPSHOT_PATH,
    WEB_PREVIEW_STREAM_PATH,
    WEB_PREVIEW_TEMP_FILE,
)


def _build_preview_html(resolution):
    aspect_ratio = "16/9" if resolution == "1280x720" else "4/3"
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>WRobot MaixCam Preview</title>
  <style>
    body{{margin:0;background:#0d1117;color:#e6edf3;font:15px sans-serif;display:grid;place-items:center;min-height:100vh}}
    main{{width:min(96vw,900px);display:grid;gap:12px}}
    h1{{font-size:20px;margin:0}}
    .frame{{background:#000;border:1px solid #30363d;border-radius:8px;overflow:hidden;aspect-ratio:{aspect_ratio}}}
    img{{display:block;width:100%;height:100%;object-fit:contain}}
    p{{margin:0;color:#8b949e}}
  </style>
</head>
<body>
  <main>
    <h1>WRobot MaixCam Preview</h1>
    <div class="frame"><img id="feed" src="{WEB_PREVIEW_STREAM_PATH}" alt="preview"></div>
    <p>MJPEG preview stream</p>
  </main>
</body>
</html>
""".encode("utf-8")


class SnapshotServer:
    def __init__(self, resolution="640x480"):
        profile = WEB_PREVIEW_PROFILES.get(resolution, {})
        self.enabled = WEB_PREVIEW_ENABLED
        self.host = WEB_PREVIEW_HOST
        self.port = WEB_PREVIEW_PORT
        self.snapshot_path = WEB_PREVIEW_SNAPSHOT_PATH
        self.stream_path = WEB_PREVIEW_STREAM_PATH
        self.encode_interval_ms = int(
            profile.get("interval_ms", WEB_PREVIEW_ENCODE_INTERVAL_MS)
        )
        self.jpeg_quality = int(
            profile.get("jpeg_quality", WEB_PREVIEW_JPEG_QUALITY)
        )
        self._server = None
        self._latest_jpeg = None
        self._html = _build_preview_html(resolution)
        self._ready = False
        self._bound_host = ""
        self._bound_port = self.port
        self._last_encode_ms = 0
        self._stream_client = None
        self._stream_boundary = b"--frame"
        self._pending_stream_data = None
        self._pending_stream_offset = 0
        self._next_stream_data = None
        self._command_handler = None
        print(
            f"[WEB] profile: {resolution}, interval={self.encode_interval_ms}ms, "
            f"quality={self.jpeg_quality}"
        )

    def set_command_handler(self, handler):
        self._command_handler = handler

    def start(self, device_ip=""):
        if not self.enabled:
            print("[WEB] Preview disabled")
            self._ready = False
            return False
        if self._server is not None:
            self._ready = True
            return True
        # Bind the active LAN address first so the preview is not exposed on
        # every interface. This server never initiates an outbound stream.
        bind_hosts = []
        if device_ip:
            bind_hosts.append(device_ip)
        if self.host and self.host not in bind_hosts:
            bind_hosts.append(self.host)
        last_error = None
        for bind_host in bind_hosts:
            server = None
            try:
                print(f"[WEB] trying bind on {bind_host}:{self.port}")
                server = socket.socket()
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((bind_host, self.port))
                server.listen(2)
                server.setblocking(False)
                self._server = server
                self._ready = True
                self._bound_host = bind_host
                self._bound_port = self.port
                print(f"[WEB] Preview server listening on {bind_host}:{self.port}")
                print(f"[WEB] Snapshot path: {self.snapshot_path}")
                print(f"[WEB] Stream path: {self.stream_path}")
                if device_ip:
                    print(f"[WEB] Preview page: http://{device_ip}:{self.port}/")
                    print(f"[WEB] Snapshot URL: http://{device_ip}:{self.port}{self.snapshot_path}")
                    print(f"[WEB] Stream URL: http://{device_ip}:{self.port}{self.stream_path}")
                return True
            except Exception as e:
                last_error = e
                print(f"[WEB] bind failed on {bind_host}:{self.port}: {e}")
                traceback.print_exc()
                try:
                    if server is not None:
                        server.close()
                except Exception:
                    pass
                self._server = None
                self._ready = False
        print(f"[WEB] Preview server start failed: {last_error}")
        return False

    def update_frame(self, img):
        if not self.enabled:
            return
        now_ms = int(time.time() * 1000)
        if (
            self._latest_jpeg is not None
            and now_ms - self._last_encode_ms < self.encode_interval_ms
        ):
            return
        try:
            self._latest_jpeg = self._encode_jpeg(img)
            self._last_encode_ms = now_ms
            self._queue_stream_frame()
        except Exception as e:
            print(f"[WEB] JPEG encode failed: {e}")

    def poll(self):
        self._flush_stream_frame()
        if self._server is None:
            return
        try:
            client, _ = self._server.accept()
        except BlockingIOError:
            return
        except Exception as e:
            print(f"[WEB] Accept failed: {e}")
            return

        keep_open = False
        try:
            # Keep network clients from stalling the tracking loop.
            client.settimeout(0.03)
            request = client.recv(1024)
            path = self._parse_path(request)
            if path == "/" or path == "":
                self._send_response(client, 200, b"text/html; charset=utf-8", self._html)
            elif path == "/ping":
                self._send_response(client, 200, b"text/plain; charset=utf-8", b"pong")
            elif path.startswith("/api/command"):
                self._handle_command(client, path)
            elif path.startswith(self.stream_path):
                self._attach_stream_client(client)
                keep_open = True
            elif path.startswith(self.snapshot_path):
                self._send_snapshot(client)
            else:
                self._send_response(client, 404, b"text/plain; charset=utf-8", b"Not Found")
        except Exception as e:
            print(f"[WEB] Request handle failed: {e}")
        finally:
            if not keep_open:
                try:
                    client.close()
                except Exception:
                    pass

    def _send_snapshot(self, client):
        if not self._latest_jpeg:
            self._send_response(client, 503, b"text/plain; charset=utf-8", b"No Video Signal")
            return
        self._send_response(client, 200, b"image/jpeg", self._latest_jpeg)

    def _handle_command(self, client, path):
        command = self._query_value(path, "cmd")
        if not command:
            self._send_response(client, 400, b"application/json", b'{"ok":false,"error":"missing cmd"}')
            return
        if self._command_handler is None:
            self._send_response(client, 503, b"application/json", b'{"ok":false,"error":"handler unavailable"}')
            return
        try:
            self._command_handler(command)
            self._send_response(client, 200, b"application/json", b'{"ok":true}')
        except Exception as exc:
            print(f"[WEB] command failed: {exc}")
            self._send_response(client, 500, b"application/json", b'{"ok":false,"error":"command failed"}')

    def _attach_stream_client(self, client):
        self._close_stream_client()
        header = (
            b"HTTP/1.1 200 OK\r\n"
            + b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            + b"Cache-Control: no-store, no-cache, must-revalidate\r\n"
            + b"Pragma: no-cache\r\n"
            + b"Connection: close\r\n\r\n"
        )
        client.sendall(header)
        client.setblocking(False)
        self._stream_client = client
        self._pending_stream_data = None
        self._pending_stream_offset = 0
        self._next_stream_data = None
        if self._latest_jpeg:
            self._queue_stream_frame()

    def _build_stream_frame(self):
        if not self._latest_jpeg:
            return None
        return (
            self._stream_boundary + b"\r\n"
            + b"Content-Type: image/jpeg\r\n"
            + b"Content-Length: " + str(len(self._latest_jpeg)).encode("ascii")
            + b"\r\n\r\n"
            + self._latest_jpeg + b"\r\n"
        )

    def _queue_stream_frame(self):
        if self._stream_client is None:
            return
        body = self._build_stream_frame()
        if body is None:
            return
        if self._pending_stream_data is None:
            self._pending_stream_data = body
            self._pending_stream_offset = 0
        else:
            # Keep only the newest complete frame waiting behind the current send.
            self._next_stream_data = body
        self._flush_stream_frame()

    def _flush_stream_frame(self):
        if self._stream_client is None or self._pending_stream_data is None:
            return
        try:
            remaining = self._pending_stream_data[self._pending_stream_offset:]
            sent = self._stream_client.send(remaining)
            if sent is None:
                sent = 0
            self._pending_stream_offset += sent
            if self._pending_stream_offset >= len(self._pending_stream_data):
                self._pending_stream_data = self._next_stream_data
                self._next_stream_data = None
                self._pending_stream_offset = 0
        except BlockingIOError:
            return
        except Exception:
            self._close_stream_client()

    def _close_stream_client(self):
        if self._stream_client is not None:
            try:
                self._stream_client.close()
            except Exception:
                pass
        self._stream_client = None
        self._pending_stream_data = None
        self._pending_stream_offset = 0
        self._next_stream_data = None

    def _send_response(self, client, status_code, content_type, body):
        reason = b"OK" if status_code == 200 else b"ERROR"
        header = (
            b"HTTP/1.1 " + str(status_code).encode("ascii") + b" " + reason + b"\r\n"
            + b"Content-Type: " + content_type + b"\r\n"
            + b"Content-Length: " + str(len(body)).encode("ascii") + b"\r\n"
            + b"Cache-Control: no-store\r\n"
            + b"Connection: close\r\n\r\n"
        )
        client.sendall(header + body)

    def _parse_path(self, request_bytes):
        if not request_bytes:
            return "/"
        try:
            first_line = request_bytes.decode("utf-8", "ignore").split("\r\n", 1)[0]
            parts = first_line.split(" ")
            if len(parts) >= 2:
                return parts[1]
        except Exception:
            pass
        return "/"

    def _query_value(self, path, name):
        try:
            _, _, query = path.partition("?")
            for item in query.split("&"):
                key, separator, value = item.partition("=")
                if separator and _url_decode(key) == name:
                    return _url_decode(value).strip()
        except Exception:
            pass
        return ""

    def _encode_jpeg(self, img):
        for method_name in ("to_jpeg", "to_jpg"):
            method = getattr(img, method_name, None)
            if callable(method):
                try:
                    data = method(self.jpeg_quality)
                except TypeError:
                    data = method()
                if isinstance(data, (bytes, bytearray)):
                    return bytes(data)

        save = getattr(img, "save", None)
        if callable(save):
            try:
                save(WEB_PREVIEW_TEMP_FILE, quality=self.jpeg_quality)
            except TypeError:
                try:
                    save(WEB_PREVIEW_TEMP_FILE, self.jpeg_quality)
                except TypeError:
                    save(WEB_PREVIEW_TEMP_FILE)
            with open(WEB_PREVIEW_TEMP_FILE, "rb") as f:
                return f.read()

        raise RuntimeError("No JPEG export method available on this firmware")

    def is_ready(self):
        return self._ready and self._server is not None

    def stop(self):
        self._close_stream_client()
        if self._server is not None:
            try:
                self._server.close()
            except Exception:
                pass
        self._server = None
        self._ready = False


def _url_decode(value):
    value = value.replace("+", " ")
    output = []
    index = 0
    while index < len(value):
        if (
            value[index] == "%"
            and index + 2 < len(value)
        ):
            try:
                output.append(chr(int(value[index + 1:index + 3], 16)))
                index += 3
                continue
            except Exception:
                pass
        output.append(value[index])
        index += 1
    return "".join(output)
