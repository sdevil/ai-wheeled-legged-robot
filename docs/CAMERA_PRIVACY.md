# Camera Privacy Audit

The MaixCam application was reviewed for outbound network behavior.

## Findings

- No HTTP or HTTPS client is present.
- No MQTT, WebSocket client, cloud SDK, analytics, telemetry, upload, `curl`, or `wget` code is present.
- Camera frames are encoded locally and served only by the local MJPEG server on TCP port 8080.
- The preview server binds the current camera LAN address rather than all network interfaces.
- `wifi_manager.py` opens a UDP socket to `192.168.8.1` only to ask the operating system which local source address would be used. It sends no camera data.
- WiFi credentials are stored locally in `/root/wrobot_wifi.json` with owner-only file permissions and are not returned by the preview server.

## Local Network Boundary

There is currently no login on the MaixCam port 8080 preview endpoint. Any device that can reach the camera IP on the same trusted LAN may view the stream. The application does not configure router port forwarding and does not upload the stream to the Internet. Do not expose port 8080 through router forwarding, a public WiFi network, or a reverse proxy.
