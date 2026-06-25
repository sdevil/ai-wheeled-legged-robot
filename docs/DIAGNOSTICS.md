# Diagnostics

The mainboard exposes a read-only `GET /api/diagnostics` endpoint for runtime troubleshooting.

The snapshot includes system memory, reset reason, battery, WiFi, camera, motion references, tracking inputs and outputs, OTA state, and the latest 24 diagnostic events. The event buffer has fixed storage and does not allocate memory in the real-time control loop.

Open **Settings > Advanced Maintenance > Diagnostics** to refresh once, enable live updates, or copy the complete JSON report. Live updates are disabled by default to avoid unnecessary network traffic.

For tracking investigations, capture a report while the target is locked and another immediately after the failure. Important fields are `tracking.state`, `confidence`, `error_x`, `error_y`, `error_z`, `yaw_command`, `drive_command`, camera angle, and the event timeline.

USB serial monitoring is not recommended while MaixCam is connected because both interfaces share the mainboard UART0.

