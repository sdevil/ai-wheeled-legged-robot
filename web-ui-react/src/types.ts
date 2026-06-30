export type TransportMode = 'auto' | 'http' | 'ws';
export type UiLanguage = 'en' | 'zh';
export type DiagnosticsSnapshot = Record<string, unknown>;

export interface RobotStatus {
  boot_id: number;
  firmware_version: string;
  firmware_build: string;
  control_owner_present?: boolean;
  control_owner?: boolean;
  camera_firmware_version: string;
  camera_firmware_build: string;
  camera_status_age_ms?: number;
  camera_version_stale?: boolean;
  enabled: boolean;
  sitting: boolean;
  gamepad: boolean;
  maintenance: boolean;
  ota_running: boolean;
  angle: number;
  battery_voltage: number;
  battery_percent: number;
  leg_height_percent?: number;
  leg_lean_percent?: number;
  ota_progress: number;
  ota_message: string;
  robot_net_state: string;
  robot_sta_ip: string;
  camera_net_state: string;
  camera_net_ip: string;
  camera_net_message: string;
  camera_url?: string;
  camera_resolution: string;
  guard_angle?: number;
  camera_detect_label: string;
  camera_detect_count: number;
  active_mode: string;
  last_event: string;
  clients: number;
}

export interface DashboardModel {
  bootId: number;
  firmwareVersion: string;
  firmwareBuild: string;
  cameraFirmwareVersion: string;
  cameraFirmwareBuild: string;
  cameraStatusAgeMs: number;
  cameraVersionStale: boolean;
  online: boolean;
  robotHost: string;
  robotIp: string;
  robotState: string;
  robotNetState: string;
  batteryPercent: number;
  batteryVoltage: number;
  legHeightPercent: number;
  legLeanPercent: number;
  guardServoAngle: number;
  wifiDbm: number;
  fps: number;
  latencyMs: number;
  aiMode: string;
  targetLabelRaw: string;
  targetLabel: string;
  speedMps: number;
  pitch: number;
  roll: number;
  temperatureC: number;
  cameraState: string;
  cameraMessage: string;
  cameraResolution: string;
  cameraIp: string;
  cameraUrl: string;
  clients: number;
  controlOwnerPresent: boolean;
  controlOwner: boolean;
  otaRunning: boolean;
  otaProgress: number;
  otaMessage: string;
  oledFace: string;
  activeMode: string;
  maintenanceMode: boolean;
  controlTransport: string;
}

export interface RobotSettings {
  robot_name: string;
  ui_language: UiLanguage;
  control_mode: string;
  gamepad_mac: string;
  robot_wifi_ssid: string;
  robot_wifi_password?: string;
  robot_wifi_password_set?: boolean;
  camera_wifi_ssid: string;
  camera_url: string;
  camera_resolution: string;
  cam_p: number;
  cam_d: number;
  yaw_threshold: number;
  battery_empty: number;
  battery_full: number;
  battery_low: number;
  ota_min_percent: number;
}



