#include "WebController.h"

#include <DNSServer.h>
#include <ESPmDNS.h>
#include <esp_system.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>

#include "Diagnostics.h"
#include "PreferencesManager.h"
#include "RGBController.h"
#include "VoltageMonitor.h"
#include "generated/WebUiBundle.h"
#include "motion/MotionCoreAdapter.h"

extern float pid_cam_p;
extern float pid_cam_d;
extern int yaw_align_threshold;
extern bool trace_mode;
extern void sendCameraWifiConfig(const String& ssid, const String& password);
extern void sendCameraRuntimeConfig(const String& resolution);
extern void sendCameraTrackSelection(int x, int y, int width, int height,
                                     int profile = 0);
extern void sendCameraTrackDistanceAdjust(int value);
extern void sendCameraTrackScan();
extern void sendCameraTrackStop();

namespace {
constexpr char WIFI_SSID[] = "sdevil-WRobot";
constexpr char WIFI_PASSWORD[] = "wrobot123";
const IPAddress WIFI_AP_IP(192, 168, 8, 1);
const IPAddress WIFI_AP_GATEWAY(192, 168, 8, 1);
const IPAddress WIFI_AP_SUBNET(255, 255, 255, 0);
// HTTP fallback can occasionally pause while the browser is also decoding the
// camera stream. Pointer release still sends an explicit zero command, while
// this wider watchdog prevents a delayed heartbeat from pulsing the motors.
constexpr unsigned long COMMAND_TIMEOUT_MS = 10000;
constexpr uint16_t WEBSOCKET_PORT = 81;
constexpr size_t WEBSOCKET_MAX_FRAME_SIZE = 384;
constexpr unsigned long STA_CONNECT_TIMEOUT_MS = 15000;
constexpr unsigned long STA_RETRY_INTERVAL_MS = 30000;
constexpr unsigned long STA_BOOT_DELAY_MS = 3000;
constexpr unsigned long MDNS_RETRY_INTERVAL_MS = 30000;
constexpr unsigned long MDNS_RESTART_DELAY_MS = 1200;
constexpr char PREF_CAMERA_URL[] = "cam_stream_url";
constexpr char PREF_CAM_P[] = "web_cam_p";
constexpr char PREF_CAM_D[] = "web_cam_d";
constexpr char PREF_YAW_THR[] = "web_yaw_thr";
constexpr char PREF_BAT_EMPTY[] = "bat_empty_v";
constexpr char PREF_BAT_FULL[] = "bat_full_v";
constexpr char PREF_BAT_LOW[] = "bat_low_v";
// New key migrates existing installations away from the previous 80% default.
constexpr char PREF_OTA_MIN[] = "ota_min_v2";
constexpr char PREF_CAMERA_WIFI_SSID[] = "cam_wifi_ssid";
// A new preference key applies the 720p default once to existing installations.
constexpr char PREF_CAMERA_RESOLUTION[] = "cam_resolution_v3";
constexpr char PREF_CAMERA_FW_VERSION[] = "cam_fw_ver";
constexpr char PREF_CAMERA_FW_BUILD[] = "cam_fw_build";
constexpr char PREF_CAMERA_ACTIVE_RESOLUTION[] = "cam_active_res";
constexpr char PREF_ENABLE_GAMEPAD[] = "enable_gamepad";
constexpr char PREF_ROBOT_WIFI_SSID[] = "robot_wifi_ssid";
constexpr char PREF_ROBOT_WIFI_PASSWORD[] = "robot_wifi_pwd";
constexpr char PREF_ROBOT_NAME[] = "robot_name";
constexpr char PREF_UI_LANGUAGE[] = "ui_language";
constexpr char DEFAULT_UI_LANGUAGE[] = "en";
constexpr char DEFAULT_ROBOT_NAME[] = "WRobot-sdevil";
constexpr char DEFAULT_CAMERA_RESOLUTION[] = "640x480";
constexpr char ROBOT_FIRMWARE_VERSION[] = "3.2.88";
constexpr char ROBOT_FIRMWARE_BUILD[] = "2026-06-28-track-loss-return-scan-01";
constexpr char CONTROL_MODE_WIFI[] = "wifi";
constexpr char CONTROL_MODE_GAMEPAD[] = "gamepad";

WebServer server(80);
DNSServer dnsServer;
WiFiServer websocketServer(WEBSOCKET_PORT);
WiFiClient websocketClient;
TaskHandle_t webTaskHandle = nullptr;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
bool websocketHandshakeComplete = false;
bool websocketHadClient = false;
String websocketHandshakeBuffer;
uint8_t websocketFrameBuffer[WEBSOCKET_MAX_FRAME_SIZE + 16];
size_t websocketFrameLength = 0;

int driveX = 0;
int driveY = 0;
bool driveActive = false;
int gimbalYawX = 0;
bool gimbalYawActive = false;
unsigned long lastGimbalYawMs = 0;
int pendingCameraPitchDelta = 0;
int pendingLegHeightDirection = 99;
int pendingLegHeightPercent = -1;
int pendingLegLeanPercent = 999;
unsigned long lastDriveCommandMs = 0;
uint32_t driveCommandCount = 0;
uint32_t driveCommandMaxGapMs = 0;
unsigned long lastLegHeightCommandMs = 0;
unsigned long lastLegLeanCommandMs = 0;
QueueHandle_t actionQueue = nullptr;
SemaphoreHandle_t statusMutex = nullptr;

bool statusRobotEnabled = false;
bool statusSittingDown = true;
bool statusGamepadConnected = false;
bool statusMaintenanceMode = false;
bool statusOtaInProgress = false;
float statusBalanceAngle = 0.0f;
float statusBatteryVoltage = 0.0f;
int statusBatteryPercent = 0;
int statusLegHeightPercent = 50;
int statusLegLeanPercent = 0;
int statusOtaProgress = 0;
int otaMinBatteryPercent = 30;
bool otaUploadAccepted = false;
bool otaUploadSucceeded = false;
size_t otaUploadBytes = 0;
size_t otaUploadExpectedBytes = 0;
unsigned long pendingRebootAtMs = 0;
bool pendingGamepadRebootAfterSit = false;
unsigned long pendingGamepadRebootDeadlineMs = 0;
String statusOtaMessage = "Idle";
String statusCameraStreamUrl;
String statusCameraWifiSsid;
String statusCameraResolution = DEFAULT_CAMERA_RESOLUTION;
String statusCameraActiveResolution;
String statusCameraNetState = "IDLE";
String statusCameraNetIp;
String statusCameraNetMessage = "Camera WiFi idle";
String statusCameraFirmwareVersion;
String statusCameraFirmwareBuild;
String statusCameraDetectLabel = "NO_TARGET";
int statusCameraDetectCount = 0;
String statusControlMode = CONTROL_MODE_WIFI;
String statusRobotName = DEFAULT_ROBOT_NAME;
String statusUiLanguage = DEFAULT_UI_LANGUAGE;
String statusRobotWifiSsid;
String statusRobotWifiPassword;
String statusRobotNetState = "AP_ONLY";
String statusRobotStaIp;
bool staConnectInProgress = false;
bool staRetryEnabled = false;
bool mdnsActive = false;
unsigned long staConnectStartedMs = 0;
unsigned long staLastAttemptMs = 0;
unsigned long mdnsRetryAtMs = 0;
unsigned long mdnsRestartAtMs = 0;
String statusLastEvent = "boot";
const uint32_t statusBootId = esp_random();

void lockStatus() {
  if (statusMutex != nullptr) xSemaphoreTake(statusMutex, portMAX_DELAY);
}

void unlockStatus() {
  if (statusMutex != nullptr) xSemaphoreGive(statusMutex);
}


String jsonEscape(const String& input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else {
      out += c;
    }
  }
  return out;
}

const char REACT_INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <meta name="theme-color" content="#07090d">
  <title>WRobot Control</title>
</head>
<body>
  <div id="root"></div>
  <script type="module" src="/app.js"></script>
</body>
</html>
)HTML";

void handleWebRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.send_P(200, "text/html", REACT_INDEX_HTML);
}

void handleCaptivePortalRedirect() {
  server.sendHeader("Location", "http://192.168.8.1/", true);
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(302, "text/plain", "Redirecting to WRobot");
}

void handleReactAppJs() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "application/javascript",
                reinterpret_cast<PGM_P>(WEB_UI_APP_JS_GZ),
                WEB_UI_APP_JS_GZ_LEN);
}

void handleNotFound() {
  if (server.uri().startsWith("/api/")) {
    server.send(404, "application/json", "{\"error\":\"not found\"}");
    return;
  }
  if (server.method() == HTTP_GET) {
    handleWebRoot();
    return;
  }
  server.send(404, "text/plain", "Not found");
}

void logRobotWiFiEvent(const String& message) {
  lockStatus();
  statusLastEvent = message;
  unlockStatus();
  Serial.println("[ROBOT-WIFI] " + message);
  recordDiagnosticEvent("network", message);
}

void startMdnsService(const char* reason) {
  if (mdnsActive) {
    MDNS.end();
    mdnsActive = false;
    delay(2);
  }
  if (MDNS.begin("wrobot")) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("wrobot-ws", "tcp", WEBSOCKET_PORT);
    mdnsActive = true;
    mdnsRetryAtMs = 0;
    logRobotWiFiEvent(String("mDNS ready: ") + reason);
  } else {
    mdnsActive = false;
    mdnsRetryAtMs = millis() + MDNS_RETRY_INTERVAL_MS;
    logRobotWiFiEvent(String("mDNS failed: ") + reason);
  }
}

void scheduleMdnsRestart(const char* reason) {
  mdnsRestartAtMs = millis() + MDNS_RESTART_DELAY_MS;
  logRobotWiFiEvent(String("mDNS restart scheduled: ") + reason);
}

void pollMdnsService() {
  const unsigned long now = millis();
  if (mdnsRestartAtMs != 0 && static_cast<int32_t>(now - mdnsRestartAtMs) >= 0) {
    mdnsRestartAtMs = 0;
    startMdnsService("network changed");
    return;
  }
  if (!mdnsActive && mdnsRetryAtMs != 0 &&
      static_cast<int32_t>(now - mdnsRetryAtMs) >= 0) {
    startMdnsService("retry");
  }
}

void handleDiagnostics() {
  const MotionTelemetry motion = motionCore().telemetry();
  const BatteryStatus battery = getBatteryStatus();
  String robotNetState;
  String robotIp;
  String cameraState;
  String cameraIp;
  String cameraResolution;
  String cameraLabel;
  int cameraCount;
  bool otaRunning;
  int otaProgress;
  int currentDriveX;
  int currentDriveY;
  bool currentDriveActive;
  unsigned long currentDriveUpdatedAt;
  uint32_t currentDriveCommandCount;
  uint32_t currentDriveMaxGapMs;

  lockStatus();
  robotNetState = statusRobotNetState;
  robotIp = statusRobotStaIp;
  cameraState = statusCameraNetState;
  cameraIp = statusCameraNetIp;
  cameraResolution = statusCameraActiveResolution;
  cameraLabel = statusCameraDetectLabel;
  cameraCount = statusCameraDetectCount;
  otaRunning = statusOtaInProgress;
  otaProgress = statusOtaProgress;
  unlockStatus();

  portENTER_CRITICAL(&stateMux);
  currentDriveX = driveX;
  currentDriveY = driveY;
  currentDriveActive = driveActive;
  currentDriveUpdatedAt = lastDriveCommandMs;
  currentDriveCommandCount = driveCommandCount;
  currentDriveMaxGapMs = driveCommandMaxGapMs;
  portEXIT_CRITICAL(&stateMux);

  const unsigned long driveAgeMs =
      currentDriveUpdatedAt == 0 ? 0 : millis() - currentDriveUpdatedAt;
  CRGB statusLeds[NUM_LEDS];
  for (int i = 0; i < NUM_LEDS; ++i) {
    statusLeds[i] = getStatusLedColor(i);
  }
  char motionTrigger[DIAGNOSTIC_TRIGGER_LENGTH] = {};
  getMotionTrigger(motionTrigger, sizeof(motionTrigger));

  String json;
  json.reserve(7000);
  json = "{\"timestamp_ms\":" + String(millis()) +
         ",\"firmware\":\"" + String(ROBOT_FIRMWARE_VERSION) + "\"" +
         ",\"system\":{\"free_heap\":" + String(ESP.getFreeHeap()) +
         ",\"min_free_heap\":" + String(ESP.getMinFreeHeap()) +
         ",\"reset_reason\":" + String((int)esp_reset_reason()) + "}" +
         ",\"battery\":{\"valid\":" + String(battery.valid ? "true" : "false") +
         ",\"percent\":" + String(battery.percent) +
         ",\"voltage\":" + String(battery.voltage, 2) + "}" +
         ",\"led\":{\"gpio13\":" + String(digitalRead(LED_BAT)) +
         ",\"rgb_blinking\":" + String(isStatusLedBlinking() ? "true" : "false") +
         ",\"low_battery_warning\":" + String(isLowBatteryWarningActive() ? "true" : "false") +
         ",\"pending_clear_frames\":" + String(pendingStatusLedClearFrames()) +
         ",\"rgb\":[";
  for (int i = 0; i < NUM_LEDS; ++i) {
    if (i > 0) json += ',';
    json += "\"" + String(statusLeds[i].r) + "," +
            String(statusLeds[i].g) + "," + String(statusLeds[i].b) + "\"";
  }
  json += "]}" +
         String(",\"network\":{\"state\":\"") + jsonEscape(robotNetState) +
         "\",\"ip\":\"" + jsonEscape(robotIp) + "\"" +
         ",\"wifi_status\":" + String((int)WiFi.status()) +
         ",\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) +
         ",\"channel\":" + String(WiFi.channel()) +
         ",\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"" +
         ",\"ap_clients\":" + String(WiFi.softAPgetStationNum()) +
         ",\"mdns_active\":" + String(mdnsActive ? "true" : "false") +
         ",\"mdns_restart_pending\":" + String(mdnsRestartAtMs != 0 ? "true" : "false") + "}" +
         ",\"web_control\":{\"websocket_connected\":" +
             String(websocketHandshakeComplete && websocketClient.connected()
                        ? "true" : "false") +
         ",\"drive_active\":" + String(currentDriveActive ? "true" : "false") +
         ",\"x\":" + String(currentDriveX) +
         ",\"y\":" + String(currentDriveY) +
         ",\"command_age_ms\":" + String(driveAgeMs) +
         ",\"command_count\":" + String(currentDriveCommandCount) +
         ",\"max_command_gap_ms\":" + String(currentDriveMaxGapMs) + "}" +
         ",\"camera\":{\"state\":\"" + jsonEscape(cameraState) +
         "\",\"ip\":\"" + jsonEscape(cameraIp) +
         "\",\"resolution\":\"" + jsonEscape(cameraResolution) +
         "\",\"label\":\"" + jsonEscape(cameraLabel) +
         "\",\"count\":" + String(cameraCount) +
         ",\"angle\":" + String(motion.cameraAngleDeg, 1) +
         ",\"target_angle\":" + String(motion.cameraTargetDeg, 1) + "}" +
         ",\"motion\":{\"mode\":\"" + String(motion.mode) +
         "\",\"enabled\":" + String(motion.enabled ? "true" : "false") +
         ",\"sitting\":" + String(motion.sitting ? "true" : "false") +
         ",\"pitch_deg\":" + String(motion.balanceAngleDeg, 2) +
         ",\"pitch_rate_deg\":" + String(motion.pitchRateDeg, 2) +
         ",\"linear_position\":" + String(motion.linearPositionM, 4) +
         ",\"linear_velocity\":" + String(motion.linearVelocityMps, 4) +
         ",\"yaw_rate_deg\":" + String(motion.yawRateDeg, 2) +
         ",\"linear_reference\":" + String(motion.linearReferenceMps, 4) +
         ",\"yaw_reference\":" + String(motion.yawReferenceRad, 4) +
         ",\"drive_axis\":" + String(motion.driveAxis, 4) +
         ",\"steering_axis\":" + String(motion.steeringAxis, 4) +
         ",\"trigger\":\"" + jsonEscape(motionTrigger) + "\"" +
         ",\"left_leg_position\":" + String(motion.leftLegPosition) +
         ",\"right_leg_position\":" + String(motion.rightLegPosition) +
         ",\"left_leg_load\":" + String(motion.leftLegLoad) +
         ",\"right_leg_load\":" + String(motion.rightLegLoad) +
         ",\"control_max_gap_us\":" + String(motion.controlLoopMaxGapUs) +
         ",\"motor_max_gap_us\":" + String(motion.motorLoopMaxGapUs) +
         ",\"motor_late_count\":" + String(motion.motorLoopLateCount) +
         ",\"motor_enabled\":" + String(motion.motorEnabled ? "true" : "false") +
         ",\"left_voltage_q\":" + String(motion.leftMotorVoltageQ, 4) +
         ",\"right_voltage_q\":" + String(motion.rightMotorVoltageQ, 4) +
         ",\"stand_recover_active\":" + String(motion.standRecoverActive ? "true" : "false") +
         ",\"stand_recover_elapsed_ms\":" + String(motion.standRecoverElapsedMs) +
         ",\"stand_recover_displacement\":" + String(motion.standRecoverDisplacementM, 4) +
         ",\"stand_recover_max_displacement\":" + String(motion.standRecoverMaxDisplacementM, 4) +
         ",\"stand_recover_min_signed_displacement\":" + String(motion.standRecoverMinSignedDisplacementM, 4) +
         ",\"stand_recover_max_signed_displacement\":" + String(motion.standRecoverMaxSignedDisplacementM, 4) +
         ",\"stand_recover_peak_time_ms\":" + String(motion.standRecoverPeakTimeMs) +
         ",\"stand_recover_correction\":" + String(motion.standRecoverCorrectionMps, 4) + "}" +
         ",\"tracking\":{\"enabled\":" + String(motion.tracking ? "true" : "false") +
         ",\"state\":\"" + String(motion.trackingState) +
         "\",\"profile\":" + String(motion.trackingProfile) +
         ",\"confidence\":" + String(motion.trackingConfidence) +
         ",\"error_x\":" + String(motion.trackingErrorX, 2) +
         ",\"error_y\":" + String(motion.trackingErrorY, 2) +
         ",\"error_z\":" + String(motion.trackingErrorZ, 2) +
         ",\"raw_error_x\":" + String(motion.trackingRawErrorX) +
         ",\"raw_error_y\":" + String(motion.trackingRawErrorY) +
         ",\"raw_error_z\":" + String(motion.trackingRawErrorZ) +
         ",\"yaw_command\":" + String(motion.trackingYawCommand, 4) +
         ",\"drive_command\":" + String(motion.trackingDriveCommand, 4) +
         ",\"yaw_engaged\":" + String(motion.trackingYawEngaged ? "true" : "false") +
         ",\"distance_engaged\":" + String(motion.trackingDistanceEngaged ? "true" : "false") +
         ",\"balance_ready\":" + String(motion.trackingBalanceReady ? "true" : "false") +
         ",\"settle_active\":" + String(motion.trackingSettleActive ? "true" : "false") +
         ",\"observation_age_ms\":" + String(motion.trackingObservationAgeMs) +
         ",\"control_age_ms\":" + String(motion.trackingControlAgeMs) +
         ",\"control_interval_ms\":" + String(motion.trackingControlIntervalMs) +
         ",\"yaw_candidate_frames\":" + String(motion.trackingYawCandidateFrames) +
         ",\"distance_candidate_frames\":" + String(motion.trackingDistanceCandidateFrames) +
         ",\"box\":{\"x\":" + String(motion.trackingBoxX) +
         ",\"y\":" + String(motion.trackingBoxY) +
         ",\"w\":" + String(motion.trackingBoxW) +
         ",\"h\":" + String(motion.trackingBoxH) + "}" +
         ",\"frame\":{\"w\":" + String(motion.trackingFrameW) +
         ",\"h\":" + String(motion.trackingFrameH) + "}" +
         ",\"target_h\":" + String(motion.trackingTargetH) +
         ",\"missed_frames\":" + String(motion.trackingMissedFrames) +
         ",\"stable_frames\":" + String(motion.trackingStableFrames) +
         ",\"raw_score\":" + String(motion.trackingRawScore) +
         ",\"velocity_x\":" + String(motion.trackingVelocityX) +
         ",\"velocity_y\":" + String(motion.trackingVelocityY) +
         ",\"decision\":\"" + String(motion.trackingDecision) + "\"}" +
         ",\"ota\":{\"running\":" + String(otaRunning ? "true" : "false") +
         ",\"progress\":" + String(otaProgress) + "},\"events\":[";

  DiagnosticEvent events[DIAGNOSTIC_EVENT_CAPACITY];
  const size_t count = copyDiagnosticEvents(events, DIAGNOSTIC_EVENT_CAPACITY);
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) json += ',';
    json += "{\"t\":" + String(events[i].timestampMs) +
            ",\"category\":\"" + jsonEscape(events[i].category) +
            "\",\"message\":\"" + jsonEscape(events[i].message) + "\"}";
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void startRobotStaConnect() {
  statusRobotStaIp = "";
  if (statusRobotWifiSsid.isEmpty()) {
    statusRobotNetState = "AP_ONLY";
    staConnectInProgress = false;
    staRetryEnabled = false;
    logRobotWiFiEvent("no saved STA SSID");
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(statusRobotWifiSsid.c_str(), statusRobotWifiPassword.c_str());
  staConnectInProgress = true;
  staRetryEnabled = true;
  staConnectStartedMs = millis();
  staLastAttemptMs = staConnectStartedMs;
  statusRobotNetState = "CONNECTING";
  logRobotWiFiEvent("connecting to " + statusRobotWifiSsid);
}

void stopRobotSta() {
  WiFi.disconnect(false, true);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
    logRobotWiFiEvent("softAPConfig failed on disconnect");
  }
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
    logRobotWiFiEvent("softAP restart failed on disconnect");
  }
  staConnectInProgress = false;
  staRetryEnabled = false;
  statusRobotNetState = "AP_ONLY";
  statusRobotStaIp = "";
  staLastAttemptMs = millis();
  logRobotWiFiEvent("STA disconnected, AP retained");
}

void pollRobotSta() {
  if (staConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setSleep(false);
      statusRobotStaIp = WiFi.localIP().toString();
      statusRobotNetState = "CONNECTED";
      staConnectInProgress = false;
      logRobotWiFiEvent("connected: " + statusRobotStaIp);
      scheduleMdnsRestart("STA connected");
      return;
    }
    if (millis() - staConnectStartedMs > STA_CONNECT_TIMEOUT_MS) {
      statusRobotNetState = "FAILED";
      staConnectInProgress = false;
      WiFi.disconnect(false, false);
      staLastAttemptMs = millis();
      logRobotWiFiEvent("connect timeout");
    }
    return;
  }

  if (statusRobotNetState == "CONNECTED" && WiFi.status() != WL_CONNECTED) {
    statusRobotNetState = "LOST";
    statusRobotStaIp = "";
    staLastAttemptMs = millis();
    logRobotWiFiEvent("STA link lost");
    scheduleMdnsRestart("STA lost");
  }

  if (staRetryEnabled && !statusRobotWifiSsid.isEmpty() &&
      (statusRobotNetState == "FAILED" || statusRobotNetState == "LOST" || statusRobotNetState == "AP_ONLY") &&
      millis() - staLastAttemptMs > STA_RETRY_INTERVAL_MS) {
    startRobotStaConnect();
  }
}

void applyStoredSettings() {
  statusRobotName = getPrefString(PREF_ROBOT_NAME, DEFAULT_ROBOT_NAME);
  statusUiLanguage = getPrefString(PREF_UI_LANGUAGE, DEFAULT_UI_LANGUAGE);
  if (statusUiLanguage != "zh") statusUiLanguage = DEFAULT_UI_LANGUAGE;
  statusCameraStreamUrl = getPrefString(PREF_CAMERA_URL, "");
  statusCameraWifiSsid = getPrefString(PREF_CAMERA_WIFI_SSID, "");
  statusCameraResolution = getPrefString(PREF_CAMERA_RESOLUTION, DEFAULT_CAMERA_RESOLUTION);
  statusCameraActiveResolution = getPrefString(PREF_CAMERA_ACTIVE_RESOLUTION, "");
  statusCameraFirmwareVersion = getPrefString(PREF_CAMERA_FW_VERSION, "");
  statusCameraFirmwareBuild = getPrefString(PREF_CAMERA_FW_BUILD, "");
  statusControlMode = getPrefBool(PREF_ENABLE_GAMEPAD, false) ? CONTROL_MODE_GAMEPAD : CONTROL_MODE_WIFI;
  statusRobotWifiSsid = getPrefString(PREF_ROBOT_WIFI_SSID, "");
  statusRobotWifiPassword = getPrefString(PREF_ROBOT_WIFI_PASSWORD, "");
  pid_cam_p = getPrefFloat(PREF_CAM_P, pid_cam_p);
  pid_cam_d = getPrefFloat(PREF_CAM_D, pid_cam_d);
  yaw_align_threshold = getPrefInt(PREF_YAW_THR, yaw_align_threshold);
  otaMinBatteryPercent = constrain(getPrefInt(PREF_OTA_MIN, otaMinBatteryPercent), 1, 100);
  setBatteryCalibration(getPrefFloat(PREF_BAT_EMPTY, getBatteryEmptyVoltage()),
                        getPrefFloat(PREF_BAT_FULL, getBatteryFullVoltage()),
                        getPrefFloat(PREF_BAT_LOW, getBatteryLowVoltage()));
}

void saveStoredSettings() {
  setPrefString(PREF_ROBOT_NAME, statusRobotName);
  setPrefString(PREF_UI_LANGUAGE, statusUiLanguage);
  setPrefString(PREF_CAMERA_URL, statusCameraStreamUrl);
  setPrefString(PREF_CAMERA_WIFI_SSID, statusCameraWifiSsid);
  setPrefString(PREF_CAMERA_RESOLUTION, statusCameraResolution);
  setPrefString(PREF_ROBOT_WIFI_SSID, statusRobotWifiSsid);
  setPrefString(PREF_ROBOT_WIFI_PASSWORD, statusRobotWifiPassword);
  setPrefFloat(PREF_CAM_P, pid_cam_p);
  setPrefFloat(PREF_CAM_D, pid_cam_d);
  setPrefInt(PREF_YAW_THR, yaw_align_threshold);
  setPrefFloat(PREF_BAT_EMPTY, getBatteryEmptyVoltage());
  setPrefFloat(PREF_BAT_FULL, getBatteryFullVoltage());
  setPrefFloat(PREF_BAT_LOW, getBatteryLowVoltage());
  setPrefInt(PREF_OTA_MIN, otaMinBatteryPercent);
}

bool requestUsbPoweredOverride() {
  const String value = server.arg("usb_powered");
  return value == "1" || value == "true" || value == "on";
}

bool otaPrecheck(String& reason, bool usbPoweredOverride = false) {
  forceBatteryMeasurement();
  const BatteryStatus measuredBattery = getBatteryStatus();
  lockStatus();
  const bool otaInProgress = statusOtaInProgress;
  const bool maintenanceMode = statusMaintenanceMode;
  const bool sittingDown = statusSittingDown;
  const bool robotEnabled = statusRobotEnabled;
  const int minimumPercent = otaMinBatteryPercent;
  unlockStatus();
  if (otaInProgress) {
    reason = "OTA 正在进行中";
    return false;
  }
  if (!maintenanceMode) {
    reason = "请先进入维护模式";
    return false;
  }
  if (!sittingDown || robotEnabled) {
    reason = "请先让机器人坐下并停机";
    return false;
  }
  if (!measuredBattery.valid && !usbPoweredOverride) {
    reason = "Battery measurement unavailable; confirm external power to override";
    return false;
  }
  if (measuredBattery.percent < minimumPercent && !usbPoweredOverride) {
    reason = "Battery " + String(measuredBattery.percent) + "% (" +
             String(measuredBattery.voltage, 2) + "V), minimum " +
             String(minimumPercent) +
             "%; confirm external power to override";
    return false;
  }
  return true;
}

void sendJsonStatus() {
  bool enabled;
  bool sitting;
  bool gamepad;
  bool maintenance;
  bool otaRunning;
  float angle;
  float batteryVoltage;
  int batteryPercent;
  int legHeightPercent;
  int legLeanPercent;
  int otaProgress;
  String otaMessage;
  String robotNetState;
  String robotStaIp;
  String cameraNetState;
  String cameraNetIp;
  String cameraNetMessage;
  String cameraFirmwareVersion;
  String cameraFirmwareBuild;
  String cameraDetectLabel;
  String cameraActiveResolution;
  String lastEvent;
  int cameraDetectCount;
  String activeMode;
  lockStatus();
  enabled = statusRobotEnabled;
  sitting = statusSittingDown;
  gamepad = statusGamepadConnected;
  maintenance = statusMaintenanceMode;
  otaRunning = statusOtaInProgress;
  angle = statusBalanceAngle;
  batteryVoltage = statusBatteryVoltage;
  batteryPercent = statusBatteryPercent;
  legHeightPercent = statusLegHeightPercent;
  legLeanPercent = statusLegLeanPercent;
  otaProgress = statusOtaProgress;
  otaMessage = statusOtaMessage;
  robotNetState = statusRobotNetState;
  robotStaIp = statusRobotStaIp;
  cameraNetState = statusCameraNetState;
  cameraNetIp = statusCameraNetIp;
  cameraNetMessage = statusCameraNetMessage;
  cameraFirmwareVersion = statusCameraFirmwareVersion;
  cameraFirmwareBuild = statusCameraFirmwareBuild;
  cameraDetectLabel = statusCameraDetectLabel;
  cameraActiveResolution = statusCameraActiveResolution;
  cameraDetectCount = statusCameraDetectCount;
  lastEvent = statusLastEvent;
  activeMode = trace_mode ? "track_mode" : "";
  unlockStatus();

  String json = "{\"boot_id\":" + String(statusBootId) +
                ",\"firmware_version\":\"" + String(ROBOT_FIRMWARE_VERSION) + "\"" +
                ",\"firmware_build\":\"" + String(ROBOT_FIRMWARE_BUILD) + "\"" +
                ",\"enabled\":" + String(enabled ? "true" : "false") +
                ",\"sitting\":" + String(sitting ? "true" : "false") +
                ",\"gamepad\":" + String(gamepad ? "true" : "false") +
                ",\"maintenance\":" + String(maintenance ? "true" : "false") +
                ",\"ota_running\":" + String(otaRunning ? "true" : "false") +
                ",\"angle\":" + String(angle, 1) +
                ",\"battery_voltage\":" + String(batteryVoltage, 2) +
                ",\"battery_percent\":" + String(batteryPercent) +
                ",\"leg_height_percent\":" + String(legHeightPercent) +
                ",\"leg_lean_percent\":" + String(legLeanPercent) +
                ",\"ota_progress\":" + String(otaProgress) +
                ",\"ota_message\":\"" + jsonEscape(otaMessage) + "\"" +
                ",\"robot_net_state\":\"" + jsonEscape(robotNetState) + "\"" +
                ",\"robot_sta_ip\":\"" + jsonEscape(robotStaIp) + "\"" +
                ",\"robot_ap_ip\":\"" + WiFi.softAPIP().toString() + "\"" +
                ",\"robot_wifi_status\":" + String((int)WiFi.status()) +
                ",\"robot_wifi_rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) +
                ",\"robot_wifi_channel\":" + String(WiFi.channel()) +
                ",\"mdns_active\":" + String(mdnsActive ? "true" : "false") +
                ",\"camera_net_state\":\"" + jsonEscape(cameraNetState) + "\"" +
                ",\"camera_net_ip\":\"" + jsonEscape(cameraNetIp) + "\"" +
                ",\"camera_net_message\":\"" + jsonEscape(cameraNetMessage) + "\"" +
                ",\"camera_firmware_version\":\"" + jsonEscape(cameraFirmwareVersion) + "\"" +
                ",\"camera_firmware_build\":\"" + jsonEscape(cameraFirmwareBuild) + "\"" +
                ",\"camera_resolution\":\"" + jsonEscape(cameraActiveResolution) + "\"" +
                ",\"camera_detect_label\":\"" + jsonEscape(cameraDetectLabel) + "\"" +
                ",\"camera_detect_count\":" + String(cameraDetectCount) + ",\"active_mode\":\"" + jsonEscape(activeMode) + "\"" +
                ",\"last_event\":\"" + jsonEscape(lastEvent) + "\"" +
                ",\"clients\":" + String(WiFi.softAPgetStationNum()) + "}";
  server.send(200, "application/json", json);
}

void sendJsonSettings() {
  const String gamepadMac = getPrefBluetoothMacAddress();
  lockStatus();
  String json = "{\"robot_name\":\"" + jsonEscape(statusRobotName) + "\"" +
                ",\"robot_wifi_ssid\":\"" + jsonEscape(statusRobotWifiSsid) + "\"" +
                ",\"robot_wifi_password_set\":" +
                String(statusRobotWifiPassword.isEmpty() ? "false" : "true") +
                ",\"camera_url\":\"" + jsonEscape(statusCameraStreamUrl) + "\"" +
                ",\"camera_wifi_ssid\":\"" + jsonEscape(statusCameraWifiSsid) + "\"" +
                ",\"camera_resolution\":\"" + jsonEscape(statusCameraResolution) + "\"" +
                ",\"control_mode\":\"" + jsonEscape(statusControlMode) + "\"" +
                ",\"gamepad_mac\":\"" + jsonEscape(gamepadMac) + "\"" +
                ",\"ui_language\":\"" + jsonEscape(statusUiLanguage) + "\"" +
                ",\"cam_p\":" + String(pid_cam_p, 4) +
                ",\"cam_d\":" + String(pid_cam_d, 4) +
                ",\"yaw_threshold\":" + String(yaw_align_threshold) +
                ",\"battery_empty\":" + String(getBatteryEmptyVoltage(), 2) +
                ",\"battery_full\":" + String(getBatteryFullVoltage(), 2) +
                ",\"battery_low\":" + String(getBatteryLowVoltage(), 2) +
                ",\"ota_min_percent\":" + String(otaMinBatteryPercent) + "}";
  unlockStatus();
  server.send(200, "application/json", json);
}

String normalizeGamepadMac(const String& input) {
  String hex;
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    if (isxdigit(static_cast<unsigned char>(c))) {
      hex += static_cast<char>(tolower(static_cast<unsigned char>(c)));
    } else if (c != ':' && c != '-' && c != ' ' && c != '.') {
      return "";
    }
  }
  if (hex.length() != 12) return "";
  String normalized;
  normalized.reserve(17);
  for (size_t i = 0; i < hex.length(); i += 2) {
    if (i > 0) normalized += ':';
    normalized += hex.substring(i, i + 2);
  }
  return normalized;
}

void updateRobotWiFiCredentialsFromRequest() {
  if (server.hasArg("robot_wifi_ssid")) {
    const String nextSsid = server.arg("robot_wifi_ssid");
    if (nextSsid != statusRobotWifiSsid &&
        (!server.hasArg("robot_wifi_password") ||
         server.arg("robot_wifi_password").isEmpty())) {
      statusRobotWifiPassword = "";
    }
    statusRobotWifiSsid = nextSsid;
  }
  if (server.hasArg("robot_wifi_password") &&
      !server.arg("robot_wifi_password").isEmpty()) {
    statusRobotWifiPassword = server.arg("robot_wifi_password");
  }
}

void handleSettingsUpdate() {
  bool shouldReboot = false;
  bool rebootAfterSit = false;
  bool cameraResolutionChanged = false;
  String cameraResolutionToApply;
  lockStatus();
  if (server.hasArg("robot_name")) statusRobotName = server.arg("robot_name");
  if (server.hasArg("ui_language")) {
    statusUiLanguage = server.arg("ui_language") == "zh" ? "zh" : DEFAULT_UI_LANGUAGE;
  }
  updateRobotWiFiCredentialsFromRequest();
  if (server.hasArg("gamepad_mac")) {
    const String normalizedMac = normalizeGamepadMac(server.arg("gamepad_mac"));
    if (!normalizedMac.isEmpty()) setPrefBluetoothMacAddress(normalizedMac);
  }
  if (server.hasArg("camera_url")) statusCameraStreamUrl = server.arg("camera_url");
  if (server.hasArg("camera_resolution")) {
    const String requestedResolution = server.arg("camera_resolution");
    if (requestedResolution == "160x120" ||
        requestedResolution == "320x240" ||
        requestedResolution == "640x480" ||
        requestedResolution == "1280x720") {
      statusCameraResolution = requestedResolution;
      cameraResolutionChanged = true;
      cameraResolutionToApply = requestedResolution;
    }
  }
  if (server.hasArg("control_mode")) {
    const String requestedMode = server.arg("control_mode") == CONTROL_MODE_GAMEPAD
                                   ? CONTROL_MODE_GAMEPAD
                                   : CONTROL_MODE_WIFI;
    if (requestedMode != statusControlMode) {
      statusControlMode = requestedMode;
      setPrefBool(PREF_ENABLE_GAMEPAD, statusControlMode == CONTROL_MODE_GAMEPAD);
      shouldReboot = true;
      rebootAfterSit = statusControlMode == CONTROL_MODE_GAMEPAD;
    }
  }
  if (server.hasArg("cam_p")) pid_cam_p = server.arg("cam_p").toFloat();
  if (server.hasArg("cam_d")) pid_cam_d = server.arg("cam_d").toFloat();
  if (server.hasArg("yaw_threshold")) yaw_align_threshold = server.arg("yaw_threshold").toInt();

  if (server.hasArg("battery_empty") || server.hasArg("battery_full") ||
      server.hasArg("battery_low")) {
    float emptyV = getBatteryEmptyVoltage();
    float fullV = getBatteryFullVoltage();
    float lowV = getBatteryLowVoltage();
    if (server.hasArg("battery_empty")) emptyV = server.arg("battery_empty").toFloat();
    if (server.hasArg("battery_full")) fullV = server.arg("battery_full").toFloat();
    if (server.hasArg("battery_low")) lowV = server.arg("battery_low").toFloat();
    setBatteryCalibration(emptyV, fullV, lowV);
  }

  if (server.hasArg("ota_min_percent")) {
    otaMinBatteryPercent = constrain(server.arg("ota_min_percent").toInt(), 1, 100);
  }

  saveStoredSettings();
  statusOtaMessage = shouldReboot ? "控制模式已切换，设备即将重启" : "设置已保存";
  unlockStatus();
  sendJsonSettings();
  if (cameraResolutionChanged) {
    sendCameraRuntimeConfig(cameraResolutionToApply);
  }
  if (shouldReboot) {
    if (rebootAfterSit) {
      const WebRobotAction sitAction = WebRobotAction::Sit;
      if (actionQueue != nullptr &&
          xQueueSend(actionQueue, &sitAction, 0) == pdTRUE) {
        pendingGamepadRebootAfterSit = true;
        pendingGamepadRebootDeadlineMs = millis() + 15000;
        pendingRebootAtMs = 0;
      } else {
        setPrefBool(PREF_ENABLE_GAMEPAD, false);
        lockStatus();
        statusControlMode = CONTROL_MODE_WIFI;
        statusOtaMessage = "Unable to queue sit action; mode switch cancelled";
        unlockStatus();
      }
    } else {
      pendingRebootAtMs = millis() + 1200;
    }
  }
}

void handleApplyRobotWiFi() {
  lockStatus();
  updateRobotWiFiCredentialsFromRequest();
  saveStoredSettings();
  unlockStatus();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSaveRobotWiFi() {
  lockStatus();
  updateRobotWiFiCredentialsFromRequest();
  saveStoredSettings();
  const String savedSsid = statusRobotWifiSsid;
  unlockStatus();
  logRobotWiFiEvent("saved credentials for " + savedSsid);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleConnectRobotWiFi() {
  lockStatus();
  updateRobotWiFiCredentialsFromRequest();
  saveStoredSettings();
  unlockStatus();
  startRobotStaConnect();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleDisconnectRobotWiFi() {
  stopRobotSta();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApplyCameraWiFi() {
  lockStatus();
  const String targetSsid = statusRobotWifiSsid;
  const String targetPassword = statusRobotWifiPassword;
  if (targetSsid.isEmpty()) {
    statusCameraNetState = "FAILED";
    statusCameraNetMessage = "Robot home WiFi is empty";
    unlockStatus();
    server.send(409, "application/json", "{\"error\":\"robot home wifi not set\"}");
    return;
  }
  statusCameraWifiSsid = targetSsid;
  statusCameraNetState = "PENDING";
  statusCameraNetIp = "";
  statusCameraNetMessage = "Sending home WiFi to camera";
  saveStoredSettings();
  const String resolution = statusCameraResolution;
  unlockStatus();
  sendCameraWifiConfig(targetSsid, targetPassword);
  sendCameraRuntimeConfig(resolution);
  server.send(200, "application/json", "{\"ok\":true}");
}


WebRobotAction actionFromName(const String& name) {
  if (name == "stand") return WebRobotAction::Stand;
  if (name == "sit") return WebRobotAction::Sit;
  if (name == "reset") return WebRobotAction::ResetPose;
  if (name == "cancel_kick") return WebRobotAction::CancelKick;
  if (name == "track_mode") return WebRobotAction::TrackMode;
  if (name == "led_test") return WebRobotAction::LedTest;
  if (name == "jump") return WebRobotAction::Jump;
  if (name == "jump_forward") return WebRobotAction::JumpForward;
  if (name == "jump_backward") return WebRobotAction::JumpBackward;
  if (name == "jump_left") return WebRobotAction::JumpLeft;
  if (name == "jump_right") return WebRobotAction::JumpRight;
  if (name == "maintenance_on") return WebRobotAction::EnterMaintenance;
  if (name == "maintenance_off") return WebRobotAction::ExitMaintenance;
  return WebRobotAction::None;
}

bool setDriveCommand(int x, int y) {
  lockStatus();
  const bool controlsLocked = statusMaintenanceMode || statusOtaInProgress;
  unlockStatus();
  if (controlsLocked) return false;
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  const unsigned long now = millis();
  portENTER_CRITICAL(&stateMux);
  if (lastDriveCommandMs != 0) {
    const uint32_t gapMs = now - lastDriveCommandMs;
    if (gapMs > driveCommandMaxGapMs) driveCommandMaxGapMs = gapMs;
  }
  ++driveCommandCount;
  driveX = x;
  driveY = y;
  driveActive = !(x == 0 && y == 0);
  lastDriveCommandMs = now;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool setGimbalYawCommand(int x) {
  lockStatus();
  const bool controlsLocked = statusMaintenanceMode || statusOtaInProgress;
  unlockStatus();
  if (controlsLocked) return false;
  x = constrain(x, -100, 100);
  portENTER_CRITICAL(&stateMux);
  gimbalYawX = x;
  gimbalYawActive = x != 0;
  lastGimbalYawMs = millis();
  portEXIT_CRITICAL(&stateMux);
  return true;
}

void queueCameraPitchDelta(int delta) {
  portENTER_CRITICAL(&stateMux);
  pendingCameraPitchDelta = constrain(pendingCameraPitchDelta + delta, -18, 18);
  portEXIT_CRITICAL(&stateMux);
}

bool setLegHeightCommand(int direction) {
  lockStatus();
  const bool controlsLocked = statusMaintenanceMode || statusOtaInProgress;
  unlockStatus();
  if (controlsLocked) return false;
  direction = constrain(direction, -1, 1);
  portENTER_CRITICAL(&stateMux);
  pendingLegHeightDirection = direction;
  lastLegHeightCommandMs = direction == 0 ? 0 : millis();
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool setLegHeightPercentCommand(int percent) {
  lockStatus();
  const bool controlsLocked = statusMaintenanceMode || statusOtaInProgress;
  unlockStatus();
  if (controlsLocked) return false;
  percent = constrain(percent, 0, 100);
  portENTER_CRITICAL(&stateMux);
  pendingLegHeightPercent = percent;
  pendingLegHeightDirection = 99;
  lastLegHeightCommandMs = 0;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool setLegLeanCommand(int percent) {
  lockStatus();
  const bool controlsLocked = statusMaintenanceMode || statusOtaInProgress;
  unlockStatus();
  if (controlsLocked) return false;
  percent = constrain(percent, -100, 100);
  portENTER_CRITICAL(&stateMux);
  pendingLegLeanPercent = percent;
  lastLegLeanCommandMs = percent == 0 ? 0 : millis();
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool isMaintenanceOnlyAction(WebRobotAction action) {
  return action == WebRobotAction::LedTest;
}

bool isAllowedDuringMaintenance(WebRobotAction action) {
  return action == WebRobotAction::ExitMaintenance ||
         action == WebRobotAction::LedTest;
}

bool queueRobotAction(const String& name) {
  const WebRobotAction action = actionFromName(name);
  lockStatus();
  const bool otaInProgress = statusOtaInProgress;
  const bool maintenanceMode = statusMaintenanceMode;
  unlockStatus();
  if (action == WebRobotAction::None || otaInProgress) return false;
  if (isMaintenanceOnlyAction(action) && !maintenanceMode) return false;
  if (maintenanceMode && !isAllowedDuringMaintenance(action)) return false;
  return actionQueue != nullptr && xQueueSend(actionQueue, &action, 0) == pdTRUE;
}

bool jsonIntValue(const String& json, const char* key, int& value) {
  const String token = "\"" + String(key) + "\"";
  int position = json.indexOf(token);
  if (position < 0) return false;
  position = json.indexOf(':', position + token.length());
  if (position < 0) return false;
  ++position;
  while (position < static_cast<int>(json.length()) &&
         (json[position] == ' ' || json[position] == '\t')) {
    ++position;
  }
  int end = position;
  if (end < static_cast<int>(json.length()) && json[end] == '-') ++end;
  while (end < static_cast<int>(json.length()) && isDigit(json[end])) ++end;
  if (end == position) return false;
  value = json.substring(position, end).toInt();
  return true;
}

bool jsonStringValue(const String& json, const char* key, String& value) {
  const String token = "\"" + String(key) + "\"";
  int position = json.indexOf(token);
  if (position < 0) return false;
  position = json.indexOf(':', position + token.length());
  if (position < 0) return false;
  position = json.indexOf('"', position + 1);
  if (position < 0) return false;
  const int end = json.indexOf('"', position + 1);
  if (end < 0) return false;
  value = json.substring(position + 1, end);
  return true;
}

void stopWebSocketDrive() {
  portENTER_CRITICAL(&stateMux);
  driveX = 0;
  driveY = 0;
  driveActive = false;
  gimbalYawX = 0;
  gimbalYawActive = false;
  lastGimbalYawMs = 0;
  lastDriveCommandMs = 0;
  pendingCameraPitchDelta = 0;
  pendingLegHeightDirection = 0;
  pendingLegHeightPercent = -1;
  pendingLegLeanPercent = 0;
  lastLegHeightCommandMs = 0;
  lastLegLeanCommandMs = 0;
  portEXIT_CRITICAL(&stateMux);
}

void closeWebSocketClient() {
  if (websocketClient) websocketClient.stop();
  websocketHadClient = false;
  websocketHandshakeComplete = false;
  websocketHandshakeBuffer = "";
  websocketFrameLength = 0;
  stopWebSocketDrive();
}

void sendWebSocketFrame(uint8_t opcode, const uint8_t* payload, size_t length) {
  if (!websocketClient.connected() || length > 125) return;
  const uint8_t header[2] = {
      static_cast<uint8_t>(0x80 | (opcode & 0x0F)),
      static_cast<uint8_t>(length)};
  websocketClient.write(header, sizeof(header));
  if (length > 0) websocketClient.write(payload, length);
}

void sendWebSocketText(const char* message) {
  sendWebSocketFrame(0x1, reinterpret_cast<const uint8_t*>(message),
                     strlen(message));
}

void sendWebSocketAck(const char* type, int requestId, bool ok) {
  if (requestId <= 0) return;
  const String message = "{\"ok\":" + String(ok ? "true" : "false") +
                         ",\"type\":\"" + type + "\",\"id\":" +
                         String(requestId) + "}";
  sendWebSocketText(message.c_str());
}

String websocketAcceptKey(const String& clientKey) {
  const String source =
      clientKey + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  unsigned char digest[20];
  unsigned char encoded[32];
  size_t encodedLength = 0;
  if (mbedtls_sha1_ret(
          reinterpret_cast<const unsigned char*>(source.c_str()),
          source.length(), digest) != 0) {
    return "";
  }
  if (mbedtls_base64_encode(encoded, sizeof(encoded) - 1, &encodedLength,
                            digest, sizeof(digest)) != 0) {
    return "";
  }
  encoded[encodedLength] = '\0';
  return String(reinterpret_cast<char*>(encoded));
}

void handleWebSocketMessage(const String& message) {
  String type;
  if (!jsonStringValue(message, "type", type)) return;
  int requestId = 0;
  jsonIntValue(message, "id", requestId);

  if (type == "drive") {
    int x = 0;
    int y = 0;
    if (jsonIntValue(message, "x", x) && jsonIntValue(message, "y", y) &&
        setDriveCommand(x, y)) {
      sendWebSocketAck("drive", requestId, true);
    }
    return;
  }

  if (type == "gimbal") {
    int yaw = 0;
    int pitchDelta = 0;
    if (!jsonIntValue(message, "yaw", yaw) ||
        !jsonIntValue(message, "pitchDelta", pitchDelta)) {
      return;
    }
    if (!setGimbalYawCommand(yaw)) return;
    pitchDelta = constrain(pitchDelta, -4, 4);
    if (pitchDelta != 0) queueCameraPitchDelta(pitchDelta);
    sendWebSocketAck("gimbal", requestId, true);
    return;
  }

  if (type == "leg_height") {
    int direction = 0;
    const bool accepted = jsonIntValue(message, "direction", direction) &&
                          setLegHeightCommand(direction);
    sendWebSocketAck("leg_height", requestId, accepted);
    return;
  }

  if (type == "leg_height_value") {
    int percent = 0;
    const bool accepted = jsonIntValue(message, "percent", percent) &&
                          setLegHeightPercentCommand(percent);
    sendWebSocketAck("leg_height_value", requestId, accepted);
    return;
  }

  if (type == "leg_lean") {
    int percent = 0;
    const bool accepted = jsonIntValue(message, "percent", percent) &&
                          setLegLeanCommand(percent);
    sendWebSocketAck("leg_lean", requestId, accepted);
    return;
  }


  if (type == "track_roi") {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int profile = 0;
    if (!jsonIntValue(message, "x", x) ||
        !jsonIntValue(message, "y", y) ||
        !jsonIntValue(message, "w", width) ||
        !jsonIntValue(message, "h", height)) {
      return;
    }
    x = constrain(x, 0, 9999);
    y = constrain(y, 0, 9999);
    width = constrain(width, 1, 10000 - x);
    height = constrain(height, 1, 10000 - y);
    jsonIntValue(message, "profile", profile);
    profile = constrain(profile, 0, 3);
    sendCameraTrackSelection(x, y, width, height, profile);
    recordDiagnosticEvent("camera", String("track_roi p=") + String(profile));
    sendWebSocketAck("track_roi", requestId, true);
    return;
  }

  if (type == "track_unlock" || type == "track_scan") {
    sendCameraTrackScan();
    recordDiagnosticEvent("camera", "track_scan");
    sendWebSocketAck(type.c_str(), requestId, true);
    return;
  }

  if (type == "track_distance") {
    int value = 0;
    if (!jsonIntValue(message, "value", value)) return;
    sendCameraTrackDistanceAdjust(value);
    sendWebSocketAck("track_distance", requestId, true);
    return;
  }

  if (type == "action") {
    String name;
    const bool accepted = jsonStringValue(message, "name", name) &&
                          queueRobotAction(name);
    sendWebSocketAck("action", requestId, accepted);
  }
}

void processWebSocketFrames() {
  while (websocketClient.available() &&
         websocketFrameLength < sizeof(websocketFrameBuffer)) {
    websocketFrameBuffer[websocketFrameLength++] =
        static_cast<uint8_t>(websocketClient.read());
  }

  while (websocketFrameLength >= 2) {
    const uint8_t first = websocketFrameBuffer[0];
    const uint8_t second = websocketFrameBuffer[1];
    const bool finalFrame = (first & 0x80) != 0;
    const bool masked = (second & 0x80) != 0;
    uint64_t payloadLength = second & 0x7F;
    size_t headerLength = 2;

    if (payloadLength == 126) {
      if (websocketFrameLength < 4) return;
      payloadLength =
          (static_cast<uint16_t>(websocketFrameBuffer[2]) << 8) |
          websocketFrameBuffer[3];
      headerLength = 4;
    } else if (payloadLength == 127) {
      closeWebSocketClient();
      return;
    }

    if (!masked || !finalFrame || payloadLength > WEBSOCKET_MAX_FRAME_SIZE) {
      closeWebSocketClient();
      return;
    }

    const size_t totalLength = headerLength + 4 + payloadLength;
    if (websocketFrameLength < totalLength) return;

    const uint8_t* mask = websocketFrameBuffer + headerLength;
    uint8_t* payload = websocketFrameBuffer + headerLength + 4;
    for (size_t i = 0; i < payloadLength; ++i) {
      payload[i] ^= mask[i % 4];
    }

    const uint8_t opcode = first & 0x0F;
    if (opcode == 0x8) {
      closeWebSocketClient();
      return;
    }
    if (opcode == 0x9) {
      sendWebSocketFrame(0xA, payload, payloadLength);
    } else if (opcode == 0x1) {
      String message;
      message.reserve(payloadLength);
      for (size_t i = 0; i < payloadLength; ++i) {
        message += static_cast<char>(payload[i]);
      }
      handleWebSocketMessage(message);
    }

    const size_t remaining = websocketFrameLength - totalLength;
    if (remaining > 0) {
      memmove(websocketFrameBuffer,
              websocketFrameBuffer + totalLength, remaining);
    }
    websocketFrameLength = remaining;
  }

  if (websocketFrameLength == sizeof(websocketFrameBuffer)) {
    closeWebSocketClient();
  }
}

void pollWebSocket() {
  if (!websocketClient || !websocketClient.connected()) {
    if (websocketHadClient) closeWebSocketClient();
    WiFiClient candidate = websocketServer.available();
    if (candidate) {
      websocketClient = candidate;
      websocketHadClient = true;
      websocketClient.setNoDelay(true);
      websocketHandshakeComplete = false;
      websocketHandshakeBuffer = "";
      websocketFrameLength = 0;
    }
    return;
  }

  if (!websocketHandshakeComplete) {
    while (websocketClient.available() &&
           websocketHandshakeBuffer.length() < 2048) {
      websocketHandshakeBuffer +=
          static_cast<char>(websocketClient.read());
    }
    const int headerEnd = websocketHandshakeBuffer.indexOf("\r\n\r\n");
    if (headerEnd < 0) {
      if (websocketHandshakeBuffer.length() >= 2048) closeWebSocketClient();
      return;
    }

    const String keyHeader = "Sec-WebSocket-Key:";
    int keyStart = websocketHandshakeBuffer.indexOf(keyHeader);
    if (keyStart < 0) {
      closeWebSocketClient();
      return;
    }
    keyStart += keyHeader.length();
    int keyEnd = websocketHandshakeBuffer.indexOf("\r\n", keyStart);
    String clientKey = websocketHandshakeBuffer.substring(keyStart, keyEnd);
    clientKey.trim();
    const String acceptKey = websocketAcceptKey(clientKey);
    if (acceptKey.isEmpty()) {
      closeWebSocketClient();
      return;
    }

    websocketClient.print(
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " +
        acceptKey + "\r\n\r\n");
    websocketHandshakeComplete = true;
    websocketHandshakeBuffer = "";
    sendWebSocketText("{\"type\":\"ready\"}");
    return;
  }

  processWebSocketFrames();
}

void handleDrive() {
  if (!server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "application/json", "{\"error\":\"missing x or y\"}");
    return;
  }
  if (!setDriveCommand(server.arg("x").toInt(), server.arg("y").toInt())) {
    server.send(423, "application/json", "{\"error\":\"robot locked for maintenance\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCameraPitch() {
  if (statusMaintenanceMode || statusOtaInProgress) {
    server.send(423, "application/json", "{\"error\":\"robot locked for maintenance\"}");
    return;
  }
  if (!server.hasArg("delta")) {
    server.send(400, "application/json", "{\"error\":\"missing delta\"}");
    return;
  }

  const int delta = constrain(server.arg("delta").toInt(), -4, 4);
  if (delta != 0) queueCameraPitchDelta(delta);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleLegHeight() {
  if (!server.hasArg("direction")) {
    server.send(400, "application/json", "{\"error\":\"missing direction\"}");
    return;
  }
  if (!setLegHeightCommand(server.arg("direction").toInt())) {
    server.send(423, "application/json", "{\"error\":\"robot locked for maintenance\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleLegHeightValue() {
  if (!server.hasArg("percent")) {
    server.send(400, "application/json", "{\"error\":\"missing percent\"}");
    return;
  }
  if (!setLegHeightPercentCommand(server.arg("percent").toInt())) {
    server.send(423, "application/json", "{\"error\":\"robot locked for maintenance\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleLegLean() {
  if (!server.hasArg("percent")) {
    server.send(400, "application/json", "{\"error\":\"missing percent\"}");
    return;
  }
  if (!setLegLeanCommand(server.arg("percent").toInt())) {
    server.send(423, "application/json", "{\"error\":\"robot locked for maintenance\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void expireLegControlCommands() {
  constexpr unsigned long timeoutMs = 350;
  const unsigned long now = millis();
  portENTER_CRITICAL(&stateMux);
  if (lastLegHeightCommandMs != 0 &&
      now - lastLegHeightCommandMs > timeoutMs) {
    pendingLegHeightDirection = 0;
    lastLegHeightCommandMs = 0;
  }
  if (lastLegLeanCommandMs != 0 &&
      now - lastLegLeanCommandMs > timeoutMs) {
    pendingLegLeanPercent = 0;
    lastLegLeanCommandMs = 0;
  }
  portEXIT_CRITICAL(&stateMux);
}


void handleTrackSelection() {
  if (statusMaintenanceMode || statusOtaInProgress) {
    server.send(423, "application/json", "{\"error\":\"robot locked\"}");
    return;
  }
  if (!server.hasArg("x") || !server.hasArg("y") ||
      !server.hasArg("w") || !server.hasArg("h")) {
    server.send(400, "application/json", "{\"error\":\"missing target rectangle\"}");
    return;
  }

  const int x = constrain(server.arg("x").toInt(), 0, 9999);
  const int y = constrain(server.arg("y").toInt(), 0, 9999);
  const int width = constrain(server.arg("w").toInt(), 1, 10000 - x);
  const int height = constrain(server.arg("h").toInt(), 1, 10000 - y);
  const int profile = constrain(server.arg("profile").toInt(), 0, 3);
  sendCameraTrackSelection(x, y, width, height, profile);
  recordDiagnosticEvent("camera", String("track_roi p=") + String(profile));
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTrackUnlock() {
  if (statusMaintenanceMode || statusOtaInProgress) {
    server.send(423, "application/json", "{\"error\":\"robot locked\"}");
    return;
  }
  sendCameraTrackScan();
  recordDiagnosticEvent("camera", "track_scan");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTrackDistanceAdjust() {
  if (statusMaintenanceMode || statusOtaInProgress) {
    server.send(423, "application/json", "{\"error\":\"robot locked\"}");
    return;
  }
  if (!server.hasArg("value")) {
    server.send(400, "application/json", "{\"error\":\"missing distance value\"}");
    return;
  }
  sendCameraTrackDistanceAdjust(server.arg("value").toInt());
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleAction() {
  const String name = server.arg("name");
  if (actionFromName(name) == WebRobotAction::None) {
    server.send(400, "application/json", "{\"error\":\"unknown action\"}");
    return;
  }
  if (!queueRobotAction(name)) {
    server.send(423, "application/json", "{\"error\":\"ota in progress\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRollback() {
  String reason;
  if (!otaPrecheck(reason, requestUsbPoweredOverride())) {
    statusOtaMessage = reason;
    server.send(409, "application/json", "{\"error\":\"" + jsonEscape(reason) + "\"}");
    return;
  }
  if (!Update.canRollBack()) {
    statusOtaMessage = "没有可回滚的旧固件";
    server.send(409, "application/json", "{\"error\":\"no rollback image\"}");
    return;
  }
  if (!Update.rollBack()) {
    statusOtaMessage = "回滚启动标记失败";
    server.send(500, "application/json", "{\"error\":\"rollback failed\"}");
    return;
  }
  statusOtaMessage = "准备回滚，设备即将重启";
  pendingRebootAtMs = millis() + 1200;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleOtaUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    String reason;
    pendingRebootAtMs = 0;
    pendingGamepadRebootAfterSit = false;
    pendingGamepadRebootDeadlineMs = 0;
    closeWebSocketClient();
    stopWebSocketDrive();
    sendCameraTrackStop();
    updateWebCameraNetworkStatus("OTA_PAUSED", statusCameraNetIp,
                                 "Camera link paused during OTA");
    otaUploadSucceeded = false;
    otaUploadBytes = 0;
    otaUploadExpectedBytes = server.hasArg("size")
                                 ? static_cast<size_t>(server.arg("size").toInt())
                                 : 0;
    if (!otaPrecheck(reason, requestUsbPoweredOverride())) {
      otaUploadAccepted = false;
      statusOtaMessage = reason;
      Serial.printf("[OTA] rejected: %s\n", reason.c_str());
      return;
    }
    if (otaUploadExpectedBytes == 0) {
      otaUploadAccepted = false;
      statusOtaMessage = "OTA file size missing";
      Serial.println("[OTA] rejected: file size missing");
      return;
    }
    statusOtaInProgress = true;
    statusOtaProgress = 0;
    statusOtaMessage = "Preparing OTA";
    Serial.printf("[OTA] upload start: %s\n", upload.filename.c_str());
    recordDiagnosticEvent("ota", String("start size=") + String(otaUploadExpectedBytes));
    otaUploadAccepted = Update.begin(otaUploadExpectedBytes, U_FLASH);
    if (!otaUploadAccepted) {
      statusOtaMessage = String("OTA init failed: ") + Update.errorString();
      Serial.printf("[OTA] begin failed: %s\n", Update.errorString());
      statusOtaInProgress = false;
      return;
    }
    statusOtaMessage = "Uploading firmware";
    Update.onProgress([](size_t progress, size_t total) {
      portENTER_CRITICAL(&stateMux);
      statusOtaProgress = total == 0 ? 0 : (int)((progress * 100U) / total);
      portEXIT_CRITICAL(&stateMux);
    });
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaUploadAccepted) return;
    const size_t written = Update.write(upload.buf, upload.currentSize);
    yield();
    otaUploadBytes += written;
    if (written != upload.currentSize) {
      statusOtaMessage = String("OTA write failed: ") + Update.errorString();
      Serial.printf("[OTA] write failed: requested=%u written=%u error=%s\n",
                    static_cast<unsigned>(upload.currentSize),
                    static_cast<unsigned>(written),
                    Update.errorString());
      otaUploadAccepted = false;
      Update.abort();
      statusOtaInProgress = false;
      statusOtaProgress = 0;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!otaUploadAccepted) {
      statusOtaInProgress = false;
      statusOtaProgress = 0;
      return;
    }
    if (otaUploadBytes != otaUploadExpectedBytes) {
      statusOtaMessage = "OTA size mismatch";
      Serial.printf("[OTA] size mismatch: expected=%u received=%u\n",
                    static_cast<unsigned>(otaUploadExpectedBytes),
                    static_cast<unsigned>(otaUploadBytes));
      Update.abort();
      otaUploadSucceeded = false;
    } else {
      otaUploadSucceeded = Update.end(false);
    }
    statusOtaInProgress = false;
    statusOtaProgress = otaUploadSucceeded ? 100 : 0;
    statusOtaMessage = otaUploadSucceeded ? "OTA complete, reboot pending" : String("OTA failed: ") + Update.errorString();
    Serial.printf("[OTA] upload end: ok=%s bytes=%u error=%s\n",
                  otaUploadSucceeded ? "true" : "false",
                  static_cast<unsigned>(otaUploadBytes),
                  Update.errorString());
    recordDiagnosticEvent("ota", otaUploadSucceeded ? "complete" : "failed");
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (otaUploadAccepted) Update.abort();
    otaUploadAccepted = false;
    statusOtaInProgress = false;
    statusOtaProgress = 0;
    statusOtaMessage = "OTA aborted";
    Serial.println("[OTA] upload aborted");
    recordDiagnosticEvent("ota", "aborted");
  }
}

void handleOtaFinish() {
  if (otaUploadSucceeded) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", "{\"ok\":true}");
    pendingRebootAtMs = millis() + 3000;
    Serial.println("[OTA] response sent, reboot scheduled");
  } else {
    server.sendHeader("Connection", "close");
    server.send(500, "application/json", "{\"error\":\"" + jsonEscape(statusOtaMessage) + "\"}");
    Serial.printf("[OTA] response failed: %s\n", statusOtaMessage.c_str());
  }
}

void webServerTask(void *) {
  server.on("/", HTTP_GET, handleWebRoot);
  server.on("/index.html", HTTP_GET, handleWebRoot);
  server.on("/app.js", HTTP_GET, handleReactAppJs);
  server.on("/generate_204", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/gen_204", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/hotspot-detect.html", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/library/test/success.html", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/ncsi.txt", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/connecttest.txt", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/redirect", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/canonical.html", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/success.txt", HTTP_GET, handleCaptivePortalRedirect);
  server.on("/api/status", HTTP_GET, sendJsonStatus);
  server.on("/api/diagnostics", HTTP_GET, handleDiagnostics);
  server.on("/api/settings", HTTP_GET, sendJsonSettings);
  server.on("/api/settings", HTTP_POST, handleSettingsUpdate);
  server.on("/api/robot/apply_wifi", HTTP_POST, handleApplyRobotWiFi);
  server.on("/api/robot/save_wifi", HTTP_POST, handleSaveRobotWiFi);
  server.on("/api/robot/connect_wifi", HTTP_POST, handleConnectRobotWiFi);
  server.on("/api/robot/disconnect_wifi", HTTP_POST, handleDisconnectRobotWiFi);
  server.on("/api/camera/apply_wifi", HTTP_POST, handleApplyCameraWiFi);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/camera/pitch", HTTP_POST, handleCameraPitch);
  server.on("/api/legs/height", HTTP_POST, handleLegHeight);
  server.on("/api/legs/height_value", HTTP_POST, handleLegHeightValue);
  server.on("/api/legs/lean", HTTP_POST, handleLegLean);
  server.on("/api/camera/track", HTTP_POST, handleTrackSelection);
  server.on("/api/camera/track_unlock", HTTP_POST, handleTrackUnlock);
  server.on("/api/camera/track_distance", HTTP_POST, handleTrackDistanceAdjust);
  server.on("/api/action", HTTP_POST, handleAction);
  server.on("/api/ota/rollback", HTTP_POST, handleRollback);
  server.on("/api/ota/upload", HTTP_POST, handleOtaFinish, handleOtaUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  websocketServer.begin();
  websocketServer.setNoDelay(true);
  Serial.printf("WebSocket control: ws://%s:%u/ws\n",
                WiFi.softAPIP().toString().c_str(), WEBSOCKET_PORT);

  for (;;) {
    server.handleClient();
    if (!statusOtaInProgress) {
      dnsServer.processNextRequest();
      pollWebSocket();
      expireLegControlCommands();
      pollRobotSta();
      pollMdnsService();
    }
    if (pendingRebootAtMs != 0 &&
        static_cast<int32_t>(millis() - pendingRebootAtMs) >= 0) {
      delay(50);
      ESP.restart();
    }
    if (pendingGamepadRebootAfterSit) {
      lockStatus();
      const bool fullySeated = statusSittingDown && !statusRobotEnabled;
      unlockStatus();
      if (fullySeated) {
        pendingGamepadRebootAfterSit = false;
        pendingRebootAtMs = millis() + 600;
      } else if (static_cast<int32_t>(millis() - pendingGamepadRebootDeadlineMs) >= 0) {
        pendingGamepadRebootAfterSit = false;
        setPrefBool(PREF_ENABLE_GAMEPAD, false);
        lockStatus();
        statusControlMode = CONTROL_MODE_WIFI;
        statusOtaMessage = "Sit action timed out; Gamepad switch cancelled";
        unlockStatus();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
}  // namespace

void initWebController() {
  if (statusMutex == nullptr) statusMutex = xSemaphoreCreateMutex();
  applyStoredSettings();
  if (actionQueue == nullptr) actionQueue = xQueueCreate(8, sizeof(WebRobotAction));
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
    Serial.println("WiFi AP config failed");
  }
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.println("WiFi AP startup failed");
    return;
  }
  dnsServer.start(53, "*", WIFI_AP_IP);
  startMdnsService("boot");
  statusRobotNetState = "AP_ONLY";
  statusRobotStaIp = "";
  staConnectInProgress = false;
  staRetryEnabled = !statusRobotWifiSsid.isEmpty();
  staLastAttemptMs = millis() - STA_RETRY_INTERVAL_MS + STA_BOOT_DELAY_MS;
  if (staRetryEnabled) {
    logRobotWiFiEvent("saved credentials loaded for " + statusRobotWifiSsid);
  }

  Serial.print("WiFi AP control: http://");
  Serial.println(WiFi.softAPIP());
  if (!statusRobotStaIp.isEmpty()) {
    Serial.print("WiFi STA control: http://");
    Serial.println(statusRobotStaIp);
  }
  xTaskCreatePinnedToCore(webServerTask, "web-controller", 12288, nullptr, 1,
                        &webTaskHandle, 0);
}

bool getWebDriveCommand(int &joyX, int &joyY) {
  int x;
  int y;
  bool active;
  unsigned long updatedAt;
  portENTER_CRITICAL(&stateMux);
  x = driveX;
  y = driveY;
  active = driveActive;
  updatedAt = lastDriveCommandMs;
  portEXIT_CRITICAL(&stateMux);

  if (!active || updatedAt == 0 || millis() - updatedAt > COMMAND_TIMEOUT_MS) return false;
  joyX = x;
  joyY = y;
  return true;
}

bool getWebGimbalYawCommand(int &joyX) {
  int x;
  unsigned long updatedAt;
  bool active;
  portENTER_CRITICAL(&stateMux);
  x = gimbalYawX;
  active = gimbalYawActive;
  updatedAt = lastGimbalYawMs;
  portEXIT_CRITICAL(&stateMux);

  if (!active || updatedAt == 0 || millis() - updatedAt > COMMAND_TIMEOUT_MS) return false;
  joyX = x;
  return true;
}
WebRobotAction consumeWebRobotAction() {
  WebRobotAction action = WebRobotAction::None;
  if (actionQueue != nullptr) xQueueReceive(actionQueue, &action, 0);
  return action;
}

int consumeWebCameraPitchDelta() {
  portENTER_CRITICAL(&stateMux);
  const int delta = pendingCameraPitchDelta;
  pendingCameraPitchDelta = 0;
  portEXIT_CRITICAL(&stateMux);
  return delta;
}

bool consumeWebLegHeightDirection(int& direction) {
  portENTER_CRITICAL(&stateMux);
  if (pendingLegHeightDirection == 99) {
    portEXIT_CRITICAL(&stateMux);
    return false;
  }
  direction = pendingLegHeightDirection;
  pendingLegHeightDirection = 99;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool consumeWebLegHeightPercent(int& percent) {
  portENTER_CRITICAL(&stateMux);
  if (pendingLegHeightPercent < 0) {
    portEXIT_CRITICAL(&stateMux);
    return false;
  }
  percent = pendingLegHeightPercent;
  pendingLegHeightPercent = -1;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool consumeWebLegLeanPercent(int& percent) {
  portENTER_CRITICAL(&stateMux);
  if (pendingLegLeanPercent == 999) {
    portEXIT_CRITICAL(&stateMux);
    return false;
  }
  percent = pendingLegLeanPercent;
  pendingLegLeanPercent = 999;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool isWebOtaInProgress() {
  lockStatus();
  const bool inProgress = statusOtaInProgress;
  unlockStatus();
  return inProgress;
}

void updateWebControllerStatus(bool robotEnabled, bool sittingDown,
                               bool gamepadConnected, float balanceAngle,
                               float batteryVoltage, int batteryPercent,
                               bool maintenanceMode,
                               int legHeightPercent, int legLeanPercent) {
  lockStatus();
  statusRobotEnabled = robotEnabled;
  statusSittingDown = sittingDown;
  statusGamepadConnected = gamepadConnected;
  statusBalanceAngle = balanceAngle;
  statusBatteryVoltage = batteryVoltage;
  statusBatteryPercent = batteryPercent;
  statusLegHeightPercent = legHeightPercent;
  statusLegLeanPercent = legLeanPercent;
  statusMaintenanceMode = maintenanceMode;
  unlockStatus();
}

void updateWebCameraNetworkStatus(const String& state, const String& ip,
                                  const String& message,
                                  const String& firmwareVersion,
                                  const String& firmwareBuild,
                                  const String& activeResolution) {
  lockStatus();
  statusCameraNetState = state;
  statusCameraNetIp = ip;
  statusCameraNetMessage = message;
  if (!firmwareVersion.isEmpty()) statusCameraFirmwareVersion = firmwareVersion;
  if (!firmwareBuild.isEmpty()) statusCameraFirmwareBuild = firmwareBuild;
  if (activeResolution == "160x120" || activeResolution == "320x240" ||
      activeResolution == "640x480" || activeResolution == "1280x720") {
    statusCameraActiveResolution = activeResolution;
  }
  const String cameraFirmwareVersionToStore = statusCameraFirmwareVersion;
  const String cameraFirmwareBuildToStore = statusCameraFirmwareBuild;
  const String cameraActiveResolutionToStore = statusCameraActiveResolution;
  String streamUrl;
  if (state == "CONNECTED" && ip.length() > 0) {
    statusCameraStreamUrl = "http://" + ip + ":8080/stream.mjpg";
    streamUrl = statusCameraStreamUrl;
  }
  unlockStatus();
  if (!streamUrl.isEmpty()) setPrefString(PREF_CAMERA_URL, streamUrl);
  if (!cameraFirmwareVersionToStore.isEmpty()) {
    setPrefString(PREF_CAMERA_FW_VERSION, cameraFirmwareVersionToStore);
  }
  if (!cameraFirmwareBuildToStore.isEmpty()) {
    setPrefString(PREF_CAMERA_FW_BUILD, cameraFirmwareBuildToStore);
  }
  if (!cameraActiveResolutionToStore.isEmpty()) {
    setPrefString(PREF_CAMERA_ACTIVE_RESOLUTION, cameraActiveResolutionToStore);
  }
  recordDiagnosticEvent("camera", state +
      (activeResolution.isEmpty() ? "" : String(" res=") + activeResolution));
}

void updateWebCameraDetectionStatus(const String& label, int count) {
  lockStatus();
  statusCameraDetectLabel = label;
  statusCameraDetectCount = count;
  unlockStatus();
}


