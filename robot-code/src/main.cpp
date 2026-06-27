#include <Arduino.h>
#include <esp_ota_ops.h>
#include <math.h>

#include "PreferencesManager.h"
#include "Diagnostics.h"
#include "RGBController.h"
#include "SerialParser.h"
#include "VoltageMonitor.h"
#include "WebController.h"
#include "motion/MotionCoreAdapter.h"

#ifndef ENABLE_GAMEPAD_BLE
#define ENABLE_GAMEPAD_BLE 0
#endif

#if ENABLE_GAMEPAD_BLE
#include "GamepadBluetooth.h"
#endif

#define PREF_ENABLE_GAMEPAD "enable_gamepad"
#define CONTROL_RECOVERY_PIN 34

float pid_cam_p = 0.07f;
float pid_cam_d = 0.05f;
int yaw_align_threshold = 10;
bool trace_mode = false;
bool gamepad_enabled = false;

namespace {
constexpr unsigned long kReadySelfCheckDelayMs = 1200;
constexpr unsigned long kWebZeroDebounceMs = 70;
constexpr unsigned long kControlModeLedSyncMs = 250;
constexpr uint8_t kModeLedOnLevel = HIGH;
constexpr uint8_t kModeLedOffLevel = LOW;
unsigned long setupCompletedAtMs = 0;
bool readyIndicated = false;

bool recoveryButtonHeldAtBoot() {
  // GPIO34 is input-only and has no internal pull-up. The board supplies the
  // button bias, so require a stable LOW instead of trusting one boot sample.
  int lowSamples = 0;
  for (int sample = 0; sample < 12; ++sample) {
    if (digitalRead(CONTROL_RECOVERY_PIN) == LOW) ++lowSamples;
    delay(4);
  }
  return lowSamples == 12;
}

const char* webActionName(WebRobotAction action) {
  switch (action) {
    case WebRobotAction::Stand: return "stand";
    case WebRobotAction::Sit: return "sit";
    case WebRobotAction::ResetPose: return "reset";
    case WebRobotAction::CancelKick: return "cancel";
    case WebRobotAction::TrackMode: return "track_mode";
    case WebRobotAction::LedTest: return "led_test";
    case WebRobotAction::Jump: return "jump_place";
    case WebRobotAction::JumpForward: return "jump_forward";
    case WebRobotAction::JumpBackward: return "jump_backward";
    case WebRobotAction::JumpLeft: return "jump_left";
    case WebRobotAction::JumpRight: return "jump_right";
    case WebRobotAction::EnterMaintenance: return "maintenance_enter";
    case WebRobotAction::ExitMaintenance: return "maintenance_exit";
    case WebRobotAction::None:
    default:
      return "none";
  }
}

void commandMotion(const MotionCommand& command, const char* trigger,
                   bool event = false) {
  setMotionTrigger(trigger);
  if (event) recordDiagnosticEvent("motion", trigger);
  motionCore().command(command);
}

void commandMotion(const MotionCommand& command, const String& trigger,
                   bool event = false) {
  setMotionTrigger(trigger);
  if (event) recordDiagnosticEvent("motion", trigger);
  motionCore().command(command);
}
#if ENABLE_GAMEPAD_BLE
// Match the 5% dead zone used by the shared motion core so Web and gamepad axes
// enter the same response curve.
constexpr int kStickDeadZone = 5;
constexpr int kGamepadHighSpeedPercent = 100;

int axisToPercent(uint16_t raw, bool invert = false,
                  int speedPercent = kGamepadHighSpeedPercent) {
  int value = map((int)raw, 0, GamepadControllerNotificationParser::maxJoy, -100, 100);
  if (invert) value = -value;
  if (abs(value) < kStickDeadZone) return 0;
  return constrain(value * speedPercent / 100, -100, 100);
}

void normalizeStick(int& x, int& y) {
  const float magnitude = sqrtf((float)x * x + (float)y * y);
  if (magnitude <= 100.0f || magnitude <= 0.0f) return;
  x = (int)roundf((float)x * 100.0f / magnitude);
  y = (int)roundf((float)y * 100.0f / magnitude);
}
#endif

void confirmRunningFirmwareIfNeeded() {
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(esp_ota_get_running_partition(), &state);
  if (state == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    Serial.println("[WROBOT] OTA image confirmed");
  }
}

void syncControlModeIndicator(bool force = false) {
  static unsigned long lastSyncMs = 0;
  static bool lastLevel = false;
  const unsigned long now = millis();
  if (!force && now - lastSyncMs < kControlModeLedSyncMs) return;
  lastSyncMs = now;

  const bool level =
#if ENABLE_GAMEPAD_BLE
      gamepad_enabled;
#else
      false;
#endif
  digitalWrite(LED_BAT, level ? kModeLedOnLevel : kModeLedOffLevel);
  lastLevel = level;
}

void dispatchWebAction(WebRobotAction action) {
  if (action != WebRobotAction::None) {
    recordDiagnosticEvent("action", String("id=") + String((int)action));
  }
  const String trigger = String("web:action:") + webActionName(action);
  switch (action) {
    case WebRobotAction::Stand:
      commandMotion(MotionCommand::simple(MotionCommandType::Stand), trigger, true);
      break;
    case WebRobotAction::Sit:
      trace_mode = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::Sit), trigger, true);
      break;
    case WebRobotAction::ResetPose:
      commandMotion(MotionCommand::simple(MotionCommandType::ResetPose), trigger, true);
      break;
    case WebRobotAction::CancelKick:
      trace_mode = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::TrackStop), trigger, true);
      break;
    case WebRobotAction::TrackMode:
      trace_mode = true;
      sendCameraTrackScan();
      commandMotion(MotionCommand::simple(MotionCommandType::TrackStart), "web:action:track_mode:start", true);
      break;
    case WebRobotAction::LedTest:
      startColorSequenceBlink();
      break;
    case WebRobotAction::Jump:
      commandMotion(MotionCommand::simple(MotionCommandType::JumpPlace), trigger, true);
      break;
    case WebRobotAction::JumpForward:
      commandMotion(MotionCommand::simple(MotionCommandType::JumpForward), trigger, true);
      break;
    case WebRobotAction::JumpBackward:
      commandMotion(MotionCommand::simple(MotionCommandType::JumpBackward), trigger, true);
      break;
    case WebRobotAction::JumpLeft:
      commandMotion(MotionCommand::simple(MotionCommandType::JumpLeft), trigger, true);
      break;
    case WebRobotAction::JumpRight:
      commandMotion(MotionCommand::simple(MotionCommandType::JumpRight), trigger, true);
      break;
    case WebRobotAction::EnterMaintenance:
      trace_mode = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::Sit), "web:action:maintenance_enter:sit", true);
      commandMotion(MotionCommand::simple(MotionCommandType::MaintenanceEnter), trigger, true);
      break;
    case WebRobotAction::ExitMaintenance:
      commandMotion(MotionCommand::simple(MotionCommandType::MaintenanceExit), trigger, true);
      break;
    case WebRobotAction::None:
      break;
  }
}

void processWebControl() {
  static bool wasDriving = false;
  static int lastNonZeroWebJoyX = 0;
  static int lastNonZeroWebJoyY = 0;
  static unsigned long webZeroSinceMs = 0;
  dispatchWebAction(consumeWebRobotAction());
  int legHeightPercent = -1;
  if (consumeWebLegHeightPercent(legHeightPercent)) {
    commandMotion(MotionCommand::legHeightPercent(legHeightPercent),
                  String("web:leg_height_value:") + legHeightPercent);
  }
  int legHeightDirection = 0;
  if (consumeWebLegHeightDirection(legHeightDirection)) {
    commandMotion(MotionCommand::legHeight(legHeightDirection),
                  String("web:leg_height:") + legHeightDirection);
  }
  int legLeanPercent = 0;
  if (consumeWebLegLeanPercent(legLeanPercent)) {
    commandMotion(MotionCommand::legLean(legLeanPercent),
                  String("web:leg_lean:") + legLeanPercent);
  }
  const int cameraPitchDelta = consumeWebCameraPitchDelta();
  if (cameraPitchDelta != 0) {
    MotionCommand cameraCommand;
    cameraCommand.type = MotionCommandType::CameraGimbal;
    cameraCommand.y = cameraPitchDelta;
    commandMotion(cameraCommand, String("web:gimbal_pitch:") + cameraPitchDelta);
  }

  int joyX = 0;
  int joyY = 0;
  if (!gamepad_enabled && getWebDriveCommand(joyX, joyY)) {
    if (joyX != 0 || joyY != 0) {
      lastNonZeroWebJoyX = joyX;
      lastNonZeroWebJoyY = joyY;
      webZeroSinceMs = 0;
    } else if (wasDriving) {
      const unsigned long now = millis();
      if (webZeroSinceMs == 0) webZeroSinceMs = now;
      if (now - webZeroSinceMs < kWebZeroDebounceMs) {
        joyX = lastNonZeroWebJoyX;
        joyY = lastNonZeroWebJoyY;
      } else {
        lastNonZeroWebJoyX = 0;
        lastNonZeroWebJoyY = 0;
      }
    }
    commandMotion(MotionCommand::move(joyX, joyY),
                  String("web:drive:x=") + joyX + ",y=" + joyY);
    wasDriving = true;
  } else if (!gamepad_enabled && getWebGimbalYawCommand(joyX)) {
    commandMotion(MotionCommand::move(joyX, 0),
                  String("web:gimbal_yaw:") + joyX);
    wasDriving = true;
  } else if (wasDriving) {
    lastNonZeroWebJoyX = 0;
    lastNonZeroWebJoyY = 0;
    webZeroSinceMs = 0;
    commandMotion(MotionCommand::simple(MotionCommandType::Stop),
                  "web:drive:stop");
    wasDriving = false;
  }
}

void publishStatus() {
  const BatteryStatus battery = getBatteryStatus();
  const MotionTelemetry telemetry = motionCore().telemetry();
  const bool gamepadConnected =
#if ENABLE_GAMEPAD_BLE
      gamepad_enabled && gamepadController.isConnected();
#else
      false;
#endif
  updateWebControllerStatus(telemetry.enabled, telemetry.sitting,
                            gamepadConnected,
                            telemetry.balanceAngleDeg,
                            battery.voltage, battery.percent,
                            telemetry.maintenance,
                            telemetry.legHeightPercent,
                            telemetry.legLeanPercent);
}

void updateSystemIndicator() {
  const BatteryStatus battery = getBatteryStatus();
  const bool lowBattery = battery.valid && battery.low;
  setLowBatteryWarning(lowBattery);
  if (readyIndicated || lowBattery || !battery.valid ||
      millis() - setupCompletedAtMs < kReadySelfCheckDelayMs) {
    return;
  }
  if (!motionCore().selfCheckPassed()) return;

  readyIndicated = true;
  confirmRunningFirmwareIfNeeded();
  indicateSystemReady();
  recordDiagnosticEvent("system", "ready");
  Serial.println("[WROBOT] Self-check passed - ready");
}
}  // namespace

#if ENABLE_GAMEPAD_BLE
void processControllerData(const GamepadControllerNotificationParser& data) {
  static bool previousRb = false;
  static bool previousLb = false;
  static bool previousRs = false;
  static bool previousY = false;
  static bool previousA = false;
  static bool previousX = false;
  static bool previousB = false;
  static unsigned long lastGimbalCommandMs = 0;

  if (data.btnRB && !previousRb) {
    commandMotion(MotionCommand::simple(MotionCommandType::Stand),
                  "gamepad:button:stand", true);
  }
  if (data.btnLB && !previousLb) {
    commandMotion(MotionCommand::simple(MotionCommandType::Sit),
                  "gamepad:button:sit", true);
  }
  if (data.btnRS && !previousRs) commandMotion(MotionCommand::simple(MotionCommandType::JumpPlace), "gamepad:button:jump_place", true);
  if (data.btnY && !previousY) commandMotion(MotionCommand::simple(MotionCommandType::JumpForward), "gamepad:button:jump_forward", true);
  if (data.btnA && !previousA) commandMotion(MotionCommand::simple(MotionCommandType::JumpBackward), "gamepad:button:jump_backward", true);
  if (data.btnX && !previousX) commandMotion(MotionCommand::simple(MotionCommandType::JumpLeft), "gamepad:button:jump_left", true);
  if (data.btnB && !previousB) commandMotion(MotionCommand::simple(MotionCommandType::JumpRight), "gamepad:button:jump_right", true);

  previousRb = data.btnRB;
  previousLb = data.btnLB;
  previousRs = data.btnRS;
  previousY = data.btnY;
  previousA = data.btnA;
  previousX = data.btnX;
  previousB = data.btnB;

  commandMotion(MotionCommand::legLean(
                    data.btnDirLeft ? -100 : (data.btnDirRight ? 100 : 0)),
                "gamepad:dpad:lean");
  commandMotion(MotionCommand::legHeight(
                    data.btnDirUp ? 1 : (data.btnDirDown ? -1 : 0)),
                "gamepad:dpad:height");

  int joyX = axisToPercent(data.joyLHori);
  int joyY = axisToPercent(data.joyLVert, true);
  normalizeStick(joyX, joyY);

  const int gimbalYaw = axisToPercent(data.joyRHori);
  if (gimbalYaw != 0) joyX = gimbalYaw;
  commandMotion(MotionCommand::move(joyX, joyY),
                String("gamepad:move:x=") + joyX + ",y=" + joyY);

  const unsigned long now = millis();
  const int gimbalPitch = axisToPercent(data.joyRVert, true);
  if (gimbalPitch != 0 && now - lastGimbalCommandMs >= 50) {
    MotionCommand command;
    command.type = MotionCommandType::CameraGimbal;
    const int direction = gimbalPitch > 0 ? 1 : -1;
    command.y = direction * constrain((abs(gimbalPitch) + 32) / 33, 1, 3);
    commandMotion(command, String("gamepad:gimbal_pitch:") + command.y);
    lastGimbalCommandMs = now;
  }
}

void stopControllerMotion() {
  commandMotion(MotionCommand::simple(MotionCommandType::Stop),
                "gamepad:disconnect:stop", true);
  commandMotion(MotionCommand::legLean(0), "gamepad:disconnect:lean_zero");
  commandMotion(MotionCommand::legHeight(0), "gamepad:disconnect:height_zero");
}
#endif

void setup() {
  pinMode(LED_BAT, OUTPUT);
  digitalWrite(LED_BAT, kModeLedOffLevel);
  Serial.begin(115200);
  delay(300);
  recordDiagnosticEvent("system", "boot");

  pinMode(CONTROL_RECOVERY_PIN, INPUT);
  initPreferencesManager();

  adc_calibration_init();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);
  forceBatteryMeasurement();

  initLEDs();

  gamepad_enabled = getPrefBool(PREF_ENABLE_GAMEPAD, false);
  if (recoveryButtonHeldAtBoot()) {
    gamepad_enabled = false;
    setPrefBool(PREF_ENABLE_GAMEPAD, false);
  }

  motionCore().begin();

#if ENABLE_GAMEPAD_BLE
  if (gamepad_enabled) {
    syncControlModeIndicator(true);
    gamepadControllerInit();
    Serial.println("[WROBOT] Control mode: Gamepad");
  } else
#endif
  {
    syncControlModeIndicator(true);
    if (gamepad_enabled) {
      gamepad_enabled = false;
      setPrefBool(PREF_ENABLE_GAMEPAD, false);
      syncControlModeIndicator(true);
      Serial.println("[WROBOT] Gamepad BLE not compiled; reverted to Web/WiFi");
    }
    initWebController();
    Serial.println("[WROBOT] Control mode: Web/WiFi");
  }

  setupCompletedAtMs = millis();
}

void loop() {
  static bool otaMotionLocked = false;
  static unsigned long lastStatusPublishMs = 0;
  const bool otaRunning = isWebOtaInProgress();
  syncControlModeIndicator();
  if (!otaRunning) {
    serialReceiveProcess();
  }
  handleLEDBlink();

#if ENABLE_GAMEPAD_BLE
  if (!otaRunning && gamepad_enabled) {
    process_gamepad_notif();
    updateGamepadVibration();
  }
#endif

  if (!otaRunning) {
    processWebControl();
    bat_check();
    updateSystemIndicator();
  }

  if (otaRunning && !otaMotionLocked) {
    commandMotion(MotionCommand::simple(MotionCommandType::MaintenanceEnter),
                  "ota:maintenance_enter", true);
    commandMotion(MotionCommand::simple(MotionCommandType::Stop),
                  "ota:stop", true);
    otaMotionLocked = true;
  } else if (!otaRunning) {
    otaMotionLocked = false;
  }

  motionCore().update();
  if (!otaRunning && millis() - lastStatusPublishMs >= 50) {
    lastStatusPublishMs = millis();
    publishStatus();
  }
  delay(2);
}


