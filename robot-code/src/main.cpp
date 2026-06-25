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
  switch (action) {
    case WebRobotAction::Stand:
      motionCore().command(MotionCommand::simple(MotionCommandType::Stand));
      break;
    case WebRobotAction::Sit:
      trace_mode = false;
      sendCameraTrackStop();
      motionCore().command(MotionCommand::simple(MotionCommandType::Sit));
      break;
    case WebRobotAction::ResetPose:
      motionCore().command(MotionCommand::simple(MotionCommandType::ResetPose));
      break;
    case WebRobotAction::CancelKick:
      trace_mode = false;
      sendCameraTrackStop();
      motionCore().command(MotionCommand::simple(MotionCommandType::TrackStop));
      break;
    case WebRobotAction::TrackMode:
      trace_mode = true;
      motionCore().command(MotionCommand::simple(MotionCommandType::Stand));
      motionCore().command(MotionCommand::simple(MotionCommandType::TrackStart));
      break;
    case WebRobotAction::LedTest:
      startColorSequenceBlink();
      break;
    case WebRobotAction::Jump:
      motionCore().command(MotionCommand::simple(MotionCommandType::JumpPlace));
      break;
    case WebRobotAction::JumpForward:
      motionCore().command(MotionCommand::simple(MotionCommandType::JumpForward));
      break;
    case WebRobotAction::JumpBackward:
      motionCore().command(MotionCommand::simple(MotionCommandType::JumpBackward));
      break;
    case WebRobotAction::JumpLeft:
      motionCore().command(MotionCommand::simple(MotionCommandType::JumpLeft));
      break;
    case WebRobotAction::JumpRight:
      motionCore().command(MotionCommand::simple(MotionCommandType::JumpRight));
      break;
    case WebRobotAction::EnterMaintenance:
      trace_mode = false;
      sendCameraTrackStop();
      motionCore().command(MotionCommand::simple(MotionCommandType::Sit));
      motionCore().command(MotionCommand::simple(MotionCommandType::MaintenanceEnter));
      break;
    case WebRobotAction::ExitMaintenance:
      motionCore().command(MotionCommand::simple(MotionCommandType::MaintenanceExit));
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
    motionCore().command(MotionCommand::legHeightPercent(legHeightPercent));
  }
  int legHeightDirection = 0;
  if (consumeWebLegHeightDirection(legHeightDirection)) {
    motionCore().command(MotionCommand::legHeight(legHeightDirection));
  }
  int legLeanPercent = 0;
  if (consumeWebLegLeanPercent(legLeanPercent)) {
    motionCore().command(MotionCommand::legLean(legLeanPercent));
  }
  const int cameraPitchDelta = consumeWebCameraPitchDelta();
  if (cameraPitchDelta != 0) {
    MotionCommand cameraCommand;
    cameraCommand.type = MotionCommandType::CameraGimbal;
    cameraCommand.y = cameraPitchDelta;
    motionCore().command(cameraCommand);
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
    motionCore().command(MotionCommand::move(joyX, joyY));
    wasDriving = true;
  } else if (!gamepad_enabled && getWebGimbalYawCommand(joyX)) {
    motionCore().command(MotionCommand::move(joyX, 0));
    wasDriving = true;
  } else if (wasDriving) {
    lastNonZeroWebJoyX = 0;
    lastNonZeroWebJoyY = 0;
    webZeroSinceMs = 0;
    motionCore().command(MotionCommand::simple(MotionCommandType::Stop));
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
    motionCore().command(MotionCommand::simple(MotionCommandType::Stand));
  }
  if (data.btnLB && !previousLb) {
    motionCore().command(MotionCommand::simple(MotionCommandType::Sit));
  }
  if (data.btnRS && !previousRs) motionCore().command(MotionCommand::simple(MotionCommandType::JumpPlace));
  if (data.btnY && !previousY) motionCore().command(MotionCommand::simple(MotionCommandType::JumpForward));
  if (data.btnA && !previousA) motionCore().command(MotionCommand::simple(MotionCommandType::JumpBackward));
  if (data.btnX && !previousX) motionCore().command(MotionCommand::simple(MotionCommandType::JumpLeft));
  if (data.btnB && !previousB) motionCore().command(MotionCommand::simple(MotionCommandType::JumpRight));

  previousRb = data.btnRB;
  previousLb = data.btnLB;
  previousRs = data.btnRS;
  previousY = data.btnY;
  previousA = data.btnA;
  previousX = data.btnX;
  previousB = data.btnB;

  motionCore().command(MotionCommand::legLean(
      data.btnDirLeft ? -100 : (data.btnDirRight ? 100 : 0)));
  motionCore().command(MotionCommand::legHeight(
      data.btnDirUp ? 1 : (data.btnDirDown ? -1 : 0)));

  int joyX = axisToPercent(data.joyLHori);
  int joyY = axisToPercent(data.joyLVert, true);
  normalizeStick(joyX, joyY);

  const int gimbalYaw = axisToPercent(data.joyRHori);
  if (gimbalYaw != 0) joyX = gimbalYaw;
  motionCore().command(MotionCommand::move(joyX, joyY));

  const unsigned long now = millis();
  const int gimbalPitch = axisToPercent(data.joyRVert, true);
  if (gimbalPitch != 0 && now - lastGimbalCommandMs >= 50) {
    MotionCommand command;
    command.type = MotionCommandType::CameraGimbal;
    const int direction = gimbalPitch > 0 ? 1 : -1;
    command.y = direction * constrain((abs(gimbalPitch) + 32) / 33, 1, 3);
    motionCore().command(command);
    lastGimbalCommandMs = now;
  }
}

void stopControllerMotion() {
  motionCore().command(MotionCommand::simple(MotionCommandType::Stop));
  motionCore().command(MotionCommand::legLean(0));
  motionCore().command(MotionCommand::legHeight(0));
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
    motionCore().command(MotionCommand::simple(MotionCommandType::MaintenanceEnter));
    motionCore().command(MotionCommand::simple(MotionCommandType::Stop));
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


