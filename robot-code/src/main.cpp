#include <Arduino.h>
#include <esp_ota_ops.h>
#include <math.h>

#include "PreferencesManager.h"
#include "Diagnostics.h"
#include "RGBController.h"
#include "SerialParser.h"
#include "VoltageMonitor.h"
#include "WebController.h"
#include "camera/CameraGimbalController.h"
#include "control/RobotControlCoordinator.h"
#include "motion/MotionCoreAdapter.h"
#include "motion/MotionCommandGateway.h"

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
    case WebRobotAction::DanceDemo: return "waltz_show";
    case WebRobotAction::StopDanceDemo: return "waltz_stop";
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
  motionGateway().dispatch(command, trigger, event);
}

void commandMotion(const MotionCommand& command, const String& trigger,
                   bool event = false) {
  motionGateway().dispatch(command, trigger, event);
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
  syncControlModeIndicatorState(gamepad_enabled, force);
}

void processWebControl() {
  processWebControlState(gamepad_enabled);
}

void publishStatus() {
  const bool gamepadConnected =
#if ENABLE_GAMEPAD_BLE
      gamepad_enabled && gamepadController.isConnected();
#else
      false;
#endif
  publishControlStatus(gamepadConnected);
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
#endif

void setup() {
  pinMode(LED_BAT, OUTPUT);
  digitalWrite(LED_BAT, kModeLedOffLevel);
  Serial.begin(115200);
  initCameraSerial();
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
  cameraGimbal().begin();

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
  cameraGimbal().update();
  if (!otaRunning && millis() - lastStatusPublishMs >= 50) {
    lastStatusPublishMs = millis();
    publishStatus();
  }
  delay(2);
}


