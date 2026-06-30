#include "control/RobotControlCoordinator.h"

#include <math.h>

#include "Diagnostics.h"
#include "RGBController.h"
#include "SerialParser.h"
#include "VoltageMonitor.h"
#include "WebController.h"
#include "camera/CameraGimbalController.h"
#include "motion/MotionCommandGateway.h"
#include "motion/MotionCoreAdapter.h"

namespace {
constexpr unsigned long kWebZeroDebounceMs = 70;
constexpr unsigned long kControlModeLedSyncMs = 250;
constexpr uint8_t kModeLedOnLevel = HIGH;
constexpr uint8_t kModeLedOffLevel = LOW;

bool trackModeActive = false;

void commandMotion(const MotionCommand& command, const char* trigger,
                   bool event = false) {
  motionGateway().dispatch(command, trigger, event);
}

void commandMotion(const MotionCommand& command, const String& trigger,
                   bool event = false) {
  motionGateway().dispatch(command, trigger, event);
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
      trackModeActive = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::Sit), trigger, true);
      break;
    case WebRobotAction::ResetPose:
      cameraGimbal().resetPose();
      commandMotion(MotionCommand::simple(MotionCommandType::ResetPose), trigger, true);
      break;
    case WebRobotAction::CancelKick:
      trackModeActive = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::TrackStop), trigger, true);
      break;
    case WebRobotAction::TrackMode:
      trackModeActive = true;
      sendCameraTrackScan();
      commandMotion(MotionCommand::simple(MotionCommandType::TrackStart),
                    "web:action:track_mode:start", true);
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
      trackModeActive = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::Sit),
                    "web:action:maintenance_enter:sit", true);
      commandMotion(MotionCommand::simple(MotionCommandType::MaintenanceEnter), trigger, true);
      break;
    case WebRobotAction::ExitMaintenance:
      commandMotion(MotionCommand::simple(MotionCommandType::MaintenanceExit), trigger, true);
      break;
    case WebRobotAction::None:
      break;
  }
}

#if ENABLE_GAMEPAD_BLE
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
}  // namespace

void syncControlModeIndicatorState(bool gamepadEnabled, bool force) {
  static unsigned long lastSyncMs = 0;
  const unsigned long now = millis();
  if (!force && now - lastSyncMs < kControlModeLedSyncMs) return;
  lastSyncMs = now;
  digitalWrite(LED_BAT, gamepadEnabled ? kModeLedOnLevel : kModeLedOffLevel);
}

void processWebControlState(bool gamepadEnabled) {
  static bool wasDriving = false;
  static int lastNonZeroWebJoyX = 0;
  static int lastNonZeroWebJoyY = 0;
  static unsigned long webZeroSinceMs = 0;
  static unsigned long lastTrackScanRetryMs = 0;
  static int lastLegHeightPercentSent = -1;
  static unsigned long lastLegHeightPercentDispatchMs = 0;

  dispatchWebAction(consumeWebRobotAction());
  if (trackModeActive && isWebCameraScanPending() &&
      millis() - lastTrackScanRetryMs >= 1000) {
    lastTrackScanRetryMs = millis();
    sendCameraTrackScan();
  }

  int legHeightPercent = -1;
  if (consumeWebLegHeightPercent(legHeightPercent)) {
    const unsigned long now = millis();
    if (legHeightPercent != lastLegHeightPercentSent ||
        now - lastLegHeightPercentDispatchMs >= 40) {
      commandMotion(MotionCommand::legHeightPercent(legHeightPercent),
                    String("web:leg_height_value:") + legHeightPercent);
      lastLegHeightPercentSent = legHeightPercent;
      lastLegHeightPercentDispatchMs = now;
    }
  } else {
    lastLegHeightPercentSent = -1;
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
    const String trigger = String("web:gimbal_pitch:") + cameraPitchDelta;
    setMotionTrigger(trigger);
    cameraGimbal().pitchDelta(cameraPitchDelta);
  }

  int guardServoAngle = -1;
  if (consumeWebGuardServoAngle(guardServoAngle)) {
    commandMotion(MotionCommand::guardServo(guardServoAngle),
                  String("web:guard:") + guardServoAngle);
  }

  int joyX = 0;
  int joyY = 0;
  if (!gamepadEnabled && getWebDriveCommand(joyX, joyY)) {
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
  } else if (!gamepadEnabled && getWebGimbalYawCommand(joyX)) {
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

void publishControlStatus(bool gamepadConnected) {
  const BatteryStatus battery = getBatteryStatus();
  const MotionTelemetry telemetry = motionCore().telemetry();
  updateWebControllerStatus(telemetry.enabled, telemetry.sitting,
                            gamepadConnected,
                            telemetry.balanceAngleDeg,
                            battery.voltage, battery.percent,
                            telemetry.maintenance,
                            telemetry.legHeightPercent,
                            telemetry.legLeanPercent);
}

bool isTrackModeActive() { return trackModeActive; }

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
    const int direction = gimbalPitch > 0 ? 1 : -1;
    const int delta = direction * constrain((abs(gimbalPitch) + 32) / 33, 1, 3);
    setMotionTrigger(String("gamepad:gimbal_pitch:") + delta);
    cameraGimbal().pitchDelta(delta);
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
