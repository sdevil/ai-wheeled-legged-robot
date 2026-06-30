#include "control/RobotControlCoordinator.h"

#include <math.h>
#include <string.h>

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
constexpr uint32_t kWaltzPresentationFrameMs = 40;
constexpr int kWaltzCameraNeutralDeg = 105;
constexpr int kWaltzCameraUpMaxDeg = 119;
constexpr int kWaltzCameraDownMaxDeg = 93;

bool trackModeActive = false;

bool waltzPresentationActive = false;
uint32_t lastWaltzPresentationMs = 0;

CRGB lerpColor(const CRGB& a, const CRGB& b, float t) {
  t = constrain(t, 0.0f, 1.0f);
  return CRGB(
      (uint8_t)roundf(a.r + (b.r - a.r) * t),
      (uint8_t)roundf(a.g + (b.g - a.g) * t),
      (uint8_t)roundf(a.b + (b.b - a.b) * t));
}

float easeInOutSine(float t) {
  t = constrain(t, 0.0f, 1.0f);
  return 0.5f - 0.5f * cosf(t * PI);
}

float interpolateKeyframes(const uint32_t* timesMs, const int* values,
                           size_t count, uint32_t timeMs) {
  if (count == 0) return 0.0f;
  if (timeMs <= timesMs[0]) return (float)values[0];
  for (size_t i = 1; i < count; ++i) {
    if (timeMs <= timesMs[i]) {
      const uint32_t span = max<uint32_t>(1, timesMs[i] - timesMs[i - 1]);
      const float t = easeInOutSine((float)(timeMs - timesMs[i - 1]) / (float)span);
      return values[i - 1] + (values[i] - values[i - 1]) * t;
    }
  }
  return (float)values[count - 1];
}

CRGB colorForWaltzTime(uint32_t elapsedMs) {
  if (elapsedMs < 15000U) {
    return lerpColor(CRGB(36, 16, 4), CRGB(84, 48, 14),
                     constrain((float)elapsedMs / 15000.0f, 0.0f, 1.0f));
  }
  if (elapsedMs < 35000U) return CRGB(148, 110, 36);
  if (elapsedMs < 55000U) return CRGB(90, 118, 190);
  if (elapsedMs < 75000U) return CRGB(60, 88, 178);
  if (elapsedMs < 95000U) return CRGB(176, 136, 44);
  if (elapsedMs < 115000U) return CRGB(82, 112, 204);
  return CRGB(30, 54, 150);
}

void updateWaltzPresentation() {
  const MotionTelemetry telemetry = motionCore().telemetry();
  const bool active = strcmp(telemetry.mode, "dance_demo") == 0;
  if (!active) {
    if (waltzPresentationActive) {
      waltzPresentationActive = false;
      lastWaltzPresentationMs = 0;
      stopLEDBlink();
      cameraGimbal().setTargetAngle(kWaltzCameraNeutralDeg);
    }
    return;
  }

  const uint32_t now = millis();
  if (now - lastWaltzPresentationMs < kWaltzPresentationFrameMs) return;
  lastWaltzPresentationMs = now;
  waltzPresentationActive = true;

  const uint32_t delayedMs =
      telemetry.danceElapsedMs > 300U ? telemetry.danceElapsedMs - 300U : 0U;
  static const uint32_t kHeadTimesMs[] = {
      0, 2000, 5300, 8500, 10000, 14000,
      15300, 19000, 22300, 24300, 27000, 29000, 32000, 34000,
      35300, 38500, 41500, 44500, 47300, 50000, 55300, 60000,
      63000, 69300, 76000, 78500, 81000, 85500, 92000, 95300,
      99000, 106000, 109000, 113000, 116000, 118000, 119000, 120000
  };
  static const int kHeadAnglesDeg[] = {
      -10, -3, 3, 0, 0, 0,
      6, -4, 0, 7, 0, -4, 0, 0,
      6, -5, 0, 8, -4, 0, 7, 0,
      -4, 10, 12, 0, -4, 0, 0, 6,
      9, -8, 0, 0, -10, 6, 0, 0
  };
  const int cameraTarget = constrain(
      (int)roundf(kWaltzCameraNeutralDeg +
                  interpolateKeyframes(kHeadTimesMs, kHeadAnglesDeg,
                                       sizeof(kHeadTimesMs) / sizeof(kHeadTimesMs[0]),
                                       delayedMs)),
      kWaltzCameraDownMaxDeg, kWaltzCameraUpMaxDeg);
  cameraGimbal().setTargetAngle(cameraTarget);

  const uint32_t beatPhaseMs = telemetry.danceElapsedMs % 1000U;
  const float beatPhase = beatPhaseMs / 1000.0f;
  const CRGB base = colorForWaltzTime(telemetry.danceElapsedMs);
  const CRGB accent =
      telemetry.danceElapsedMs < 15000U ? CRGB(150, 110, 50)
      : telemetry.danceElapsedMs < 35000U ? CRGB(220, 180, 96)
      : telemetry.danceElapsedMs < 95000U ? CRGB(120, 180, 255)
      : CRGB(255, 204, 100);
  const float pulse = beatPhase < 0.24f ? (1.0f - beatPhase / 0.24f) : 0.0f;
  setLEDColor(lerpColor(base, accent, pulse * 0.82f));
}

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
    case WebRobotAction::DanceDemo:
      trackModeActive = false;
      sendCameraTrackStop();
      commandMotion(MotionCommand::simple(MotionCommandType::TrackStop),
                    "web:action:dance_demo:track_stop", false);
      commandMotion(MotionCommand::simple(MotionCommandType::DanceDemoStart),
                    trigger, true);
      break;
    case WebRobotAction::StopDanceDemo:
      commandMotion(MotionCommand::simple(MotionCommandType::DanceDemoStop),
                    trigger, true);
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
  updateWaltzPresentation();
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

const char* activeUiMode() {
  if (trackModeActive) return "track_mode";
  const MotionTelemetry telemetry = motionCore().telemetry();
  if (strcmp(telemetry.mode, "dance_demo") == 0) return "waltz_show";
  return "";
}

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
