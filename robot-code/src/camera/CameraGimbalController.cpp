#include "CameraGimbalController.h"

#include <math.h>

#include "devices/ptk7350.h"

namespace {
constexpr int kCameraMinDeg = 45;
constexpr int kCameraMaxDeg = 150;
constexpr int kCameraStandbyDeg = 105;
constexpr float kCameraSlewDegPerSecond = 70.0f;
constexpr uint32_t kCameraCalibrationHighMs = 1200;
constexpr uint32_t kCameraCalibrationLowMs = 3000;
constexpr uint32_t kCameraCalibrationFinishMs = 4500;
constexpr uint32_t kTrackPitchIntervalMs = 24;
constexpr int kTrackPitchSafeError = 333;
constexpr int kTrackPitchStepDeg = 2;
constexpr int kTrackPitchMinConfidence = 450;

float approach(float current, float target, float maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}
}  // namespace

CameraGimbalController& cameraGimbal() {
  static CameraGimbalController instance;
  return instance;
}

void CameraGimbalController::begin() {
  angleDeg_ = kCameraStandbyDeg;
  targetDeg_ = kCameraMaxDeg;
  cam_servo.set_angle((uint16_t)roundf(angleDeg_));
  calibrationActive_ = true;
  calibrationStartedMs_ = millis();
  lastUpdateMs_ = 0;
}

void CameraGimbalController::update() {
  const uint32_t now = millis();

  if (calibrationActive_) {
    const uint32_t elapsed = now - calibrationStartedMs_;
    if (elapsed < kCameraCalibrationHighMs) {
      targetDeg_ = kCameraMaxDeg;
    } else if (elapsed < kCameraCalibrationLowMs) {
      targetDeg_ = kCameraMinDeg;
    } else {
      targetDeg_ = kCameraStandbyDeg;
    }
  }

  const uint32_t elapsedMs = lastUpdateMs_ == 0 ? 2 : now - lastUpdateMs_;
  lastUpdateMs_ = now;
  const float dt = min(elapsedMs, (uint32_t)50) / 1000.0f;
  const float previous = angleDeg_;
  angleDeg_ = approach(angleDeg_, targetDeg_, kCameraSlewDegPerSecond * dt);
  if ((int)roundf(previous) != (int)roundf(angleDeg_)) {
    cam_servo.set_angle((uint16_t)roundf(angleDeg_));
  }

  if (calibrationActive_ &&
      now - calibrationStartedMs_ >= kCameraCalibrationFinishMs &&
      fabsf(angleDeg_ - (float)kCameraStandbyDeg) < 1.0f) {
    angleDeg_ = kCameraStandbyDeg;
    targetDeg_ = kCameraStandbyDeg;
    cam_servo.set_angle(kCameraStandbyDeg);
    calibrationActive_ = false;
  }
}

void CameraGimbalController::pitchDelta(int deltaDeg) {
  if (calibrationActive_) return;
  targetDeg_ = constrain(targetDeg_ + deltaDeg,
                         (float)kCameraMinDeg,
                         (float)kCameraMaxDeg);
}

void CameraGimbalController::setTargetAngle(int angleDeg) {
  if (calibrationActive_) return;
  targetDeg_ = constrain((float)angleDeg,
                         (float)kCameraMinDeg,
                         (float)kCameraMaxDeg);
}


void CameraGimbalController::trackVertical(int normalizedErrorY, bool locked,
                                           int confidence) {
  lastTrackSeenMs_ = millis();
  lastTrackErrorY_ = normalizedErrorY;
  lastTrackConfidence_ = confidence;
  lastTrackStepDeg_ = 0;

  if (calibrationActive_ || !locked || confidence < kTrackPitchMinConfidence) {
    return;
  }

  if (abs(normalizedErrorY) <= kTrackPitchSafeError) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastTrackPitchMs_ < kTrackPitchIntervalMs) {
    return;
  }
  lastTrackPitchMs_ = now;

  const int delta = normalizedErrorY > 0 ? -kTrackPitchStepDeg
                                         : kTrackPitchStepDeg;
  lastTrackStepDeg_ = delta;
  pitchDelta(delta);
}

void CameraGimbalController::resetPose() {
  targetDeg_ = kCameraStandbyDeg;
  lastTrackStepDeg_ = 0;
}

CameraGimbalTelemetry CameraGimbalController::telemetry() const {
  CameraGimbalTelemetry data;
  data.angleDeg = angleDeg_;
  data.targetDeg = targetDeg_;
  data.calibrating = calibrationActive_;
  data.trackingActive = lastTrackSeenMs_ != 0 && millis() - lastTrackSeenMs_ < 500;
  data.trackingErrorY = lastTrackErrorY_;
  data.trackingConfidence = lastTrackConfidence_;
  data.trackingAgeMs = lastTrackSeenMs_ == 0 ? 0 : millis() - lastTrackSeenMs_;
  data.trackingStepDeg = lastTrackStepDeg_;
  return data;
}
