#ifndef CAMERA_GIMBAL_CONTROLLER_H
#define CAMERA_GIMBAL_CONTROLLER_H

#include <Arduino.h>

struct CameraGimbalTelemetry {
  float angleDeg = 105.0f;
  float targetDeg = 105.0f;
  bool calibrating = false;
  bool trackingActive = false;
  int trackingErrorY = 0;
  int trackingConfidence = 0;
  uint32_t trackingAgeMs = 0;
  int trackingStepDeg = 0;
};

class CameraGimbalController {
 public:
  void begin();
  void update();
  void pitchDelta(int deltaDeg);
  void trackVertical(int normalizedErrorY, bool locked, int confidence);
  void resetPose();
  CameraGimbalTelemetry telemetry() const;

 private:
  float angleDeg_ = 105.0f;
  float targetDeg_ = 105.0f;
  uint32_t lastUpdateMs_ = 0;
  uint32_t lastTrackPitchMs_ = 0;
  uint32_t lastTrackSeenMs_ = 0;
  int lastTrackErrorY_ = 0;
  int lastTrackConfidence_ = 0;
  int lastTrackStepDeg_ = 0;
  bool calibrationActive_ = false;
  uint32_t calibrationStartedMs_ = 0;
};

CameraGimbalController& cameraGimbal();

#endif
