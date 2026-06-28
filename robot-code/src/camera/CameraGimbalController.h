#ifndef CAMERA_GIMBAL_CONTROLLER_H
#define CAMERA_GIMBAL_CONTROLLER_H

#include <Arduino.h>

struct CameraGimbalTelemetry {
  float angleDeg = 105.0f;
  float targetDeg = 105.0f;
  bool calibrating = false;
};

class CameraGimbalController {
 public:
  void begin();
  void update();
  void pitchDelta(int deltaDeg);
  void resetPose();
  CameraGimbalTelemetry telemetry() const;

 private:
  float angleDeg_ = 105.0f;
  float targetDeg_ = 105.0f;
  uint32_t lastUpdateMs_ = 0;
  bool calibrationActive_ = false;
  uint32_t calibrationStartedMs_ = 0;
};

CameraGimbalController& cameraGimbal();

#endif
