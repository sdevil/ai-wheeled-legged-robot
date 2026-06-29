#ifndef WEB_CONTROLLER_H
#define WEB_CONTROLLER_H

#include <Arduino.h>

enum class WebRobotAction : uint8_t {
  None,
  Stand,
  Sit,
  ResetPose,
  CancelKick,
  TrackMode,
  LedTest,
  Jump,
  JumpForward,
  JumpBackward,
  JumpLeft,
  JumpRight,
  EnterMaintenance,
  ExitMaintenance
};

void initWebController();
bool getWebDriveCommand(int &joyX, int &joyY);
bool getWebGimbalYawCommand(int &joyX);
int consumeWebCameraPitchDelta();
bool consumeWebGuardServoAngle(int& angle);
bool consumeWebLegHeightDirection(int& direction);
bool consumeWebLegHeightPercent(int& percent);
bool consumeWebLegLeanPercent(int& percent);
WebRobotAction consumeWebRobotAction();
bool isWebOtaInProgress();
void updateWebControllerStatus(bool robotEnabled, bool sittingDown,
                               bool gamepadConnected, float balanceAngle,
                               float batteryVoltage, int batteryPercent,
                               bool maintenanceMode,
                               int legHeightPercent, int legLeanPercent);
void updateWebCameraNetworkStatus(const String& state, const String& ip,
                                  const String& message,
                                  const String& firmwareVersion = "",
                                  const String& firmwareBuild = "",
                                  const String& activeResolution = "");
void updateWebCameraDetectionStatus(const String& label, int count);
bool isWebCameraScanPending();

#endif

