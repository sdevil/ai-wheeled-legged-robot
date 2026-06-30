#ifndef WEB_CONTROL_STATE_H
#define WEB_CONTROL_STATE_H

#include <Arduino.h>

struct WebControlDiagnosticsSnapshot {
  int driveX;
  int driveY;
  bool driveActive;
  unsigned long driveUpdatedAtMs;
  uint32_t driveCommandCount;
  uint32_t driveCommandMaxGapMs;

  int pendingLegLeanPercent;
  char pendingLegLeanSource[32];
  int lastLegLeanPercent;
  char lastLegLeanSource[32];
  unsigned long lastLegLeanCommandAtMs;
  uint32_t legLeanCommandCount;
  uint32_t legLeanCommandMaxGapMs;
  uint32_t websocketCloseCount;
};

void webControlStateInit();

bool webControlSetDriveCommand(bool controlsLocked, int x, int y);
bool webControlSetGimbalYawCommand(bool controlsLocked, int x);
void webControlQueueCameraPitchDelta(int delta);
bool webControlSetGuardServoAngleCommand(bool controlsLocked, int angle);
bool webControlSetLegHeightCommand(bool controlsLocked, int direction);
bool webControlSetLegHeightPercentCommand(bool controlsLocked, int percent);
bool webControlSetLegLeanCommand(bool controlsLocked, int percent,
                                 const char* source);
void webControlExpireLegHeightCommands();
void webControlStopRealtimeCommands();
void webControlNoteWebsocketClosed();

bool webControlGetDriveCommand(int& joyX, int& joyY,
                               unsigned long timeoutMs);
bool webControlGetGimbalYawCommand(int& joyX, unsigned long timeoutMs);
int webControlConsumeCameraPitchDelta();
bool webControlConsumeGuardServoAngle(int& angle);
bool webControlConsumeLegHeightDirection(int& direction);
bool webControlConsumeLegHeightPercent(int& percent);
bool webControlConsumeLegLeanPercent(int& percent);
void webControlGetDiagnosticsSnapshot(WebControlDiagnosticsSnapshot& snapshot);

#endif
