#include "control/WebControlState.h"

#include <string.h>

namespace {
portMUX_TYPE webControlMux = portMUX_INITIALIZER_UNLOCKED;

int driveX = 0;
int driveY = 0;
bool driveActive = false;
int gimbalYawX = 0;
bool gimbalYawActive = false;
unsigned long lastGimbalYawMs = 0;
int pendingCameraPitchDelta = 0;
int pendingGuardServoAngle = -1;
int pendingLegHeightDirection = 99;
int pendingLegHeightPercent = -1;
int pendingLegLeanPercent = 999;
char pendingLegLeanSource[32] = "boot";
int lastLegLeanPercent = 0;
char lastLegLeanSource[32] = "boot";
unsigned long lastDriveCommandMs = 0;
uint32_t driveCommandCount = 0;
uint32_t driveCommandMaxGapMs = 0;
uint32_t websocketCloseCount = 0;
unsigned long lastLegHeightCommandMs = 0;
unsigned long lastLegLeanCommandMs = 0;
uint32_t legLeanCommandCount = 0;
uint32_t legLeanCommandMaxGapMs = 0;
}  // namespace

void webControlStateInit() {}

bool webControlSetDriveCommand(bool controlsLocked, int x, int y) {
  if (controlsLocked) return false;
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  const unsigned long now = millis();
  portENTER_CRITICAL(&webControlMux);
  if (lastDriveCommandMs != 0) {
    const uint32_t gapMs = now - lastDriveCommandMs;
    if (gapMs > driveCommandMaxGapMs) driveCommandMaxGapMs = gapMs;
  }
  ++driveCommandCount;
  driveX = x;
  driveY = y;
  driveActive = !(x == 0 && y == 0);
  lastDriveCommandMs = now;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlSetGimbalYawCommand(bool controlsLocked, int x) {
  if (controlsLocked) return false;
  x = constrain(x, -100, 100);
  portENTER_CRITICAL(&webControlMux);
  gimbalYawX = x;
  gimbalYawActive = x != 0;
  lastGimbalYawMs = millis();
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

void webControlQueueCameraPitchDelta(int delta) {
  portENTER_CRITICAL(&webControlMux);
  pendingCameraPitchDelta = constrain(pendingCameraPitchDelta + delta, -18, 18);
  portEXIT_CRITICAL(&webControlMux);
}

bool webControlSetGuardServoAngleCommand(bool controlsLocked, int angle) {
  if (controlsLocked) return false;
  angle = constrain(angle, 0, 180);
  portENTER_CRITICAL(&webControlMux);
  pendingGuardServoAngle = angle;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlSetLegHeightCommand(bool controlsLocked, int direction) {
  if (controlsLocked) return false;
  direction = constrain(direction, -1, 1);
  portENTER_CRITICAL(&webControlMux);
  pendingLegHeightPercent = -1;
  pendingLegHeightDirection = direction;
  lastLegHeightCommandMs = direction == 0 ? 0 : millis();
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlSetLegHeightPercentCommand(bool controlsLocked, int percent) {
  if (controlsLocked) return false;
  percent = constrain(percent, 0, 100);
  portENTER_CRITICAL(&webControlMux);
  pendingLegHeightPercent = percent;
  pendingLegHeightDirection = 99;
  lastLegHeightCommandMs = 0;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlSetLegLeanCommand(bool controlsLocked, int percent,
                                 const char* source) {
  if (controlsLocked) return false;
  percent = constrain(percent, -100, 100);
  const unsigned long nowMs = millis();
  portENTER_CRITICAL(&webControlMux);
  if (lastLegLeanCommandMs != 0) {
    const uint32_t gap = nowMs - lastLegLeanCommandMs;
    if (gap > legLeanCommandMaxGapMs) legLeanCommandMaxGapMs = gap;
  }
  ++legLeanCommandCount;
  pendingLegLeanPercent = percent;
  strncpy(pendingLegLeanSource, source == nullptr ? "unknown" : source,
          sizeof(pendingLegLeanSource) - 1);
  pendingLegLeanSource[sizeof(pendingLegLeanSource) - 1] = '\0';
  lastLegLeanPercent = percent;
  strncpy(lastLegLeanSource, pendingLegLeanSource,
          sizeof(lastLegLeanSource) - 1);
  lastLegLeanSource[sizeof(lastLegLeanSource) - 1] = '\0';
  lastLegLeanCommandMs = nowMs;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

void webControlExpireLegHeightCommands() {
  constexpr unsigned long timeoutMs = 350;
  const unsigned long now = millis();
  portENTER_CRITICAL(&webControlMux);
  if (lastLegHeightCommandMs != 0 &&
      now - lastLegHeightCommandMs > timeoutMs) {
    pendingLegHeightDirection = 0;
    lastLegHeightCommandMs = 0;
  }
  portEXIT_CRITICAL(&webControlMux);
}

void webControlStopRealtimeCommands() {
  portENTER_CRITICAL(&webControlMux);
  driveX = 0;
  driveY = 0;
  driveActive = false;
  gimbalYawX = 0;
  gimbalYawActive = false;
  lastGimbalYawMs = 0;
  lastDriveCommandMs = 0;
  pendingCameraPitchDelta = 0;
  pendingLegHeightDirection = 0;
  pendingLegHeightPercent = -1;
  lastLegHeightCommandMs = 0;
  portEXIT_CRITICAL(&webControlMux);
}

void webControlNoteWebsocketClosed() {
  portENTER_CRITICAL(&webControlMux);
  ++websocketCloseCount;
  portEXIT_CRITICAL(&webControlMux);
}

bool webControlGetDriveCommand(int& joyX, int& joyY,
                               unsigned long timeoutMs) {
  int x;
  int y;
  bool active;
  unsigned long updatedAt;
  portENTER_CRITICAL(&webControlMux);
  x = driveX;
  y = driveY;
  active = driveActive;
  updatedAt = lastDriveCommandMs;
  portEXIT_CRITICAL(&webControlMux);

  if (!active || updatedAt == 0 || millis() - updatedAt > timeoutMs) return false;
  joyX = x;
  joyY = y;
  return true;
}

bool webControlGetGimbalYawCommand(int& joyX, unsigned long timeoutMs) {
  int x;
  unsigned long updatedAt;
  bool active;
  portENTER_CRITICAL(&webControlMux);
  x = gimbalYawX;
  active = gimbalYawActive;
  updatedAt = lastGimbalYawMs;
  portEXIT_CRITICAL(&webControlMux);

  if (!active || updatedAt == 0 || millis() - updatedAt > timeoutMs) return false;
  joyX = x;
  return true;
}

int webControlConsumeCameraPitchDelta() {
  portENTER_CRITICAL(&webControlMux);
  const int delta = pendingCameraPitchDelta;
  pendingCameraPitchDelta = 0;
  portEXIT_CRITICAL(&webControlMux);
  return delta;
}

bool webControlConsumeGuardServoAngle(int& angle) {
  portENTER_CRITICAL(&webControlMux);
  if (pendingGuardServoAngle < 0) {
    portEXIT_CRITICAL(&webControlMux);
    return false;
  }
  angle = pendingGuardServoAngle;
  pendingGuardServoAngle = -1;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlConsumeLegHeightDirection(int& direction) {
  portENTER_CRITICAL(&webControlMux);
  if (pendingLegHeightDirection == 99) {
    portEXIT_CRITICAL(&webControlMux);
    return false;
  }
  direction = pendingLegHeightDirection;
  pendingLegHeightDirection = 99;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlConsumeLegHeightPercent(int& percent) {
  portENTER_CRITICAL(&webControlMux);
  if (pendingLegHeightPercent < 0) {
    portEXIT_CRITICAL(&webControlMux);
    return false;
  }
  percent = pendingLegHeightPercent;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

bool webControlConsumeLegLeanPercent(int& percent) {
  portENTER_CRITICAL(&webControlMux);
  if (pendingLegLeanPercent == 999) {
    portEXIT_CRITICAL(&webControlMux);
    return false;
  }
  percent = pendingLegLeanPercent;
  pendingLegLeanPercent = 999;
  portEXIT_CRITICAL(&webControlMux);
  return true;
}

void webControlGetDiagnosticsSnapshot(WebControlDiagnosticsSnapshot& snapshot) {
  portENTER_CRITICAL(&webControlMux);
  snapshot.driveX = driveX;
  snapshot.driveY = driveY;
  snapshot.driveActive = driveActive;
  snapshot.driveUpdatedAtMs = lastDriveCommandMs;
  snapshot.driveCommandCount = driveCommandCount;
  snapshot.driveCommandMaxGapMs = driveCommandMaxGapMs;
  snapshot.pendingLegLeanPercent = pendingLegLeanPercent;
  strncpy(snapshot.pendingLegLeanSource, pendingLegLeanSource,
          sizeof(snapshot.pendingLegLeanSource) - 1);
  snapshot.pendingLegLeanSource[sizeof(snapshot.pendingLegLeanSource) - 1] = '\0';
  snapshot.lastLegLeanPercent = lastLegLeanPercent;
  strncpy(snapshot.lastLegLeanSource, lastLegLeanSource,
          sizeof(snapshot.lastLegLeanSource) - 1);
  snapshot.lastLegLeanSource[sizeof(snapshot.lastLegLeanSource) - 1] = '\0';
  snapshot.lastLegLeanCommandAtMs = lastLegLeanCommandMs;
  snapshot.legLeanCommandCount = legLeanCommandCount;
  snapshot.legLeanCommandMaxGapMs = legLeanCommandMaxGapMs;
  snapshot.websocketCloseCount = websocketCloseCount;
  portEXIT_CRITICAL(&webControlMux);
}
