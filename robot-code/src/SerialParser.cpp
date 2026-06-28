#include "SerialParser.h"

#include "Diagnostics.h"
#include "PreferencesManager.h"
#include "WebController.h"
#include "camera/CameraGimbalController.h"
#include "motion/MotionCoreAdapter.h"
#include "motion/MotionCommandGateway.h"

namespace {
char serialBuffer[512];
uint8_t serialIndex = 0;
bool serialFrameOverflow = false;
uint32_t cameraTxCount = 0;
uint32_t cameraStatusRxCount = 0;
uint32_t cameraDetectionRxCount = 0;
unsigned long cameraLastTxMs = 0;
unsigned long cameraLastStatusRxMs = 0;
unsigned long cameraLastDetectionRxMs = 0;
String cameraLastTxCommand;
String cameraLastStatus;
String cameraLastDetection;

String trimToken(const char* value) {
  String result = String(value ? value : "");
  result.trim();
  return result;
}

const char* trackingStateText(TrackObservationState state) {
  switch (state) {
    case TrackObservationState::Acquiring: return "acquiring";
    case TrackObservationState::Locked: return "locked";
    case TrackObservationState::Coasting: return "coasting";
    case TrackObservationState::Reacquiring: return "reacquiring";
    case TrackObservationState::Lost: return "lost";
    case TrackObservationState::Idle:
    default:
      return "idle";
  }
}

void sendCameraCommand(const String& command, const String& diagnosticName) {
  Serial.println(command);
  cameraTxCount++;
  cameraLastTxMs = millis();
  cameraLastTxCommand = diagnosticName;
}
}  // namespace

void serialReceiveProcess() {
  uint16_t budget = 96;
  while (budget-- > 0 && Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ';') {
      if (serialFrameOverflow) {
        serialFrameOverflow = false;
        serialIndex = 0;
        continue;
      }
      serialBuffer[serialIndex] = '\0';
      if (serialIndex > 0) {
        if (strstr(serialBuffer, "MODE:") != nullptr) {
          parseSetRobotMode(serialBuffer);
        } else if (strstr(serialBuffer, "S1:") != nullptr && strstr(serialBuffer, "S2:") != nullptr) {
          parseServoAngleCommand(serialBuffer);
        } else if (strstr(serialBuffer, "TRK:") != nullptr) {
          parseTrackingObservationCommand(serialBuffer);
        } else if (strstr(serialBuffer, "DX:") != nullptr && strstr(serialBuffer, "DY:") != nullptr) {
          parseCameraDeviationCommand(serialBuffer);
        } else if (strstr(serialBuffer, "CAMSTAT:") != nullptr) {
          parseCameraStatusCommand(serialBuffer);
        } else if (strstr(serialBuffer, "CAMDET:") != nullptr) {
          parseCameraDetectionCommand(serialBuffer);
        } else if (strchr(serialBuffer, '=') != nullptr) {
          parseSingleParam(serialBuffer);
        }
      }
      serialIndex = 0;
    } else if (!serialFrameOverflow && serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = c;
    } else {
      serialFrameOverflow = true;
      serialIndex = 0;
    }
  }
}

void parseSetRobotMode(char* cmd) {
  char* modeText = strstr(cmd, "MODE:");
  if (modeText == nullptr) return;
  const int mode = atoi(modeText + 5);
  if (mode == 11) {
    setMotionTrigger("serial:mode:stand");
    motionGateway().dispatch(MotionCommand::simple(MotionCommandType::Stand),
                             "serial:mode:stand");
  } else if (mode == 10) {
    setMotionTrigger("serial:mode:sit");
    motionGateway().dispatch(MotionCommand::simple(MotionCommandType::Sit),
                             "serial:mode:sit");
  } else if (mode == 12) {
    setMotionTrigger("serial:mode:reset");
    cameraGimbal().resetPose();
    motionGateway().dispatch(MotionCommand::simple(MotionCommandType::ResetPose),
                             "serial:mode:reset");
  }
}

void parseServoAngleCommand(char* cmd) {
  char* s1 = strstr(cmd, "S1:");
  char* s2 = strstr(cmd, "S2:");
  if (s1 == nullptr || s2 == nullptr) return;

  setMotionTrigger("serial:gimbal");
  cameraGimbal().pitchDelta(atoi(s2 + 3) - 105);
}

void parseCameraDeviationCommand(char* cmd) {
  char* dxPtr = strstr(cmd, "DX:");
  char* dyPtr = strstr(cmd, "DY:");
  char* dzPtr = strstr(cmd, "DZ:");
  if (dxPtr == nullptr || dyPtr == nullptr) return;

  const int dx = atoi(dxPtr + 3);
  const int dy = atoi(dyPtr + 3);
  const int dz = dzPtr != nullptr ? atoi(dzPtr + 3) : 9999;
  setMotionTrigger("camera:legacy_dxdy");
  motionGateway().dispatch(MotionCommand::trackTarget(dx, dy, dz),
                           "camera:legacy_dxdy");
}

void parseTrackingObservationCommand(char* cmd) {
  char* statePtr = strstr(cmd, "TRK:");
  if (statePtr == nullptr) return;
  statePtr += 4;

  TrackObservationState state = TrackObservationState::Idle;
  bool validState = true;
  if (strncmp(statePtr, "ACQUIRING", 9) == 0) {
    state = TrackObservationState::Acquiring;
  } else if (strncmp(statePtr, "LOCKED", 6) == 0) {
    state = TrackObservationState::Locked;
  } else if (strncmp(statePtr, "COASTING", 8) == 0) {
    state = TrackObservationState::Coasting;
  } else if (strncmp(statePtr, "REACQUIRING", 11) == 0) {
    state = TrackObservationState::Reacquiring;
  } else if (strncmp(statePtr, "LOST", 4) == 0) {
    state = TrackObservationState::Lost;
  } else if (strncmp(statePtr, "IDLE", 4) != 0) {
    validState = false;
  }
  if (!validState) return;

  char* dxPtr = strstr(cmd, "DX:");
  char* dyPtr = strstr(cmd, "DY:");
  char* dzPtr = strstr(cmd, "DZ:");
  char* exPtr = strstr(cmd, "EX:");
  char* eyPtr = strstr(cmd, "EY:");
  char* ezPtr = strstr(cmd, "EZ:");
  char* confidencePtr = strstr(cmd, "Q:");
  char* profilePtr = strstr(cmd, "P:");
  char* boxXPtr = strstr(cmd, "BX:");
  char* boxYPtr = strstr(cmd, "BY:");
  char* boxWPtr = strstr(cmd, "BW:");
  char* boxHPtr = strstr(cmd, "BH:");
  char* frameWPtr = strstr(cmd, "FW:");
  char* frameHPtr = strstr(cmd, "FH:");
  char* targetHeightPtr = strstr(cmd, "TH:");
  char* missedPtr = strstr(cmd, "MF:");
  char* stablePtr = strstr(cmd, "SF:");
  char* rawScorePtr = strstr(cmd, "RS:");
  char* velocityXPtr = strstr(cmd, "VX:");
  char* velocityYPtr = strstr(cmd, "VY:");

  const int legacyDx = dxPtr != nullptr ? atoi(dxPtr + 3) : 9999;
  const int legacyDy = dyPtr != nullptr ? atoi(dyPtr + 3) : 9999;
  const int legacyDz = dzPtr != nullptr ? atoi(dzPtr + 3) : 9999;
  const int normalizedX = exPtr != nullptr ? atoi(exPtr + 3) :
      (legacyDx == 9999 ? 9999 : constrain(legacyDx * 1000 / 320, -1000, 1000));
  const int normalizedY = eyPtr != nullptr ? atoi(eyPtr + 3) :
      (legacyDy == 9999 ? 9999 : constrain(legacyDy * 1000 / 240, -1000, 1000));
  const int normalizedZ = ezPtr != nullptr ? atoi(ezPtr + 3) :
      (legacyDz == 9999 ? 9999 : constrain(legacyDz * 6, -1000, 1000));
  const int confidence = confidencePtr != nullptr ? atoi(confidencePtr + 2) : 0;
  const int profile = profilePtr != nullptr ? atoi(profilePtr + 2) : 0;
  MotionCommand observation = MotionCommand::trackObservation(
      state, normalizedX, normalizedY, normalizedZ,
      constrain(confidence, 0, 1000), constrain(profile, 0, 3));
  observation.targetBoxX = boxXPtr != nullptr ? atoi(boxXPtr + 3) : 0;
  observation.targetBoxY = boxYPtr != nullptr ? atoi(boxYPtr + 3) : 0;
  observation.targetBoxW = boxWPtr != nullptr ? atoi(boxWPtr + 3) : 0;
  observation.targetBoxH = boxHPtr != nullptr ? atoi(boxHPtr + 3) : 0;
  observation.frameWidth = frameWPtr != nullptr ? atoi(frameWPtr + 3) : 0;
  observation.frameHeight = frameHPtr != nullptr ? atoi(frameHPtr + 3) : 0;
  observation.targetHeight = targetHeightPtr != nullptr ? atoi(targetHeightPtr + 3) : 0;
  observation.missedFrames = missedPtr != nullptr ? atoi(missedPtr + 3) : 0;
  observation.stableFrames = stablePtr != nullptr ? atoi(stablePtr + 3) : 0;
  observation.rawScore = rawScorePtr != nullptr ? atoi(rawScorePtr + 3) : confidence;
  observation.velocityX = velocityXPtr != nullptr ? atoi(velocityXPtr + 3) : 0;
  observation.velocityY = velocityYPtr != nullptr ? atoi(velocityYPtr + 3) : 0;
  cameraGimbal().trackVertical(normalizedY,
                               state == TrackObservationState::Locked,
                               observation.rawScore);
  motionGateway().dispatch(observation,
                           String("camera:track:") + trackingStateText(state));
}
void parseCameraStatusCommand(char* cmd) {
  cameraStatusRxCount++;
  cameraLastStatusRxMs = millis();
  cameraLastStatus = String(cmd);
  char* statusPtr = strstr(cmd, "CAMSTAT:");
  if (statusPtr == nullptr) return;
  statusPtr += 8;

  String state;
  String ip;
  String message;
  String firmwareVersion;
  String firmwareBuild;
  String activeResolution;
  char* token = strtok(statusPtr, ",");
  bool first = true;
  while (token != nullptr) {
    if (first) {
      state = trimToken(token);
      first = false;
    } else if (strncmp(token, "IP:", 3) == 0) {
      ip = trimToken(token + 3);
    } else if (strncmp(token, "MSG:", 4) == 0) {
      message = trimToken(token + 4);
    } else if (strncmp(token, "VER:", 4) == 0) {
      firmwareVersion = trimToken(token + 4);
    } else if (strncmp(token, "BUILD:", 6) == 0) {
      firmwareBuild = trimToken(token + 6);
    } else if (strncmp(token, "RES:", 4) == 0) {
      activeResolution = trimToken(token + 4);
    }
    token = strtok(nullptr, ",");
  }
  if (state.isEmpty()) state = "UNKNOWN";
  if (message.isEmpty()) message = state;
  updateWebCameraNetworkStatus(state, ip, message, firmwareVersion,
                               firmwareBuild, activeResolution);
}

void parseCameraDetectionCommand(char* cmd) {
  cameraDetectionRxCount++;
  cameraLastDetectionRxMs = millis();
  cameraLastDetection = String(cmd);
  char* detectPtr = strstr(cmd, "CAMDET:");
  if (detectPtr == nullptr) return;
  detectPtr += 7;

  String label;
  int count = 0;
  char* token = strtok(detectPtr, ",");
  bool first = true;
  while (token != nullptr) {
    if (first) {
      label = trimToken(token);
      first = false;
    } else if (strncmp(token, "COUNT:", 6) == 0) {
      count = atoi(token + 6);
    }
    token = strtok(nullptr, ",");
  }
  if (label.isEmpty()) label = "NO_TARGET";
  updateWebCameraDetectionStatus(label, count);
}

void parseSingleParam(char* cmd) {
  char* eq = strchr(cmd, '=');
  if (eq == nullptr) return;
  *eq = '\0';
  const char* name = cmd;
  const char* value = eq + 1;

  if (strcmp(name, "mac") == 0) {
    setPrefBluetoothMacAddress(String(value));
    restartDevice();
  } else if (strcmp(name, "show_mac") == 0) {
    Serial.println("Bluetooth MAC: " + getPrefBluetoothMacAddress());
  }
}

String percentEncodeForCamera(const String& input) {
  String encoded;
  const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < input.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(input[i]);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += static_cast<char>(c);
    } else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }
  return encoded;
}

void sendCameraWifiConfig(const String& ssid, const String& password) {
  sendCameraCommand("CAMWIFI:SSID=" + percentEncodeForCamera(ssid) +
                    ",PWD=" + percentEncodeForCamera(password) + ";",
                    "CAMWIFI");
  updateWebCameraNetworkStatus("PENDING", "", "Camera WiFi config sent");
}

void sendCameraRuntimeConfig(const String& resolution) {
  sendCameraCommand("CAMCFG:RES=" + percentEncodeForCamera(resolution) + ";",
                    "CAMCFG");
  updateWebCameraNetworkStatus("PENDING", "", "Camera config sent");
}

void sendCameraTrackSelection(int x, int y, int width, int height,
                              int profile) {
  sendCameraCommand("TRACKROI:x=" + String(x) + ",y=" + String(y) +
                    ",w=" + String(width) + ",h=" + String(height) +
                    ",p=" + String(constrain(profile, 0, 3)) + ";",
                    "TRACKROI");
  updateWebCameraDetectionStatus("TARGET_SELECTED", 1);
}

void sendCameraTrackDistanceAdjust(int value) {
  sendCameraCommand("TRACKDIST:v=" + String(constrain(value, -100, 100)) + ";",
                    "TRACKDIST");
}

void sendCameraTrackScan() {
  sendCameraCommand("TRACKSCAN;", "TRACKSCAN");
  updateWebCameraDetectionStatus("TARGET_SCAN", 0);
}

void sendCameraTrackStop() {
  sendCameraCommand("TRACKSTOP;", "TRACKSTOP");
  updateWebCameraDetectionStatus("NO_TARGET", 0);
}

void sendCameraPing() {
  sendCameraCommand("CAMPING;", "CAMPING");
}

uint32_t cameraProtocolTxCount() { return cameraTxCount; }
uint32_t cameraProtocolStatusRxCount() { return cameraStatusRxCount; }
uint32_t cameraProtocolDetectionRxCount() { return cameraDetectionRxCount; }
unsigned long cameraProtocolLastTxMs() { return cameraLastTxMs; }
unsigned long cameraProtocolLastStatusRxMs() { return cameraLastStatusRxMs; }
unsigned long cameraProtocolLastDetectionRxMs() { return cameraLastDetectionRxMs; }
String cameraProtocolLastTxCommand() { return cameraLastTxCommand; }
String cameraProtocolLastStatus() { return cameraLastStatus; }
String cameraProtocolLastDetection() { return cameraLastDetection; }
