#include "SerialParser.h"

#include <HTTPClient.h>
#include <WiFi.h>

#include "Diagnostics.h"
#include "PreferencesManager.h"
#include "WebController.h"
#include "camera/CameraGimbalController.h"
#include "motion/MotionCoreAdapter.h"
#include "motion/MotionCommandGateway.h"

namespace {
constexpr int CAMERA_UART_RX_PIN = 36;
constexpr int CAMERA_UART_TX_PIN = 5;
constexpr uint32_t CAMERA_UART_BAUD = 115200;
constexpr char PREF_CAMERA_URL[] = "cam_stream_url";
HardwareSerial cameraSerial(1);
struct SerialFrameBuffer {
  char data[512];
  uint8_t index;
  bool overflow;
};
SerialFrameBuffer usbFrame = {};
SerialFrameBuffer cameraFrame = {};
uint32_t cameraTxCount = 0;
uint32_t cameraStatusRxCount = 0;
uint32_t cameraDetectionRxCount = 0;
unsigned long cameraLastTxMs = 0;
unsigned long cameraLastStatusRxMs = 0;
unsigned long cameraLastDetectionRxMs = 0;
String cameraLastTxCommand;
String cameraLastStatus;
String cameraLastDetection;
String cameraLastRxSource;
uint32_t cameraHttpTxCount = 0;
int cameraLastHttpCode = 0;
String cameraLastHttpUrl;

String trimToken(const char* value) {
  String result = String(value ? value : "");
  result.trim();
  return result;
}
void sendCameraHttpCommand(const String& command, const String& diagnosticName);

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
  cameraSerial.println(command);
  Serial.println(command);
  cameraTxCount++;
  cameraLastTxMs = millis();
  cameraLastTxCommand = diagnosticName;
  sendCameraHttpCommand(command, diagnosticName);
}

void parseSerialFrame(char* frame, const char* source) {
  if (strstr(frame, "MODE:") != nullptr) {
    parseSetRobotMode(frame);
  } else if (strstr(frame, "S1:") != nullptr && strstr(frame, "S2:") != nullptr) {
    parseServoAngleCommand(frame);
  } else if (strstr(frame, "TRK:") != nullptr) {
    cameraLastRxSource = source;
    parseTrackingObservationCommand(frame);
  } else if (strstr(frame, "DX:") != nullptr && strstr(frame, "DY:") != nullptr) {
    cameraLastRxSource = source;
    parseCameraDeviationCommand(frame);
  } else if (strstr(frame, "CAMSTAT:") != nullptr) {
    cameraLastRxSource = source;
    parseCameraStatusCommand(frame);
  } else if (strstr(frame, "CAMDET:") != nullptr) {
    cameraLastRxSource = source;
    parseCameraDetectionCommand(frame);
  } else if (strchr(frame, '=') != nullptr) {
    parseSingleParam(frame);
  }
}

void pollSerialStream(Stream& stream, SerialFrameBuffer& buffer,
                      const char* source, uint16_t budget) {
  while (budget-- > 0 && stream.available()) {
    const char c = static_cast<char>(stream.read());
    if (c == '\n' || c == '\r' || c == ';') {
      if (buffer.overflow) {
        buffer.overflow = false;
        buffer.index = 0;
        continue;
      }
      buffer.data[buffer.index] = '\0';
      if (buffer.index > 0) parseSerialFrame(buffer.data, source);
      buffer.index = 0;
    } else if (!buffer.overflow && buffer.index < sizeof(buffer.data) - 1) {
      buffer.data[buffer.index++] = c;
    } else {
      buffer.overflow = true;
      buffer.index = 0;
    }
  }
}

String cameraCommandBaseUrl() {
  String url = getPrefString(PREF_CAMERA_URL, "");
  url.trim();
  if (url.isEmpty()) return "";
  if (!url.startsWith("http://")) return "";
  const int schemeEnd = url.indexOf("://");
  const int pathStart = url.indexOf('/', schemeEnd + 3);
  if (pathStart < 0) return url;
  return url.substring(0, pathStart);
}

void sendCameraHttpCommand(const String& command, const String& diagnosticName) {
  if (WiFi.status() != WL_CONNECTED) return;
  const String baseUrl = cameraCommandBaseUrl();
  if (baseUrl.isEmpty()) return;
  HTTPClient http;
  const String url = baseUrl + "/api/command?cmd=" + percentEncodeForCamera(command);
  cameraLastHttpUrl = diagnosticName;
  if (!http.begin(url)) {
    cameraLastHttpCode = -1000;
    return;
  }
  http.setTimeout(180);
  cameraLastHttpCode = http.GET();
  http.end();
  cameraHttpTxCount++;
}
}  // namespace

void initCameraSerial() {
  cameraSerial.begin(CAMERA_UART_BAUD, SERIAL_8N1,
                     CAMERA_UART_RX_PIN, CAMERA_UART_TX_PIN);
  recordDiagnosticEvent(
      "camera",
      String("uart dedicated rx=") + CAMERA_UART_RX_PIN +
          " tx=" + CAMERA_UART_TX_PIN);
}

void serialReceiveProcess() {
  pollSerialStream(cameraSerial, cameraFrame, "uart1", 160);
  pollSerialStream(Serial, usbFrame, "uart0", 64);
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
  updateWebCameraDetectionStatus("TARGET_LOCKED", 1);
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
uint32_t cameraProtocolHttpTxCount() { return cameraHttpTxCount; }
int cameraProtocolLastHttpCode() { return cameraLastHttpCode; }
String cameraProtocolLastHttpUrl() { return cameraLastHttpUrl; }
String cameraProtocolUartMode() {
  return String("dual:uart1(rx") + CAMERA_UART_RX_PIN + ",tx" +
         CAMERA_UART_TX_PIN + ")+uart0";
}
int cameraProtocolDedicatedRxPin() { return CAMERA_UART_RX_PIN; }
int cameraProtocolDedicatedTxPin() { return CAMERA_UART_TX_PIN; }
