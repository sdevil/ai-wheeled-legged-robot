#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>

void serialReceiveProcess();
void initCameraSerial();
void parseSetRobotMode(char* cmd);
void parseServoAngleCommand(char* cmd);
void parseCameraDeviationCommand(char* cmd);
void parseTrackingObservationCommand(char* cmd);
void parseCameraStatusCommand(char* cmd);
void parseCameraDetectionCommand(char* cmd);
void parseSingleParam(char* cmd);

void sendCameraWifiConfig(const String& ssid, const String& password);
void sendCameraRuntimeConfig(const String& resolution);
void sendCameraTrackSelection(int x, int y, int width, int height,
                              int profile = 0);
void sendCameraTrackDistanceAdjust(int value);
void sendCameraTrackScan();
void sendCameraTrackStop();
void sendCameraPing();
uint32_t cameraProtocolTxCount();
uint32_t cameraProtocolStatusRxCount();
uint32_t cameraProtocolDetectionRxCount();
unsigned long cameraProtocolLastTxMs();
unsigned long cameraProtocolLastStatusRxMs();
unsigned long cameraProtocolLastDetectionRxMs();
String cameraProtocolLastTxCommand();
String cameraProtocolLastStatus();
String cameraProtocolLastDetection();
uint32_t cameraProtocolHttpTxCount();
int cameraProtocolLastHttpCode();
String cameraProtocolLastHttpUrl();
String cameraProtocolUartMode();
int cameraProtocolDedicatedRxPin();
int cameraProtocolDedicatedTxPin();
String percentEncodeForCamera(const String& input);

#endif
