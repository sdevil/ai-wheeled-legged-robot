#ifndef ROBOT_CONTROL_COORDINATOR_H
#define ROBOT_CONTROL_COORDINATOR_H

#include <Arduino.h>

#if ENABLE_GAMEPAD_BLE
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
using GamepadControllerNotificationParser = XboxControllerNotificationParser;
#endif

void syncControlModeIndicatorState(bool gamepadEnabled, bool force = false);
void processWebControlState(bool gamepadEnabled);
void publishControlStatus(bool gamepadConnected);
bool isTrackModeActive();

#if ENABLE_GAMEPAD_BLE
void processControllerData(const GamepadControllerNotificationParser& data);
void stopControllerMotion();
#endif

#endif
