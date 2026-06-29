#ifndef GAMEPAD_BLUETOOTH_H
#define GAMEPAD_BLUETOOTH_H

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include "PreferencesManager.h"
#include "Diagnostics.h"
#include "RGBController.h"

using GamepadControllerNotificationParser = XboxControllerNotificationParser;

enum VibrateState {
  VIBRATE_OFF,
  VIBRATE_STARTED,
  VIBRATE_TRIGGER
};

inline XboxSeriesXControllerESP32_asukiaaa::Core& getGamepadController() {
  static String macAddress = getPrefBluetoothMacAddress();
  macAddress.trim();
  macAddress.toLowerCase();
  if (macAddress.length() != 17) {
    macAddress = DEFAULT_BLUETOOTH_MAC;
  }
  static XboxSeriesXControllerESP32_asukiaaa::Core instance(macAddress.c_str());
  return instance;
}

#define gamepadController getGamepadController()

static XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
static bool wasConnected = false;
static VibrateState vibrateState = VIBRATE_OFF;
static unsigned long vibrateStartMs = 0;
static unsigned long long gamepad_vibrate_duration = 0;

extern void processControllerData(const GamepadControllerNotificationParser& data);
extern void stopControllerMotion();

inline void gamepadControllerInit() {
  getGamepadController().begin();
  vibrateState = VIBRATE_OFF;
}

inline void updateGamepadVibration() {
  const unsigned long currentMs = millis();
  if (vibrateState == VIBRATE_STARTED || vibrateState == VIBRATE_TRIGGER) {
    if (currentMs - vibrateStartMs >= gamepad_vibrate_duration) {
      repo.v.power.center = 0;
      repo.v.power.shake = 0;
      gamepadController.writeHIDReport(repo);
      vibrateState = VIBRATE_OFF;
    }
  }
}

inline void GamepadKeyVibration(uint8_t power = 20,
                                unsigned long duration = 200) {
  power = constrain(power, 0, 100);
  duration = constrain(duration, 50, 2000);
  gamepad_vibrate_duration = duration;

  if (vibrateState != VIBRATE_OFF) {
    repo.v.power.center = 0;
    repo.v.power.shake = 0;
    gamepadController.writeHIDReport(repo);
  }

  repo.setAllOff();
  repo.v.select.center = true;
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.select.shake = true;
  repo.v.power.center = power;
  repo.v.power.shake = power;
  repo.v.timeActive = 10;
  gamepadController.writeHIDReport(repo);

  vibrateState = VIBRATE_STARTED;
  vibrateStartMs = millis();
}

inline void GamepadTrigRtVibration(unsigned long duration = 300) {
  static uint16_t trigMax = GamepadControllerNotificationParser::maxTrig;
  const uint8_t power =
      (uint8_t)((float)gamepadController.xboxNotif.trigRT / trigMax * 100) *
      0.5;
  gamepad_vibrate_duration = duration;

  if (power == 0) {
    repo.v.power.center = 0;
    repo.v.power.shake = 0;
    gamepadController.writeHIDReport(repo);
    vibrateState = VIBRATE_OFF;
    return;
  }

  repo.setAllOff();
  repo.v.select.center = true;
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.select.shake = true;
  repo.v.power.center = power;
  repo.v.power.shake = power;
  repo.v.timeActive = 10;
  gamepadController.writeHIDReport(repo);

  vibrateState = VIBRATE_TRIGGER;
  vibrateStartMs = millis();
}

inline void process_gamepad_notif() {
  gamepadController.onLoop();
  if (gamepadController.isConnected() &&
      !gamepadController.isWaitingForFirstNotification()) {
    if (!wasConnected) {
      GamepadKeyVibration(100, 1000);
      startLEDBlink(CRGB::Green, 50, 3);
      wasConnected = true;
      recordDiagnosticEvent("gamepad", "connected");
    }
    processControllerData(gamepadController.xboxNotif);
  } else {
    if (wasConnected) {
      GamepadKeyVibration(80);
      stopControllerMotion();
      wasConnected = false;
      recordDiagnosticEvent("gamepad", "disconnected");
    }
  }
}

#endif  // GAMEPAD_BLUETOOTH_H
