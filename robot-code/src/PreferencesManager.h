#pragma once

#include <Arduino.h>
#include <Preferences.h>

#define PREF_MAC_ADDRESS_KEY "mac_address"
#define PREFERENCES_NAMESPACE "wrobot_esp32"
// MAC used by the controller in the original known-good WRobot firmware.
#define DEFAULT_BLUETOOTH_MAC "ac:00:03:28:ad:60"

extern Preferences prefManager;

void initPreferencesManager();
String getPrefBluetoothMacAddress();
void setPrefBluetoothMacAddress(const String& macAddress);
String getPrefString(const char* key, const String& defaultValue);
void setPrefString(const char* key, const String& value);
float getPrefFloat(const char* key, float defaultValue);
void setPrefFloat(const char* key, float value);
int getPrefInt(const char* key, int defaultValue);
void setPrefInt(const char* key, int value);
bool getPrefBool(const char* key, bool defaultValue);
void setPrefBool(const char* key, bool value);
void restartDevice();
