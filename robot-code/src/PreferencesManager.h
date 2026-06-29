#pragma once

#include <Arduino.h>
#include <Preferences.h>

#define PREF_MAC_ADDRESS_KEY "mac_address"
#define PREFERENCES_NAMESPACE "wrobot_esp32"
// Users should set their own gamepad MAC address from the Web UI.
#define DEFAULT_BLUETOOTH_MAC ""

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
