#include "PreferencesManager.h"

#include <freertos/semphr.h>

Preferences prefManager;
namespace {
SemaphoreHandle_t preferencesMutex = nullptr;

void lockPreferences() {
  if (preferencesMutex != nullptr) xSemaphoreTake(preferencesMutex, portMAX_DELAY);
}

void unlockPreferences() {
  if (preferencesMutex != nullptr) xSemaphoreGive(preferencesMutex);
}
}  // namespace

void initPreferencesManager() {
  if (preferencesMutex == nullptr) preferencesMutex = xSemaphoreCreateMutex();
  if (!prefManager.begin(PREFERENCES_NAMESPACE, false)) {
    Serial.println("[WROBOT] Preferences initialization failed");
  }
}

String getPrefBluetoothMacAddress() {
  lockPreferences();
  const String value = prefManager.getString(PREF_MAC_ADDRESS_KEY, DEFAULT_BLUETOOTH_MAC);
  unlockPreferences();
  return value;
}

void setPrefBluetoothMacAddress(const String& macAddress) {
  lockPreferences();
  prefManager.putString(PREF_MAC_ADDRESS_KEY, macAddress);
  unlockPreferences();
}

String getPrefString(const char* key, const String& defaultValue) {
  lockPreferences();
  const String value = prefManager.getString(key, defaultValue);
  unlockPreferences();
  return value;
}

void setPrefString(const char* key, const String& value) {
  lockPreferences();
  prefManager.putString(key, value);
  unlockPreferences();
}

float getPrefFloat(const char* key, float defaultValue) {
  lockPreferences();
  const float value = prefManager.getFloat(key, defaultValue);
  unlockPreferences();
  return value;
}

void setPrefFloat(const char* key, float value) {
  lockPreferences();
  prefManager.putFloat(key, value);
  unlockPreferences();
}

int getPrefInt(const char* key, int defaultValue) {
  lockPreferences();
  const int value = prefManager.getInt(key, defaultValue);
  unlockPreferences();
  return value;
}

void setPrefInt(const char* key, int value) {
  lockPreferences();
  prefManager.putInt(key, value);
  unlockPreferences();
}

bool getPrefBool(const char* key, bool defaultValue) {
  lockPreferences();
  const bool value = prefManager.getBool(key, defaultValue);
  unlockPreferences();
  return value;
}

void setPrefBool(const char* key, bool value) {
  lockPreferences();
  prefManager.putBool(key, value);
  unlockPreferences();
}

void restartDevice() {
  Serial.println("[WROBOT] Restarting in 100 ms");
  delay(100);
  ESP.restart();
}
