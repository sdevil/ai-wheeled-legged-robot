#include "VoltageMonitor.h"
#include "RGBController.h"

#include <math.h>

namespace {
BatteryStatus g_battery_status = {0.0f, 0, false, false};
float g_battery_empty_voltage = 6.6f;
float g_battery_full_voltage = 8.35f;
float g_battery_low_voltage = 7.8f;
uint8_t g_low_battery_samples = 0;

int batteryPercentFromVoltage(float voltage) {
  struct CurvePoint {
    float voltage;
    int percent;
  };

  static const CurvePoint curve[] = {
      {8.35f, 100}, {8.20f, 95}, {8.05f, 85}, {7.90f, 75}, {7.75f, 62},
      {7.60f, 50},  {7.45f, 38}, {7.30f, 24}, {7.10f, 10}, {6.90f, 3},
      {6.60f, 0},
  };

  if (voltage >= g_battery_full_voltage) return 100;
  if (voltage <= g_battery_empty_voltage) return 0;
  const float calibratedRange = g_battery_full_voltage - g_battery_empty_voltage;
  if (calibratedRange <= 0.1f) return 0;
  const float curveVoltage = 6.60f +
      (voltage - g_battery_empty_voltage) * (8.35f - 6.60f) / calibratedRange;
  const size_t pointCount = sizeof(curve) / sizeof(curve[0]);
  for (size_t i = 0; i + 1 < pointCount; ++i) {
    const CurvePoint& high = curve[i];
    const CurvePoint& low = curve[i + 1];
    if (curveVoltage <= high.voltage && curveVoltage >= low.voltage) {
      const float range = high.voltage - low.voltage;
      if (range <= 0.0f) return high.percent;
      const float ratio = (curveVoltage - low.voltage) / range;
      return constrain((int)(low.percent + ratio * (high.percent - low.percent) + 0.5f), 0, 100);
    }
  }
  return 0;
}

void updateBatteryStatusFromVoltage(float batteryVoltage) {
  g_battery_status.voltage = batteryVoltage;
  g_battery_status.percent = batteryPercentFromVoltage(batteryVoltage);
  if (g_battery_status.percent < 10) {
    if (g_low_battery_samples < 3) g_low_battery_samples++;
  } else {
    g_low_battery_samples = 0;
  }
  if (!g_battery_status.low && g_low_battery_samples >= 3) {
    g_battery_status.low = true;
  } else if (g_battery_status.low && g_battery_status.percent >= 12) {
    g_battery_status.low = false;
  }
  g_battery_status.valid = true;
}
}  // namespace

// 初始化电压检测相关全局变量（与头文件声明对应）
uint16_t bat_check_num = 0;
esp_adc_cal_characteristics_t adc_chars;

/**
 * @brief 电压检测初始化（原代码逻辑完全保留）
 */
void adc_calibration_init()
{
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)  {
    printf("eFuse Two Point: Supported\n");
  }  else  {
    printf("eFuse Two Point: NOT supported\n");
  }
  // Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)  {
    printf("eFuse Vref: Supported\n");
  }  else  {
    printf("eFuse Vref: NOT supported\n");
  }
}

/**
 * @brief 电压检测逻辑（原代码逻辑完全保留）
 */
void bat_check()
{
  if (bat_check_num > 1000)
  {
    forceBatteryMeasurement();

    bat_check_num = 0;
  }
  else
    bat_check_num++;
}

void forceBatteryMeasurement() {
  uint32_t rawSum = 0;
  constexpr int sampleCount = 8;
  for (int i = 0; i < sampleCount; ++i) {
    rawSum += analogRead(BAT_PIN);
  }
  const uint32_t rawAverage = rawSum / sampleCount;
  const uint32_t millivolts = esp_adc_cal_raw_to_voltage(rawAverage, &adc_chars);
  const float batteryVoltage = (millivolts * 3.97f) / 1000.0f;
  updateBatteryStatusFromVoltage(batteryVoltage);
}

BatteryStatus getBatteryStatus() {
  return g_battery_status;
}

float getBatteryVoltage() {
  return g_battery_status.voltage;
}

int getBatteryPercentage() {
  return g_battery_status.percent;
}

bool isBatteryLow() {
  return g_battery_status.low;
}

void setBatteryCalibration(float emptyVoltage, float fullVoltage, float lowVoltage) {
  if (!isfinite(emptyVoltage) || !isfinite(fullVoltage) || !isfinite(lowVoltage) ||
      emptyVoltage < 5.0f || fullVoltage > 9.0f ||
      fullVoltage - emptyVoltage < 0.5f) {
    return;
  }
  g_battery_empty_voltage = emptyVoltage;
  g_battery_full_voltage = fullVoltage;
  g_battery_low_voltage = constrain(lowVoltage, emptyVoltage, fullVoltage);
  if (g_battery_status.valid) {
    updateBatteryStatusFromVoltage(g_battery_status.voltage);
  }
}

float getBatteryEmptyVoltage() {
  return g_battery_empty_voltage;
}

float getBatteryFullVoltage() {
  return g_battery_full_voltage;
}

float getBatteryLowVoltage() {
  return g_battery_low_voltage;
}
