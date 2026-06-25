#ifndef VOLTAGE_MONITOR_H
#define VOLTAGE_MONITOR_H

#include <Arduino.h>
#include <esp_adc_cal.h>

// 电压检测相关宏定义（原代码中电压模块的宏）
#define LED_BAT 13                  // 电量显示LED引脚
#define BAT_PIN 35                  // 电池电压检测ADC引脚
static const adc1_channel_t channel = ADC1_CHANNEL_7;  // ADC通道
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;// ADC采样宽度
static const adc_atten_t atten = ADC_ATTEN_DB_12;      // ADC衰减系数
static const adc_unit_t unit = ADC_UNIT_1;             // ADC单元

struct BatteryStatus {
  float voltage;
  int percent;
  bool low;
  bool valid;
};

// 电压检测相关全局变量（原代码中电压模块的全局变量）
extern uint16_t bat_check_num;                          // 电压检测计数
extern esp_adc_cal_characteristics_t adc_chars;        // ADC校准参数

// 电压检测相关函数声明（原代码中电压模块的函数）
void adc_calibration_init();  // 电压检测初始化
void bat_check();             // 电压检测逻辑
void forceBatteryMeasurement();
BatteryStatus getBatteryStatus();
float getBatteryVoltage();
int getBatteryPercentage();
bool isBatteryLow();
void setBatteryCalibration(float emptyVoltage, float fullVoltage, float lowVoltage);
float getBatteryEmptyVoltage();
float getBatteryFullVoltage();
float getBatteryLowVoltage();

#endif // VOLTAGE_MONITOR_H
