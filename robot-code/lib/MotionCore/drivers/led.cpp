#include "led.h"

/**
 * @brief led 构造函数
 * 
 * @param pin led 引脚号
 */
led::led(uint8_t pin)
    : pin(pin)
{
    pinMode(pin, OUTPUT);
}

/**
 * @brief 打开 led
 */
void led::on()
{
    digitalWrite(pin, HIGH);
}

/**
 * @brief 关闭 led
 */
void led::off()
{
    digitalWrite(pin, LOW);
}

/**
 * @brief 切换 led 状态
 */
void led::toggle()
{
    digitalWrite(pin, !digitalRead(pin));
}
