#include "led_dev.h"

led board_led(13);

/**
 * @brief led 灯进程函数
 */
void led_dev_proc(uint32_t tick)
{
    board_led.toggle();
}
