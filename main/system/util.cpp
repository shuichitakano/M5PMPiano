/*
 * author : Shuichi TAKANO
 * since  : Sun Jan 06 2019 21:3:42
 */

#include "util.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>

namespace sys
{

void
yield()
{
    vPortYield();
}

uint32_t IRAM_ATTR
micros()
{
    return (uint32_t)esp_timer_get_time();
}

uint32_t IRAM_ATTR
millis()
{
    return micros() / 1000;
}

void
delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void IRAM_ATTR
delayMicroseconds(uint32_t us)
{
    if (us)
    {
        auto t0 = micros();
        while (micros() - t0 < us)
        {
            asm volatile("nop");
        }
    }
}

} // namespace sys
