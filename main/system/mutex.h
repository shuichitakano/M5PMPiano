/*
 * author : Shuichi TAKANO
 * since  : Tue Jan 01 2019 13:40:5
 */
#ifndef _00FB69F2_2134_1396_CC81_73001F59FA3E
#define _00FB69F2_2134_1396_CC81_73001F59FA3E

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace sys
{

class Mutex
{
    SemaphoreHandle_t handle_;

public:
    Mutex() { handle_ = xSemaphoreCreateRecursiveMutex(); }
    ~Mutex() { vSemaphoreDelete(handle_); }

    void lock() { xSemaphoreTakeRecursive(handle_, portMAX_DELAY); }
    void unlock() { xSemaphoreGiveRecursive(handle_); }

private:
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;
};

} // namespace sys

#endif /* _00FB69F2_2134_1396_CC81_73001F59FA3E */
