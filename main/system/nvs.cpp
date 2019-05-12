/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 2:55:18
 */

#include "nvs.h"
#include <nvs.h>
#include <nvs_flash.h>

namespace sys
{

bool
initializeNVS()
{
    static bool initialized = false;
    if (initialized)
    {
        return true;
    }

    auto ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    if (ret)
    {
        return false;
    }
    initialized = true;
    return true;
}

} // namespace sys
