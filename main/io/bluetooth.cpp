/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 1:53:15
 */

#include "bluetooth.h"

#include "../debug.h"
#include <esp_bt.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <system/nvs.h>
#include <vector>

namespace io
{

bool
initializeBluetooth()
{
    static bool initialized = false;
    if (initialized)
    {
        return true;
    }

    if (!sys::initializeNVS())
    {
        return false;
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    {
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        if (auto ret = esp_bt_controller_init(&bt_cfg))
        {
            DBOUT(("%s: initialize controller failed: %s\n",
                   __func__,
                   esp_err_to_name(ret)));
            return false;
        }
    }

    if (auto ret = esp_bt_controller_enable(ESP_BT_MODE_BLE))
    {
        DBOUT(("%s: enable controller failed: %s\n",
               __func__,
               esp_err_to_name(ret)));
        return false;
    }

    if (auto ret = esp_bluedroid_init())
    {
        DBOUT((
            "%s: init bluetooth failed: %s\n", __func__, esp_err_to_name(ret)));
        return false;
    }

    if (auto ret = esp_bluedroid_enable())
    {
        DBOUT(("%s: enable bluetooth failed: %s\n",
               __func__,
               esp_err_to_name(ret)));
        return false;
    }

#if 0
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
#endif

    initialized = true;
    return true;
}

void
setBluetoothDeviceName(const char* name)
{
    esp_bt_dev_set_device_name(name);
}

void
dumpBondedClassicBTDevices()
{
    int n = esp_bt_gap_get_bond_device_num();

    std::vector<esp_bd_addr_t> list(n);
    if (auto r = esp_bt_gap_get_bond_device_list(&n, list.data()))
    {
        printf("esp_bt_gap_get_bond_device_list failed %d\n", r);
        return;
    }

    DBOUT(("%d bonded classic BT devices.\n", n));
    for (auto& a : list)
    {
        DBOUT((" %02x:%02x:%02x:%02x:%02x:%02x\n",
               a[0],
               a[1],
               a[2],
               a[3],
               a[4],
               a[5]));
    }
}

void
removeAllBondedClassicBTDevices()
{
    int n = esp_bt_gap_get_bond_device_num();

    std::vector<esp_bd_addr_t> list(n);
    esp_bt_gap_get_bond_device_list(&n, list.data());

    for (auto& a : list)
    {
        esp_bt_gap_remove_bond_device(a);
    }
}

} // namespace io
