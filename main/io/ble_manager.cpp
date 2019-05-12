/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 2:59:9
 */

#include "ble_manager.h"
#include "../debug.h"
#include <algorithm>
#include <deque>
#include <esp_bt.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gattc_api.h>
#include <memory>
#include <mutex>
#include <string.h>
#include <system/mutex.h>
#include <vector>

namespace io
{

namespace
{

sys::Mutex mutex_;

bool
less(const BLEServiceEntry& a, const BLEServiceEntry& b)
{
    if (a.name != b.name)
    {
        return a.name < b.name;
    }
    return a.addr < b.addr;
}

bool
eq(const BLEServiceEntry& a, const BLEServiceEntry& b)
{
    return a.addr == b.addr && a.name == b.name;
}

std::string
makeUUID16String(uint16_t v)
{
    char str[5];
    sprintf(str, "%04x", v);
    return str;
}

std::string
makeUUID32String(uint32_t v)
{
    char str[9];
    sprintf(str, "%08x", v);
    return str;
}

std::string
makeUUID128String(const uint8_t* uuid128)
{
    char str[37];
    sprintf(str,
            "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%"
            "02x%02x%02x%02x%02x",
            uuid128[15],
            uuid128[14],
            uuid128[13],
            uuid128[12],
            uuid128[11],
            uuid128[10],
            uuid128[9],
            uuid128[8],
            uuid128[7],
            uuid128[6],
            uuid128[5],
            uuid128[4],
            uuid128[3],
            uuid128[2],
            uuid128[1],
            uuid128[0]);
    return str;
}

std::string
makeUUIDString(const esp_bt_uuid_t& v)
{
    switch (v.len)
    {
    case ESP_UUID_LEN_16:
        return makeUUID16String(v.uuid.uuid16);

    case ESP_UUID_LEN_32:
        return makeUUID32String(v.uuid.uuid32);

    case ESP_UUID_LEN_128:
        return makeUUID128String(v.uuid.uuid128);
    }
    return {};
}

void
dumpBondedDevices()
{
    int dev_num = esp_ble_get_bond_device_num();
    std::vector<esp_ble_bond_dev_t> list(dev_num);

    esp_ble_get_bond_device_list(&dev_num, list.data());
    DBOUT(("%d bonded BLE devices.\n", dev_num));
    for (auto& v : list)
    {
        DBOUT((" device: %02x:%02x:%02x:%02x:%02x:%02x\n",
               v.bd_addr[0],
               v.bd_addr[1],
               v.bd_addr[2],
               v.bd_addr[3],
               v.bd_addr[4],
               v.bd_addr[5]));
    }
}

bool
isBondedDevice(esp_bd_addr_t bda)
{
    int dev_num = esp_ble_get_bond_device_num();
    std::vector<esp_ble_bond_dev_t> list(dev_num);

    esp_ble_get_bond_device_list(&dev_num, list.data());
    for (auto& v : list)
    {
        if (memcmp(v.bd_addr, bda, 6) == 0)
        {
            return true;
        }
    }
    return false;
}

} // namespace

void
BLEServiceEntry::dump() const
{
    DBOUT(("ADDR: %02x:%02x:%02x:%02x:%02x:%02x, type %d RSSI: %d\n",
           addr[0],
           addr[1],
           addr[2],
           addr[3],
           addr[4],
           addr[5],
           addrType,
           rssi));
    DBOUT(("'%s':'%s' flag %d\n", name.c_str(), uuid.c_str(), flag));
}

void
updateServiceList(std::vector<BLEServiceEntry>& list, const BLEServiceEntry& s)
{
    list.push_back(s);
    std::sort(list.begin(), list.end(), less);
    list.erase(std::unique(list.begin(), list.end(), eq), list.end());
}

struct BLEManager::Impl
{
    struct Service
    {
        std::pair<int, int> handle_;
        esp_bt_uuid_t uuid_;
        std::string uuidStr_;
        std::string uuidSecondaryStr_;

        std::vector<BLECharacteristic> characteristics_;

        BLECharacteristic* getCurrentChar()
        {
            return characteristics_.empty() ? nullptr
                                            : &characteristics_.back();
        }
    };

    struct ClientState
    {
        int appID_            = -1;
        esp_gatt_if_t gattIF_ = ESP_GATT_IF_NONE;

        enum class Stage
        {
            IDLE,
            OPENED,
            CONNECTED,
        };
        Stage stage_ = Stage::IDLE;

        esp_bd_addr_t bda_{};
        int connectID_ = -1;

        std::vector<Service> services_;

        BLEClientHandler* handler_;

        struct Job
        {
            virtual void process(ClientState* state) = 0;
            virtual esp_gatt_status_t
            handleEvent(ClientState* state,
                        esp_gattc_cb_event_t event,
                        esp_ble_gattc_cb_param_t* param) = 0;
        };

        struct JobReadCh : public Job
        {
            int handle_;

            JobReadCh(int handle)
                : handle_{handle}
            {
            }

            void process(ClientState* state) override
            {
                esp_ble_gattc_read_char(state->gattIF_,
                                        state->connectID_,
                                        handle_,
                                        ESP_GATT_AUTH_REQ_NONE);
            }

            esp_gatt_status_t
            handleEvent(ClientState* state,
                        esp_gattc_cb_event_t event,
                        esp_ble_gattc_cb_param_t* param) override
            {
                assert(event == ESP_GATTC_WRITE_CHAR_EVT);
                auto& p = param->read;
                if (p.status == ESP_GATT_OK)
                {
                    DBOUT(("read char success: h %d\n", p.handle));
                    for (int i = 0; i < p.value_len; ++i)
                    {
                        DBOUT(("%02x ", p.value[i]));
                    }
                    DBOUT(("\n"));
                }
                else
                {
                    DBOUT(("read char failed: h %d, status %02x\n",
                           p.handle,
                           p.status));
                }
                if (p.status != ESP_GATT_INSUF_AUTHENTICATION)
                {
                    state->handler_->onRead(p.status == ESP_GATT_OK ? p.value
                                                                    : nullptr,
                                            p.value_len,
                                            p.handle);
                }
                return p.status;
            }
        };

        struct JobWriteCh : public Job
        {
            int handle_;
            std::vector<uint8_t> data_;
            bool needResponse_;

            JobWriteCh(int handle,
                       std::vector<uint8_t>&& data,
                       bool needResponse = true)
                : handle_{handle}
                , data_{data}
                , needResponse_{needResponse}
            {
            }

            void process(ClientState* state) override
            {
                esp_ble_gattc_write_char(state->gattIF_,
                                         state->connectID_,
                                         handle_,
                                         data_.size(),
                                         data_.data(),
                                         needResponse_
                                             ? ESP_GATT_WRITE_TYPE_RSP
                                             : ESP_GATT_WRITE_TYPE_NO_RSP,
                                         ESP_GATT_AUTH_REQ_NONE);
            }

            esp_gatt_status_t
            handleEvent(ClientState* state,
                        esp_gattc_cb_event_t event,
                        esp_ble_gattc_cb_param_t* param) override
            {
                assert(event == ESP_GATTC_WRITE_CHAR_EVT);
                auto& p = param->write;
                if (p.status == ESP_GATT_OK)
                {
                    DBOUT(("write char success: h %d\n", p.handle));
                }
                else
                {
                    DBOUT(("write char failed: h %d, status %02x\n",
                           p.handle,
                           p.status));
                }
                if (p.status != ESP_GATT_INSUF_AUTHENTICATION)
                {
                    state->handler_->onWriteComplete(p.status == ESP_GATT_OK,
                                                     p.handle);
                }
                return p.status;
            }
        };

        struct JobWriteChDesc : public Job
        {
            int handle_;
            size_t dataSize_;
            const void* data_;
            bool needResponse_;

            JobWriteChDesc(int handle,
                           const void* data,
                           size_t size,
                           bool needResponse = true)
                : handle_{handle}
                , dataSize_{size}
                , data_{data}
                , needResponse_{needResponse}
            {
                DBOUT(("create write ch desc job %d\n", handle));
            }

            void process(ClientState* state) override
            {
                DBOUT(("proc write desc ch %d\n", handle_));
                esp_ble_gattc_write_char_descr(
                    state->gattIF_,
                    state->connectID_,
                    handle_,
                    dataSize_,
                    reinterpret_cast<uint8_t*>(const_cast<void*>(data_)),
                    needResponse_ ? ESP_GATT_WRITE_TYPE_RSP
                                  : ESP_GATT_WRITE_TYPE_NO_RSP,
                    ESP_GATT_AUTH_REQ_NONE);
            }

            esp_gatt_status_t
            handleEvent(ClientState* state,
                        esp_gattc_cb_event_t event,
                        esp_ble_gattc_cb_param_t* param) override
            {
                DBOUT(("handle ev: write desc %d\n", handle_));
                assert(event == ESP_GATTC_WRITE_DESCR_EVT);
                auto& p = param->write;
                if (p.status == ESP_GATT_OK)
                {
                    DBOUT(("write description success: h %d\n", p.handle));
                }
                else
                {
                    DBOUT(("write description failed: h %d, status %02x\n",
                           p.handle,
                           p.status));
                }
                return p.status;
            }
        };

        struct JobRegisterNotify : public Job
        {
            int handle_;

            JobRegisterNotify(int handle)
                : handle_{handle}
            {
                DBOUT(("create reg notify job %d\n", handle));
            }

            void process(ClientState* state) override
            {
                DBOUT(("proc reg notify %d\n", handle_));
                esp_ble_gattc_register_for_notify(
                    state->gattIF_, state->bda_, handle_);
            }

            esp_gatt_status_t
            handleEvent(ClientState* state,
                        esp_gattc_cb_event_t event,
                        esp_ble_gattc_cb_param_t* param) override
            {
                assert(event == ESP_GATTC_REG_FOR_NOTIFY_EVT);
                auto& p = param->reg_for_notify;
                if (p.status == ESP_GATT_OK)
                {
                    DBOUT(("reg for notify success: h %d\n", p.handle));
                }
                else
                {
                    DBOUT(("reg for notify failed: h %d, status %02x\n",
                           p.handle,
                           p.status));
                }
                return p.status;
            }
        };

        std::deque<std::unique_ptr<Job>> jobs_;

    public:
        bool isOpened() const { return stage_ >= Stage::OPENED; }
        bool isConnected() const { return stage_ >= Stage::CONNECTED; }

        void setOpened() { stage_ = Stage::OPENED; }
        void setConnected() { stage_ = Stage::CONNECTED; }

        void setAddress(esp_bd_addr_t addr)
        {
            memcpy(bda_, addr, sizeof(esp_bd_addr_t));
        }

        bool checkAddress(esp_bd_addr_t addr)
        {
            return memcmp(bda_, addr, 6) == 0;
        }

        void disconnect()
        {
            stage_     = Stage::IDLE;
            connectID_ = -1;

            services_.clear();
        }

        void addJob(Job* job)
        {
            std::lock_guard<sys::Mutex> lock(mutex_);
            bool kick = jobs_.empty();
            jobs_.emplace_back(job);

            if (kick)
            {
                DBOUT(("kick first job %p\n", job));
                job->process(this);
            }
        }

        Job* getCurrentJob()
        {
            return jobs_.empty() ? nullptr : jobs_.front().get();
        }

        void callJob(bool next = true)
        {
            if (jobs_.empty())
            {
                return;
            }
            if (next)
            {
                std::lock_guard<sys::Mutex> lock(mutex_);
                jobs_.pop_front();
            }

            auto job = getCurrentJob();
            if (job)
            {
                job->process(this);
            }
        }

        void startAuthentication()
        {
            esp_ble_set_encryption(bda_, ESP_BLE_SEC_ENCRYPT);

            if (isBondedDevice(bda_))
            {
                callJob(false); // retry
            }
        }
    };

    std::deque<ClientState> clients_;

    esp_ble_scan_params_t scanParams_ = {
        .scan_type          = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval      = 0x50,
        .scan_window        = 0x30,
        .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE};

    int scanDurationInSec_ = 30;
    bool scanning_         = false;

    int currentAppID_ = 0;

public:
    static Impl& getImpl() { return *BLEManager::instance().pimpl_; }

    ClientState* findClientByAppID(int id)
    {
        for (auto& p : clients_)
        {
            if (p.appID_ == id)
            {
                return &p;
            }
        }
        return nullptr;
    }

    ClientState* findClientByGATTIF(int gattIF)
    {
        for (auto& p : clients_)
        {
            if (p.gattIF_ == gattIF)
            {
                return &p;
            }
        }
        return nullptr;
    }

    ClientState* findClientByAddress(esp_bd_addr_t bda)
    {
        for (auto& p : clients_)
        {
            if (p.checkAddress(bda))
            {
                return &p;
            }
        }
        return nullptr;
    }

    ClientState* findClientByHandler(BLEClientHandler* h)
    {
        for (auto& p : clients_)
        {
            if (p.handler_ == h)
            {
                return &p;
            }
        }
        return nullptr;
    }

    void onGAPEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
    {
        DBOUT(("onGAPEvent: event %d, param %p\n", event, param));

        switch (event)
        {
        case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        {
            auto& p = param->local_privacy_cmpl;
            if (p.status != ESP_BT_STATUS_SUCCESS)
            {
                DBOUT(("config local privacy failed, error code =%x\n",
                       p.status));
                break;
            }
            DBOUT(("set local privacy complete.\n"));
            //            startScan();
        }
        break;

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            DBOUT(("ESP_GAP_BLE_PASSKEY_REQ_EVT\n"));
            break;

        case ESP_GAP_BLE_OOB_REQ_EVT:
            DBOUT(("ESP_GAP_BLE_OOB_REQ_EVT\n"));
            break;

        case ESP_GAP_BLE_LOCAL_IR_EVT:
            DBOUT(("ESP_GAP_BLE_LOCAL_IR_EVT\n"));
            break;

        case ESP_GAP_BLE_LOCAL_ER_EVT:
            DBOUT(("ESP_GAP_BLE_LOCAL_ER_EVT\n"));
            break;

        case ESP_GAP_BLE_SEC_REQ_EVT:
            DBOUT(("ESP_GAP_BLE_SEC_REQ_EVT\n"));
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;

        case ESP_GAP_BLE_NC_REQ_EVT:
            DBOUT(("ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d\n",
                   param->ble_security.key_notif.passkey));
            break;

        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        {
            auto& p = param->ble_security.key_notif;
            DBOUT(("Passkey notify number:%d\n", p.passkey));
        }
        break;

        case ESP_GAP_BLE_KEY_EVT:
            DBOUT(("key type = %s\n", [](auto t) {
                switch (t)
                {
                case ESP_LE_KEY_NONE:
                    return "ESP_LE_KEY_NONE";
                case ESP_LE_KEY_PENC:
                    return "ESP_LE_KEY_PENC";
                case ESP_LE_KEY_PID:
                    return "ESP_LE_KEY_PID";
                case ESP_LE_KEY_PCSRK:
                    return "ESP_LE_KEY_PCSRK";
                case ESP_LE_KEY_PLK:
                    return "ESP_LE_KEY_PLK";
                case ESP_LE_KEY_LLK:
                    return "ESP_LE_KEY_LLK";
                case ESP_LE_KEY_LENC:
                    return "ESP_LE_KEY_LENC";
                case ESP_LE_KEY_LID:
                    return "ESP_LE_KEY_LID";
                case ESP_LE_KEY_LCSRK:
                    return "ESP_LE_KEY_LCSRK";
                default:
                    return "INVALID BLE KEY TYPE";
                }
            }(param->ble_security.ble_key.key_type)));
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
        {
            auto& p = param->ble_security.auth_cmpl;
            DBOUT(("ESP_GAP_BLE_AUTH_CMPL_EVT %02x:%02x:%02x:%02x:%02x:%02x\n",
                   p.bd_addr[0],
                   p.bd_addr[1],
                   p.bd_addr[2],
                   p.bd_addr[3],
                   p.bd_addr[4],
                   p.bd_addr[5]));
            DBOUT((" pair status = %s\n", p.success ? "success" : "fail"));
            if (!p.success)
            {
                DBOUT((" fail reason = 0x%x\n", p.fail_reason));
                // hcidefs.h にエラーコード
                // bta_api.h に HCI_ERR_MAX_ERR 以降の値がある
                // 0x63 : BTA_DM_AUTH_SMP_RSP_TIMEOUT
            }
            else
            {
                DBOUT((" auth mode = %s\n", [](auto t) {
                    switch (t)
                    {
                    case ESP_LE_AUTH_NO_BOND:
                        return "ESP_LE_AUTH_NO_BOND";
                    case ESP_LE_AUTH_BOND:
                        return "ESP_LE_AUTH_BOND";
                    case ESP_LE_AUTH_REQ_MITM:
                        return "ESP_LE_AUTH_REQ_MITM";
                    case ESP_LE_AUTH_REQ_SC_ONLY:
                        return "ESP_LE_AUTH_REQ_SC_ONLY";
                    case ESP_LE_AUTH_REQ_SC_BOND:
                        return "ESP_LE_AUTH_REQ_SC_BOND";
                    case ESP_LE_AUTH_REQ_SC_MITM:
                        return "ESP_LE_AUTH_REQ_SC_MITM";
                    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
                        return "ESP_LE_AUTH_REQ_SC_MITM_BOND";
                    default:
                        return "INVALID BLE AUTH REQ";
                    }
                }(p.auth_mode)));

                if (auto client = findClientByAddress(p.bd_addr))
                {
                    client->callJob(false); // retry job before authentication
                    // if (auto r = esp_ble_gattc_send_mtu_req(client->gattIF_,
                    //                                         client->connectID_))
                    // {
                    //     DBOUT(("config MTU error, error code = %x\n", r));
                    // }
                }
            }
        }
        break;

        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            scanning_ = true;
            esp_ble_gap_start_scanning(scanDurationInSec_);
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                DBOUT(("scan start failed, error status = %x\n",
                       param->scan_start_cmpl.status));
                scanning_ = false;
                break;
            }
            DBOUT(("scan start success\n"));
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
            auto& res = param->scan_rst;
            switch (res.search_evt)
            {
            case ESP_GAP_SEARCH_INQ_RES_EVT:
            {
                BLEServiceEntry e;
                e.addr[0]  = res.bda[0];
                e.addr[1]  = res.bda[1];
                e.addr[2]  = res.bda[2];
                e.addr[3]  = res.bda[3];
                e.addr[4]  = res.bda[4];
                e.addr[5]  = res.bda[5];
                e.rssi     = res.rssi;
                e.flag     = res.flag;
                e.addrType = res.ble_addr_type;

                uint8_t len;
                auto adv_name = esp_ble_resolve_adv_data(
                    res.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &len);
                std::string name((const char*)adv_name,
                                 (const char*)adv_name + len);
                e.name = name;

                auto uuid128 = esp_ble_resolve_adv_data(
                    res.ble_adv, ESP_BLE_AD_TYPE_128SRV_CMPL, &len);
                if (uuid128)
                {
                    e.uuid = makeUUID128String(uuid128);
                }

                if (0)
                {
                    auto p   = res.ble_adv;
                    auto end = p + 62;
                    while (p[0] && p < end)
                    {
                        auto l    = p[0];
                        auto type = p[1];
                        DBOUT(("type %02x: ", type));
                        auto ct = l - 1;
                        auto d  = p + 2;
                        while (ct)
                        {
                            DBOUT(("%02x ", *d));
                            ++d;
                            --ct;
                        }
                        DBOUT((": size %d\n", l));
                        p += l + 1;
                    }
                }

                e.dump();

                for (auto& p : clients_)
                {
                    if (!p.isOpened() && p.handler_->onScanEntry(e))
                    {
                        p.setOpened();
                        DBOUT(("connect to the remote device: %s, if %d\n",
                               e.name.c_str(),
                               p.gattIF_));
                        DBOUT(("addr %02x:%02x:%02x:%02x:%02x:%02x %d\n",
                               e.addr[0],
                               e.addr[1],
                               e.addr[2],
                               e.addr[3],
                               e.addr[4],
                               e.addr[5],
                               e.addrType));

                        esp_ble_gap_stop_scanning();

                        esp_ble_gattc_open(p.gattIF_,
                                           e.addr.data(),
                                           (esp_ble_addr_type_t)e.addrType,
                                           true);
                    }
                }
            }
            break;

            case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                break;

            default:
                break;
            }
            break;
        }

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                DBOUT(("scan stop failed, error status = %x\n",
                       param->scan_stop_cmpl.status));
                break;
            }
            DBOUT(("stop scan successfully\n"));
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                DBOUT(("adv stop failed, error status = %x\n",
                       param->adv_stop_cmpl.status));
                break;
            }
            DBOUT(("stop adv successfully\n"));
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            DBOUT(("update connection params status = %d, min_int = %d, "
                   "max_int = %d,conn_int = %d,latency = %d, timeout = %d\n",
                   param->update_conn_params.status,
                   param->update_conn_params.min_int,
                   param->update_conn_params.max_int,
                   param->update_conn_params.conn_int,
                   param->update_conn_params.latency,
                   param->update_conn_params.timeout));
            break;

        default:
            break;
        }
    }

    void onGATTClientEvent(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t* param)
    {
        DBOUT(("onGATTClientEvent: event %d, if %d, param %p\n",
               event,
               gattc_if,
               param));

        if (event == ESP_GATTC_REG_EVT)
        {
            if (param->reg.status == ESP_GATT_OK)
            {
                DBOUT(("GATTC REG: appID %d, if %d\n",
                       param->reg.app_id,
                       gattc_if));
                if (auto c = findClientByAppID(param->reg.app_id))
                {
                    c->gattIF_ = gattc_if;
                    esp_ble_gap_config_local_privacy(true);
                }
                else
                {
                    DBOUT(("can't find ClientState appID %d\n",
                           param->reg.app_id));
                }
            }
            else
            {
                DBOUT(("reg app failed, app_id %04x, status %d\n",
                       param->reg.app_id,
                       param->reg.status));
                return;
            }

            return;
        }

        auto client = findClientByGATTIF(gattc_if);

        switch (event)
        {
        case ESP_GATTC_CONNECT_EVT:
        {
            auto& p = param->connect;
            DBOUT(("connect: %02x:%02x:%02x:%02x:%02x:%02x: connID %d\n",
                   p.remote_bda[0],
                   p.remote_bda[1],
                   p.remote_bda[2],
                   p.remote_bda[3],
                   p.remote_bda[4],
                   p.remote_bda[5],
                   p.conn_id));
            client->setAddress(p.remote_bda);
            client->connectID_ = p.conn_id;

            if (auto r = esp_ble_gattc_send_mtu_req(gattc_if, p.conn_id))
            {
                DBOUT(("config MTU error, error code = %x\n", r));
            }
        }
        break;

        case ESP_GATTC_OPEN_EVT:
        {
            auto& p = param->open;
            if (p.status != ESP_GATT_OK)
            {
                DBOUT(("open failed, status %d\n", p.status));
                client->disconnect();
                break;
            }
            DBOUT(("open success\n"));
        }
        break;

        case ESP_GATTC_DISCONNECT_EVT:
            DBOUT(("disconnect\n"));
            client->disconnect();
            break;

        case ESP_GATTC_CFG_MTU_EVT:
        {
            auto& p = param->cfg_mtu;
            if (p.status != ESP_GATT_OK)
            {
                DBOUT(("config mtu failed, error status = %x\n", p.status));
            }
            DBOUT(("ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, connID %d\n",
                   p.status,
                   p.mtu,
                   p.conn_id));

            client->services_.clear();
            esp_ble_gattc_search_service(gattc_if, p.conn_id, nullptr);
        }
        break;

        case ESP_GATTC_SEARCH_RES_EVT:
        {
            auto& p = param->search_res;

            Service s;
            s.handle_  = {p.start_handle, p.end_handle};
            s.uuid_    = p.srvc_id.uuid;
            s.uuidStr_ = makeUUIDString(p.srvc_id.uuid);

            DBOUT(("ESP_GATTC_SEARCH_RES_EVT: start %d, end %d, inst %d, uuid "
                   "%d, %s\n",
                   p.start_handle,
                   p.end_handle,
                   p.srvc_id.inst_id,
                   p.srvc_id.uuid.len,
                   s.uuidStr_.c_str()));

            client->services_.push_back(s);
        }
        break;

        case ESP_GATTC_SEARCH_CMPL_EVT:
        {
            auto& p = param->search_cmpl;
            if (p.status != ESP_GATT_OK)
            {
                DBOUT(("search service failed, error status = %x\n", p.status));
                break;
            }
            if (p.searched_service_source ==
                ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            {
                DBOUT(("Get service information from remote device\n"));
            }
            else if (p.searched_service_source ==
                     ESP_GATT_SERVICE_FROM_NVS_FLASH)
            {
                DBOUT(("Get service information from flash\n"));
            }
            else
            {
                DBOUT(("unknown service source\n"));
            }
            DBOUT(("ESP_GATTC_SEARCH_CMPL_EVT\n"));

            for (auto& s : client->services_)
            {
                uint16_t count = 0;
                if (auto status = esp_ble_gattc_get_attr_count(gattc_if,
                                                               p.conn_id,
                                                               ESP_GATT_DB_ALL,
                                                               s.handle_.first,
                                                               s.handle_.second,
                                                               0,
                                                               &count))
                {
                    DBOUT(("esp_ble_gattc_get_attr_count error %d\n", status));
                }
                DBOUT(("%d db entries.\n", count));

                if (count)
                {
                    std::vector<esp_gattc_db_elem_t> db(count);

                    if (auto r = esp_ble_gattc_get_db(gattc_if,
                                                      p.conn_id,
                                                      s.handle_.first,
                                                      s.handle_.second,
                                                      db.data(),
                                                      &count))
                    {
                        DBOUT(("get db falied %d\n", r));
                        continue;
                    }
                    DBOUT(("%d result db entries.\n", count));
                    assert(count <= db.size());
                    db.resize(count);

                    for (auto& e : db)
                    {
                        switch (e.type)
                        {
                        case ESP_GATT_DB_PRIMARY_SERVICE:
                        {
                            auto uuid = makeUUIDString(e.uuid);
                            DBOUT(("Pimary service: %s\n", uuid.c_str()));
                            assert(s.uuidStr_ == uuid);
                            assert(s.handle_.first == e.start_handle);
                            assert(s.handle_.second == e.end_handle);
                            s.uuidStr_ = std::move(uuid);
                        }
                        break;

                        case ESP_GATT_DB_SECONDARY_SERVICE:
                        {
                            auto uuid = makeUUIDString(e.uuid);
                            DBOUT(("Secondary service: %s, %d:%d\n",
                                   uuid.c_str(),
                                   e.start_handle,
                                   e.end_handle));
                            s.uuidSecondaryStr_ = std::move(uuid);
                        }
                        break;

                        case ESP_GATT_DB_CHARACTERISTIC:
                        {
                            BLECharacteristic ch;
                            ch.handle = e.attribute_handle;
                            ch.uuid   = makeUUIDString(e.uuid);
                            ch.broadcast =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_BROADCAST;
                            ch.read =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_READ;
                            ch.writeNoResponse =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
                            ch.write =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_WRITE;
                            ch.notify =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY;
                            ch.indicate =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_INDICATE;
                            ch.auth =
                                e.properties & ESP_GATT_CHAR_PROP_BIT_AUTH;

                            DBOUT(("Characteristic: %s, h %d, %02x ( ",
                                   ch.uuid.c_str(),
                                   ch.handle,
                                   e.properties));
                            if (ch.broadcast)
                            {
                                DBOUT(("broadcast "));
                            }
                            if (ch.read)
                            {
                                DBOUT(("read "));
                            }
                            if (ch.writeNoResponse)
                            {
                                DBOUT(("writeNR "));
                            }
                            if (ch.write)
                            {
                                DBOUT(("write "));
                            }
                            if (ch.notify)
                            {
                                DBOUT(("notify "));
                            }
                            if (ch.indicate)
                            {
                                DBOUT(("indicate "));
                            }
                            if (ch.auth)
                            {
                                DBOUT(("auth "));
                            }
                            DBOUT((")\n"));

                            s.characteristics_.push_back(std::move(ch));

                            if (ch.notify)
                            {
                                client->addJob(
                                    new ClientState::JobRegisterNotify(
                                        ch.handle));
                            }
                        }
                        break;

                        case ESP_GATT_DB_DESCRIPTOR:
                        {
                            auto ch   = s.getCurrentChar();
                            auto uuid = makeUUIDString(e.uuid);
                            DBOUT(("Descriptor: %s h %d, chh %d\n",
                                   uuid.c_str(),
                                   e.attribute_handle,
                                   ch ? ch->handle : -1));

                            if (ch)
                            {
                                if (e.uuid.len == ESP_UUID_LEN_16 &&
                                    e.uuid.uuid.uuid16 ==
                                        ESP_GATT_UUID_CHAR_CLIENT_CONFIG &&
                                    ch->notify)
                                {
                                    ch->handleCCCD = e.attribute_handle;
                                    DBOUT(("CCCD. h%d\n", e.attribute_handle));

                                    static uint8_t dataNotify[]   = {1, 0};
                                    static uint8_t dataIndicate[] = {2, 0};
                                    client->addJob(
                                        new ClientState::JobWriteChDesc(
                                            ch->handleCCCD,
                                            ch->notify ? dataNotify
                                                       : dataIndicate,
                                            2));
                                }
                            }
                        }
                        break;

                        case ESP_GATT_DB_INCLUDED_SERVICE:
                        {
                            auto uuid = makeUUIDString(e.uuid);
                            DBOUT(("Included Service: %s\n", uuid.c_str()));
                        }
                        break;

                        case ESP_GATT_DB_ALL:
                            DBOUT(("DBAll\n"));
                            break;

                        default:
                            break;
                        }
                    }
                }
                else
                {
                    DBOUT(("no char found\n"));
                }
                client->handler_->onSearchService(
                    s.uuidStr_, s.uuidSecondaryStr_, s.characteristics_);
            }
        }
        break;

        case ESP_GATTC_NOTIFY_EVT:
        {
            auto& p = param->notify;
            if (p.is_notify)
            {
                DBOUT(("notify: %d bytes\n", p.value_len));
            }
            else
            {
                DBOUT(("indicate: %d bytes\n", p.value_len));
            }
            for (int i = 0; i < p.value_len; ++i)
            {
                DBOUT(("%02x ", p.value[i]));
            }
            DBOUT(("\n"));

            client->handler_->onNotify(p.value, p.value_len, p.handle);
        }
        break;

        case ESP_GATTC_WRITE_DESCR_EVT:
        case ESP_GATTC_READ_CHAR_EVT:
        case ESP_GATTC_WRITE_CHAR_EVT:
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            if (auto job = client->getCurrentJob())
            {
                auto st = job->handleEvent(client, event, param);
                if (st == ESP_GATT_OK)
                {
                    DBOUT(("proc success. try next job.\n"));
                    client->callJob(); // next
                }
                else if (st == ESP_GATT_INSUF_AUTHENTICATION)
                {
                    DBOUT(("insuf auth. try authentication.\n"));
                    client->startAuthentication(); // auth & retry
                }
            }
            break;

        case ESP_GATTC_SRVC_CHG_EVT:
        {
            DBOUT(("service changed.\n"));
        }
        break;

        default:
            break;
        }
    }

    void registerClientHandler(BLEClientHandler& p)
    {
        DBOUT(("Register BLEClient %p:%d\n", &p, currentAppID_));

        ClientState e;
        e.appID_   = currentAppID_++;
        e.handler_ = &p;
        clients_.push_back(std::move(e));

        // 先に ClientState を登録しておかないといけない
        if (auto ret = esp_ble_gattc_app_register(e.appID_))
        {
            DBOUT(("%s gattc app register failed, error code = %x\n",
                   __func__,
                   ret));
        }
    }

    bool initialize()
    {
        DBOUT(("initialize BLEManager.\n"));
        if (auto ret = esp_ble_gap_register_callback(
                [](esp_gap_ble_cb_event_t event,
                   esp_ble_gap_cb_param_t* param) {
                    getImpl().onGAPEvent(event, param);
                }))
        {
            DBOUT(("%s gap register failed, error code = %x\n", __func__, ret));
            return false;
        }

        if (auto ret = esp_ble_gattc_register_callback(
                [](esp_gattc_cb_event_t event,
                   esp_gatt_if_t gattc_if,
                   esp_ble_gattc_cb_param_t* param) {
                    getImpl().onGATTClientEvent(event, gattc_if, param);
                }))
        {
            DBOUT(
                ("%s gattc register failed, error code = %x\n", __func__, ret));
            return false;
        }
#if 1
        if (auto local_mtu_ret = esp_ble_gatt_set_local_mtu(500))
        {
            DBOUT(("set local  MTU failed, error code = %x", local_mtu_ret));
        }
#endif
        // uint8_t auth_req = ESP_LE_AUTH_BOND;
        // uint8_t auth_req = ESP_LE_AUTH_REQ_SC_ONLY;
        uint8_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
        // uint8_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
        uint8_t iocap = ESP_IO_CAP_NONE;
        // uint8_t iocap = ESP_IO_CAP_IO;
        // uint8_t iocap    = ESP_IO_CAP_OUT;
        uint8_t key_size = 16;
        uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
        uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

        esp_ble_gap_set_security_param(
            ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
        return true;
    }

    void startScan()
    {
        dumpBondedDevices();

        DBOUT(("Start BLE Scannig.\n"));
        if (scanning_)
        {
            return;
        }

        if (auto r = esp_ble_gap_set_scan_params(&scanParams_))
        {
            DBOUT(("set scan params error, error code = %x\n", r));
            return;
        }

        DBOUT(("start BLE scanning done.\n"));
    }

    bool read(BLEClientHandler* h, int charHandle)
    {
        auto client = findClientByHandler(h);
        if (!client)
        {
            DBOUT(("invalid client handler: %p\n", client));
            return false;
        }

        client->addJob(new ClientState::JobReadCh(charHandle));
        return true;
    }

    bool write(BLEClientHandler* h,
               int charHandle,
               std::vector<uint8_t>&& data,
               bool needResponse)
    {
        auto client = findClientByHandler(h);
        if (!client)
        {
            DBOUT(("invalid client handler: %p\n", client));
            return false;
        }

        client->addJob(new ClientState::JobWriteCh(
            charHandle, std::move(data), needResponse));
        return true;
    }
};

BLEManager::BLEManager()
    : pimpl_(std::make_unique<Impl>())
{
}

BLEManager::~BLEManager() = default;

bool
BLEManager::initialize()
{
    return pimpl_->initialize();
}

void
BLEManager::registerClientProfile(BLEClientHandler& p)
{
    pimpl_->registerClientHandler(p);
}

void
BLEManager::startScan()
{
    pimpl_->startScan();
}

bool
BLEManager::read(BLEClientHandler* h, int charHandle)
{
    return pimpl_->read(h, charHandle);
}

bool
BLEManager::write(BLEClientHandler* h,
                  int charHandle,
                  std::vector<uint8_t>&& data,
                  bool needResponse)
{
    return pimpl_->write(h, charHandle, std::move(data), needResponse);
}

void
BLEManager::removeAllBondedDevices()
{
    int dev_num = esp_ble_get_bond_device_num();
    std::vector<esp_ble_bond_dev_t> list(dev_num);

    esp_ble_get_bond_device_list(&dev_num, list.data());
    for (auto& v : list)
    {
        esp_ble_remove_bond_device(v.bd_addr);
    }
    DBOUT(("remove %d bonded devices.\n", dev_num));
}

BLEManager&
BLEManager::instance()
{
    static BLEManager inst;
    return inst;
}

} // namespace io
