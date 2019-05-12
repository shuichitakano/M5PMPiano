/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 1:45:10
 */
#ifndef C8C5BCA3_0133_F0D1_1622_9B66657C8E58
#define C8C5BCA3_0133_F0D1_1622_9B66657C8E58

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace io
{

struct BLECharacteristic
{
    std::string uuid;

    bool broadcast       = false;
    bool read            = false;
    bool writeNoResponse = false;
    bool write           = false;
    bool notify          = false;
    bool indicate        = false;
    bool auth            = false;

    int handle     = -1;
    int handleCCCD = -1;
};

struct BLEServiceEntry
{
    std::array<uint8_t, 6> addr;
    int addrType{};

    std::string name;
    std::string uuid;
    int flag{0};
    int rssi{0};

public:
    void dump() const;
};

void updateServiceList(std::vector<BLEServiceEntry>& list,
                       const BLEServiceEntry& s);

///
class BLEClientHandler
{
public:
    virtual bool onScanEntry(const BLEServiceEntry&) = 0;
    virtual void
    onSearchService(const std::string& primary,
                    const std::string& secondary,
                    const std::vector<BLECharacteristic>& characteristics) = 0;

    virtual void onNotify(const uint8_t* p, size_t size, int handle) {}
    virtual void onRead(const uint8_t* p, size_t size, int handle) {}
    virtual void onWriteComplete(bool result, int handle) {}
};

///
class BLEManager
{
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

public:
    BLEManager();
    ~BLEManager();

    bool initialize();
    void registerClientProfile(BLEClientHandler& p);

    void startScan();

    bool read(BLEClientHandler* h, int charHandle);
    bool write(BLEClientHandler* h,
               int charHandle,
               std::vector<uint8_t>&& data,
               bool needResponse);

    void removeAllBondedDevices();

    static BLEManager& instance();
};

//
inline bool
read(BLEClientHandler* h, int charHandle)
{
    return BLEManager::instance().read(h, charHandle);
}

inline bool
write(BLEClientHandler* h,
      int charHandle,
      std::vector<uint8_t>&& data,
      bool needResponse)
{
    return BLEManager::instance().write(
        h, charHandle, std::move(data), needResponse);
}

} // namespace io

#endif /* C8C5BCA3_0133_F0D1_1622_9B66657C8E58 */
