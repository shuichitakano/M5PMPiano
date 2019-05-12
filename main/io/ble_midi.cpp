/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 5:33:5
 */

#include "ble_midi.h"
#include "../debug.h"

#define ENABLE_DEBUG_PRINT 1

#if ENABLE_DEBUG_PRINT
#define DB DBOUT
#else
#define DB(x)
#endif

namespace io
{
namespace
{
constexpr const char* serviceUUID_ = "03b80e5a-ede8-4b33-a751-6ce34ec4c700";
constexpr const char* charUUID_    = "7772e5db-3868-4112-a1a9-f2669d106bf3";

} // namespace

BLEMidiClient&
BLEMidiClient::instance()
{
    static BLEMidiClient inst;
    return inst;
}

bool
BLEMidiClient::onScanEntry(const BLEServiceEntry& e)
{
    if (e.uuid != serviceUUID_)
    {
        return false;
    }

    updateServiceList(list_, e);
    DB(("%zd BLE Midi Devices.\n", list_.size()));

    //    return false;
    return true; // 見つけ次第問答無用で接続
}

void
BLEMidiClient::onSearchService(
    const std::string& primary,
    const std::string& secondary,
    const std::vector<BLECharacteristic>& characteristics)
{
    if (primary != serviceUUID_)
    {
        return;
    }

    for (auto& ch : characteristics)
    {
        if (ch.uuid == charUUID_)
        {
            DB(("MIDI char handle = %d\n", ch.handle));
            handle_ = ch.handle;

            readable_         = ch.read;
            writable_         = ch.write || ch.writeNoResponse;
            writeNRSupported_ = ch.writeNoResponse;
        }
    }
}

void
BLEMidiClient::onNotify(const uint8_t* p, size_t size, int handle)
{
    if (handle == handle_ && midiIn_)
    {
        DB(("Midi in: handle %d, %zd bytes.\n", handle, size));

        if (size < 3)
        {
            DB((" too short message.\n"));
            return;
        }

        auto process = [this](int timeH,
                              int timeL,
                              const uint8_t* top,
                              const uint8_t* bottom) {
            DB(("time %d\n", timeL | (timeH << 7)));
            midiInMessageMaker_.analyze(
                top, bottom, [this](const MidiMessage& m) {
                    midiIn_->put(m);
                    m.dump();
                });
        };

        auto tail = p + size;

        if ((*p & 0x80) == 0)
        {
            DB(("invalid timestamp (H) %02x\n", *p));
            return;
        }
        auto timeH = *p & 0x3f;
        ++p;

        while (p < tail)
        {
            if (p + 2 > tail)
            {
                DB(("invalid message length.\n"));
                return;
            }

            if ((p[0] & 0x80) == 0)
            {
                DB(("invalid timestamp (L) %02x\n", p[0]));
                return;
            }
            auto timeL      = p[0] & 0x7f;
            auto messageTop = p + 1;
            p += 2;

            while (p < tail)
            {
                if (*p & 0x80)
                {
                    break;
                }
                ++p;
            }
            process(timeH, timeL, messageTop, p);
        }
    }
}

void
BLEMidiClient::onRead(const uint8_t* p, size_t size, int handle)
{
}

void
BLEMidiClient::onWriteComplete(bool result, int handle)
{
    if (handle == handle_)
    {
        DB(("Midi write complete: %d\n", handle));
    }
}

void
BLEMidiClient::put(const MidiMessage& m)
{
    // MidiOut
    if (handle_ < 0 || !writable_ || !m.isValid())
    {
        return;
    }

    // DB(("out:"));
    // m.dump();

    // 本当はqueueしてまとめた方がいい
    int time = 0;
    std::vector<uint8_t> data;
    if (m.isEndOfSysEx() && m.size > 1)
    {
        data.reserve(2 + m.size - 1 + 1 + 1);
        data.push_back(0x80 + ((time >> 7) & 0x3f));
        data.push_back(0x80 + (time & 0x7f));
        data.insert(data.end(), m.data.data(), m.data.data() + m.size - 1);
        data.push_back(0x80 + (time & 0x7f));
        data.push_back(m.data[m.size - 1]);
    }
    else
    {
        data.reserve(m.size + 2);
        data.push_back(0x80 + ((time >> 7) & 0x3f));
        data.push_back(0x80 + (time & 0x7f));
        data.insert(data.end(), m.data.data(), m.data.data() + m.size);
    }

    DB(("write %zd bytes.\n", data.size()));
    write(this, handle_, std::move(data), !writeNRSupported_);
}

} // namespace io
