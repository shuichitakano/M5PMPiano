/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 5:26:20
 */
#ifndef _0BD14EBE_B133_F0D1_5047_03A8B485BC4D
#define _0BD14EBE_B133_F0D1_5047_03A8B485BC4D

#include "ble_manager.h"
#include "midi.h"

namespace io
{

class BLEMidiClient : public BLEClientHandler, public MidiOut
{
    std::vector<BLEServiceEntry> list_;

    int handle_            = -1;
    bool readable_         = false;
    bool writable_         = false;
    bool writeNRSupported_ = false;

    MidiMessageQueue* midiIn_ = nullptr;

    MidiMessageMaker midiInMessageMaker_;

public:
    // BLEClientHandler
    bool onScanEntry(const BLEServiceEntry& e) override;
    void onSearchService(
        const std::string& primary,
        const std::string& secondary,
        const std::vector<BLECharacteristic>& characteristics) override;

    void onNotify(const uint8_t* p, size_t size, int handle) override;
    void onRead(const uint8_t* p, size_t size, int handle) override;
    void onWriteComplete(bool result, int handle) override;

    // MIDIOut
    void put(const MidiMessage& m) override;

    void setMIDIIn(MidiMessageQueue* m) { midiIn_ = m; }

    static BLEMidiClient& instance();
};

} // namespace io

#endif /* _0BD14EBE_B133_F0D1_5047_03A8B485BC4D */
