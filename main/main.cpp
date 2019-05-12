#include <M5StickC.h>

#include <io/ble_manager.h>
#include <io/ble_midi.h>
#include <io/bluetooth.h>

extern "C" void
app_main()
{
    DBOUT(("enter main.\n"));

    M5.begin();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor(0xffff);
    M5.Lcd.print("M5 PM PIANO");

    if (!io::initializeBluetooth() || !io::BLEManager::instance().initialize())
    {
        DBOUT(("bluetooth initialize error."));
    }
    io::setBluetoothDeviceName("M5PMPiano");

    static io::MidiMessageQueue midiIn;
    io::BLEMidiClient::instance().setMIDIIn(&midiIn);
    io::BLEManager::instance().registerClientProfile(
        io::BLEMidiClient::instance());

    io::BLEManager::instance().startScan();

    midiIn.setActive(true);

    while (1)
    {
        M5.Lcd.setTextColor(0xff00, 0);
        M5.Lcd.setCursor(0, 12);
        M5.Lcd.printf("vbat:%d ", M5.Axp.GetVbatData());
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("Ich:%d ", M5.Axp.GetIchargeData());
        M5.Lcd.setCursor(0, 28);
        M5.Lcd.printf("Idch:%d ", M5.Axp.GetIdischargeData());
        M5.Lcd.setCursor(0, 36);
        M5.Lcd.printf("Vin:%d ", M5.Axp.GetVinData());

        io::MidiMessage m;
        while (midiIn.get(&m))
        {
            DBOUT(("in: "));
            m.dump();
        }

        delay(1);
    }
}