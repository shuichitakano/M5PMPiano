#include <M5StickC.h>

#include <io/ble_manager.h>
#include <io/ble_midi.h>
#include <io/bluetooth.h>

#include <pm_piano/piano.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <driver/i2s.h>

#include <memory>

namespace
{

io::MidiMessageQueue midiIn_;
physical_modeling_piano::Piano piano_;

constexpr size_t sampleFreq =
    physical_modeling_piano::SystemParameters::sampleRate;
// constexpr uint32_t sampleFreq = 44100 * 2;

static constexpr size_t UNIT_SAMPLES = 128;

} // namespace

struct Encoder
{
    uint32_t residual_{};

public:
    inline uint32_t encode(uint32_t v)
    {
        uint32_t r = 0;

        residual_ += v;
        r |= (residual_ & 0x10000) << 15;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 14;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 13;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 12;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 11;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 10;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 9;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 8;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 7;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 6;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 5;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 4;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 3;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 2;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) << 1;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000);
        residual_ &= 0xffff;

        residual_ += v;
        r |= (residual_ & 0x10000) >> 1;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 2;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 3;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 4;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 5;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 6;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 7;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 8;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 9;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 10;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 11;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 12;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 13;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 14;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 15;
        residual_ &= 0xffff;
        residual_ += v;
        r |= (residual_ & 0x10000) >> 16;
        residual_ &= 0xffff;

        //        printf("v = %d %u\n", v, r);

        return r;
    }
};

uint32_t
getSample()
{
    static float t = 0;

    constexpr float _2pi = 3.1415926535f * 2;
    constexpr float freq = 440;
    constexpr float dt   = freq / sampleFreq * _2pi;

    t += dt;
    if (t > _2pi)
    {
        t -= _2pi;
    }

    auto v = uint32_t((sinf(t) + 1) * 32767);

    static Encoder enc;
    return enc.encode(v);
}

void
initIO()
{
    {
        uint64_t _1 = 1;
        gpio_config_t cnf{};
        cnf.mode         = GPIO_MODE_OUTPUT;
        cnf.pin_bit_mask = ((_1 << 0) | (_1 << 26));
        cnf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cnf.pull_up_en   = GPIO_PULLUP_DISABLE;
        cnf.intr_type    = GPIO_INTR_DISABLE;

        auto r = gpio_config(&cnf);
        assert(r == ESP_OK);
    }

    auto initI2S = [](auto port, int pin) {
        i2s_config_t cfg{};

        cfg.mode            = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        cfg.sample_rate     = sampleFreq;
        cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format  = I2S_CHANNEL_FMT_RIGHT_LEFT;
        cfg.communication_format =
            i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
        cfg.intr_alloc_flags = 0;
        cfg.dma_buf_count    = 2;
        cfg.dma_buf_len      = UNIT_SAMPLES;
        cfg.use_apll         = false;

        auto r = i2s_driver_install(port, &cfg, 0, nullptr);
        assert(r == ESP_OK);

        {
            i2s_pin_config_t cfg{};
            cfg.bck_io_num   = I2S_PIN_NO_CHANGE;
            cfg.ws_io_num    = I2S_PIN_NO_CHANGE;
            cfg.data_out_num = pin;
            cfg.data_in_num  = I2S_PIN_NO_CHANGE;

            auto r = i2s_set_pin(port, &cfg);
            assert(r == ESP_OK);
        }
    };

    initI2S(I2S_NUM_0, 26);
    initI2S(I2S_NUM_1, 0);
}

void
soundTask(void*)
{
    Encoder encL;
    while (1)
    {
        static int32_t samples[UNIT_SAMPLES];
        memset(samples, 0, sizeof(samples));
        piano_.update(samples, UNIT_SAMPLES, midiIn_);

        for (int i = 0; i < UNIT_SAMPLES; ++i)
        {
            samples[i] = encL.encode(samples[i] + 32768);
        }
#if 0
        uint32_t sample[128];
        for (int i = 0; i < 128; ++i)
        {
            sample[i] = getSample();
        }
#endif
        size_t writeBytes;
        i2s_write(
            I2S_NUM_0, samples, sizeof(samples), &writeBytes, portMAX_DELAY);
        // i2s_write(
        //     I2S_NUM_1, samples, sizeof(samples), &writeBytes, portMAX_DELAY);
    }
}

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

    io::BLEMidiClient::instance().setMIDIIn(&midiIn_);
    io::BLEManager::instance().registerClientProfile(
        io::BLEMidiClient::instance());

    io::BLEManager::instance().startScan();

    midiIn_.setActive(true);

    DBOUT(("piano = %dbytes.\n", sizeof(piano_)));
    piano_.initialize(8);

    initIO();

    xTaskCreate(&soundTask, "sound_task", 2048 + 1024, NULL, 5, NULL);

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

        delay(1);
    }
}