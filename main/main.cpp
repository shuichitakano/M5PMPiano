#include <M5StickC.h>

#include <io/ble_manager.h>
#include <io/ble_midi.h>
#include <io/bluetooth.h>

#include <pm_piano/piano.h>

#include <graphics/bmp.h>
#include <util/binary.h>

#include <graphics/framebuffer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <driver/i2s.h>

#include <memory>

DEF_LINKED_BINARY(kb_mini_bmp);

namespace
{

io::MidiMessageQueue midiIn_;
physical_modeling_piano::Piano piano_;

#define DELTA_SIGMA 1

constexpr size_t sampleFreq =
    physical_modeling_piano::SystemParameters::sampleRate;

constexpr int overSampleShiftDeltaSigma = 1;
constexpr int overSampleShift           = 2;

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
#if DELTA_SIGMA
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
        cfg.sample_rate     = sampleFreq << overSampleShiftDeltaSigma;
        cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        cfg.channel_format  = I2S_CHANNEL_FMT_RIGHT_LEFT;
        cfg.communication_format =
            i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
        cfg.intr_alloc_flags = 0;
        cfg.dma_buf_count    = 4;
        cfg.dma_buf_len      = UNIT_SAMPLES << overSampleShiftDeltaSigma;
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
#else
    i2s_config_t cfg{};

    cfg.mode =
        (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
    cfg.sample_rate          = sampleFreq << overSampleShift;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S_MSB);
    cfg.intr_alloc_flags     = 0;
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = UNIT_SAMPLES << overSampleShift;
    cfg.use_apll             = false;

    auto r = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    assert(r == ESP_OK);

    i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);

#endif
}

void
soundTask(void*)
{
#if DELTA_SIGMA
    Encoder encL;
    while (1)
    {
        static int32_t samples[UNIT_SAMPLES];
        memset(samples, 0, sizeof(samples));
        piano_.update(samples, UNIT_SAMPLES, midiIn_);

        static uint32_t out[UNIT_SAMPLES << overSampleShiftDeltaSigma];
        auto* dst = out;
        auto* src = samples;
        for (int i = 0; i < UNIT_SAMPLES; ++i)
        {
            auto v = *src + 32768;
            dst[0] = encL.encode(v);
            dst[1] = encL.encode(v);
            src += 1;
            dst += 2;
        }

        size_t writeBytes;
        i2s_write(I2S_NUM_0, out, sizeof(out), &writeBytes, portMAX_DELAY);
        //        i2s_write(I2S_NUM_1, out, sizeof(out), &writeBytes,
        //        portMAX_DELAY);
    }
#else
    static int32_t samples[UNIT_SAMPLES];
    static uint16_t pcm[((UNIT_SAMPLES << overSampleShift) << 1)];
    int residual = 0;
    int pv       = 32768;

    while (1)
    {
        memset(samples, 0, sizeof(samples));
        piano_.update(samples, UNIT_SAMPLES, midiIn_);

        const auto* src = samples;
        auto* dst       = pcm;
        int ct          = UNIT_SAMPLES;
        do
        {
            int v = *src + 32768;

#if 0
            dst[0] = v;
            dst[1] = v;
            ++src;
            dst += 2;
#else
            int v0   = ((pv * 3 + v) >> 2) + residual;
            int vq   = v0 & 0xff00;
            residual = v0 - vq;
            dst[0]   = vq;
            dst[1]   = vq;

            int v1   = ((pv + v) >> 1) + residual;
            vq       = v1 & 0xff00;
            residual = v1 - vq;
            dst[2]   = vq;
            dst[3]   = vq;

            int v2   = ((pv + v * 3) >> 2) + residual;
            vq       = v2 & 0xff00;
            residual = v2 - vq;
            dst[4]   = vq;
            dst[5]   = vq;

            int v3   = v + residual;
            vq       = v3 & 0xff00;
            residual = v3 - vq;
            dst[6]   = vq;
            dst[7]   = vq;

            src += 1;
            dst += 8;
            pv = v;
#endif
        } while (--ct);

        size_t writeBytes;
        i2s_write(I2S_NUM_0, pcm, sizeof(pcm), &writeBytes, portMAX_DELAY);
    }

#endif
}

class KeyboardDisp
{
    graphics::Framebuffer fb_;
    graphics::Texture tex_;

    struct MaskDef
    {
        int x, y, w, h;
        int ox; //< オクターブ先頭からのオフセット
    };

public:
    void initialize()
    {
        tex_.initialize(GET_LINKED_BINARY_T(graphics::BMP, kb_mini_bmp));
        fb_.resize(156, 20);
    }

    void clear()
    {
        fb_.put(tex_, 0, 0, 0, 20, 6, 20);
        for (int i = 0; i < 7; ++i)
        {
            fb_.put(tex_, 6 + i * 21, 0, 6, 20, 21, 20);
        }
        fb_.put(tex_, 153, 0, 27, 20, 3, 20);
    }

    void update()
    {
        static const MaskDef maskDef[] = {
            {0, 4, 2, 16, 15},  // a0
            {8, 3, 2, 10, 17},  // a+0
            {12, 4, 2, 16, 18}, // b0

            {0, 4, 2, 16, 0},  // c
            {8, 3, 2, 10, 2},  // c+
            {18, 4, 2, 16, 3}, // d
            {8, 3, 2, 10, 5},  // d+
            {12, 4, 2, 16, 6}, // e

            {0, 4, 2, 16, 9},   // f
            {8, 3, 2, 10, 11},  // f+
            {18, 4, 2, 16, 12}, // g
            {8, 3, 2, 10, 14},  // g+
            {18, 4, 2, 16, 15}, // a
            {8, 3, 2, 10, 17},  // a+
            {12, 4, 2, 16, 18}, // b

            {27, 4, 2, 16, 0}, // c8
        };

        clear();

        const auto& keyOnState = piano_.getKeyOnStateForDisp();
        for (size_t i = 0; i < keyOnState.size(); ++i)
        {
            if (keyOnState[i])
            {
                int oct   = (i + 9) / 12;
                int note  = (i + 9) % 12;
                int baseX = (oct - 1) * 7 * 3 + 6;
                int maskIdx;
                if (i < 3)
                {
                    maskIdx = i;
                }
                else if (i == 87)
                {
                    maskIdx = 15;
                }
                else
                {
                    maskIdx = note + 3;
                }

                const auto& mask = maskDef[maskIdx];
                fb_.putTrans(tex_,
                             baseX + mask.ox,
                             mask.y,
                             mask.x,
                             mask.y,
                             mask.w,
                             mask.h);
            }
        }

        M5.Lcd.drawBitmap(2, 50, 156, 20, fb_.getBits());
    }
};

extern "C" void
app_main()
{
    DBOUT(("start\n"));

    M5.begin();
    M5.Axp.ScreenBreath(9);
    M5.Lcd.setRotation(3);

    M5.Lcd.fillRect(0, 0, 160, 12, graphics::makeColor(128, 20, 0));
    M5.Lcd.setCursor(4, 2);
    M5.Lcd.setTextColor(0xffff);
    M5.Lcd.print("M5 PM PIANO");

    KeyboardDisp kbDisp;
    kbDisp.initialize();

    auto& btManager = io::BLEManager::instance();
    if (!io::initializeBluetooth() || !btManager.initialize())
    {
        DBOUT(("bluetooth initialize error."));
    }
    io::setBluetoothDeviceName("M5PMPiano");

    auto& btMidi = io::BLEMidiClient::instance();
    btMidi.setMIDIIn(&midiIn_);
    btManager.registerClientProfile(btMidi);

    //    btManager.startScan();

    midiIn_.setActive(true);

    DBOUT(("piano = %dbytes.\n", sizeof(piano_)));
    piano_.initialize(10);

    initIO();

    xTaskCreate(&soundTask, "sound_task", 2048 + 1024, NULL, 15, NULL);

    bool connected = false;

    while (1)
    {
        if (!btManager.isActive(btMidi) && !btManager.isScanning())
        {
            DBOUT(("bt idle.\n"));
            if (connected)
            {
                M5.Lcd.fillRect(0, 72, 160, 8, 0);
            }
            connected = false;
            btManager.startScan();
        }
        if (!connected)
        {
            auto& name = btMidi.getDeviceName();
            if (!name.empty())
            {
                M5.Lcd.setTextColor(0x02ff, 0);
                M5.Lcd.setCursor(2, 72);
                M5.Lcd.print(name.c_str());
                connected = true;
            }
        }

        kbDisp.update();

        int v = M5.Axp.GetVbatData();
        M5.Lcd.setTextColor(graphics::makeColor(168, 168, 168), 0);
        M5.Lcd.setCursor(160 - 6 * 6, 14);
        M5.Lcd.printf("%d.%03dV", v / 1000, v % 1000);
        M5.Lcd.setCursor(2, 14);
        M5.Lcd.printf("Voice:%zd ", piano_.getCurrentNoteCount());

        delay(1);
    }
}