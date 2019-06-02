/*
 * author : Shuichi TAKANO
 * since  : Sun May 12 2019 23:15:6
 */

#include "piano.h"

namespace physical_modeling_piano
{

void
Piano::initialize(size_t nPoly)
{
    noteManager_.initialize(sysParams_, nPoly);
    soundboard_.initialize(sysParams_);
}

void
Piano::update(int32_t* samples, size_t nSamples, io::MidiMessageQueue& midiIn)
{
    // 入出力は同じバッファで大丈夫
    static_assert(sizeof(Note::SampleT) == sizeof(int32_t), "");
    static_assert(sizeof(Soundboard::ResultT) == sizeof(int32_t), "");

    io::MidiMessage m;
    while (midiIn.get(&m))
    {
        auto cmd = m.data[0] & 0xf0;
        if (cmd == 0x80)
        {
            noteManager_.keyOff(m.data[1]);
        }
        else if (cmd == 0x90)
        {
            float v = m.data[2] * (10 / 127.0f);
            noteManager_.keyOn(m.data[1], v);
        }
        else if (cmd == 0xb0)
        {
            switch (m.data[1])
            {
            case 64:
                pedal_.setDamper(m.data[2] >= 64);
                break;

            case 66:
                pedal_.setSostenuto(m.data[2] >= 64);
                break;
            }
        }
    }

    noteManager_.update(reinterpret_cast<Note::SampleT*>(samples),
                        nSamples,
                        sysParams_,
                        pedal_);

    soundboard_.update(reinterpret_cast<Soundboard::ResultT*>(samples),
                       reinterpret_cast<Note::SampleT*>(samples),
                       nSamples);
}

SystemParameters::DeltaTimeT SystemParameters::deltaTF =
    1.0f / SystemParameters::sampleRate;
SystemParameters::DeltaTimeT SystemParameters::deltaT_2F =
    0.5f / SystemParameters::sampleRate;

} // namespace physical_modeling_piano
