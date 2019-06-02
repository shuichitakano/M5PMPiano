/*
 * author : Shuichi TAKANO
 * since  : Sun May 12 2019 23:12:11
 */
#ifndef DC39274B_A134_1524_1625_4EACE18FA496
#define DC39274B_A134_1524_1625_4EACE18FA496

#include "note_manager.h"
#include "soundboard.h"
#include <io/midi.h>

namespace physical_modeling_piano
{

class Piano
{
    NoteManager noteManager_;
    Soundboard soundboard_;

    SystemParameters sysParams_;
    PedalState pedal_;

public:
    Piano() {}

    void initialize(size_t nPoly);
    void
    update(int32_t* samples, size_t nSamples, io::MidiMessageQueue& midiIn);

    size_t getCurrentNoteCount() const
    {
        return noteManager_.getCurrentNoteCount();
    }
};

} // namespace physical_modeling_piano

#endif /* DC39274B_A134_1524_1625_4EACE18FA496 */
