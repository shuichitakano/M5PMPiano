/*
 * author : Shuichi TAKANO
 * since  : Sat May 11 2019 22:34:15
 */
#ifndef _103DE5E1_1134_152A_154E_889BBA4369C5
#define _103DE5E1_1134_152A_154E_889BBA4369C5

#include "note.h"
#include "pedal.h"
#include "sys_params.h"
#include <array>
#include <atomic>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

namespace physical_modeling_piano
{

class NoteManager
{
    static constexpr int NOTE_BEGIN = 21;
    static constexpr int NOTE_END   = 109;
    static constexpr size_t N_NOTES = NOTE_END - NOTE_BEGIN;

    std::array<Note, N_NOTES> notes_;
    std::array<int8_t, N_NOTES> noteNode_;
    std::array<bool, N_NOTES> keyOnStateForDisp_;

    struct Node
    {
        Note::State state_;
        int noteIndex_{};

        Node* prev_{};
        Node* next_{};
    };

    std::vector<Node> nodes_;
    Node* free_{};   // 片方向
    Node* active_{}; // 双方向
    Node* activeTail_{};

    const SystemParameters* currentSysParams_{};
    const PedalState* currentPedalState_{};
    std::vector<Node*> workNodes_;
    std::atomic<int> workIdx_;

    std::vector<Note::SampleT> workerSamples_{};

    size_t currentNoteCount_{};

    TaskHandle_t workerTaskHandle_{};
    EventGroupHandle_t eventGroupHandle_{};

public:
    void initialize(const SystemParameters& sysParams, size_t nPoly);
    void keyOn(int note, float v);
    void keyOff(int note);

    void update(Note::SampleT* samples,
                size_t nSamples,
                const SystemParameters& sysParams,
                const PedalState& pedal);

    size_t getCurrentNoteCount() const { return currentNoteCount_; }
    const std::array<bool, N_NOTES>& getKeyOnStateForDisp() const
    {
        return keyOnStateForDisp_;
    }

protected:
    int getNodeIndex(Node* node) const;

    Node* allocateNode();
    void freeNode(Node* node);
    void pushActive(Node* node);
    void pushFrontActive(Node* node);
    Node* popFrontActive();
    void removeActive(Node* node);

    int process(Note::SampleT* samples, size_t nSamples);

    static void workerEntry(void* p);
    void worker();
};

} // namespace physical_modeling_piano

#endif /* _103DE5E1_1134_152A_154E_889BBA4369C5 */
