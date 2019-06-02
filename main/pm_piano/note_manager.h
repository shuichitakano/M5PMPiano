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
#include <vector>

namespace physical_modeling_piano
{

class NoteManager
{
    static constexpr int NOTE_BEGIN = 21;
    static constexpr int NOTE_END   = 109;
    static constexpr size_t N_NOTES = NOTE_END - NOTE_BEGIN;

    std::array<Note, N_NOTES> notes_;
    std::array<int8_t, N_NOTES> noteNode_;

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

public:
    void initialize(const SystemParameters& sysParams, size_t nPoly);
    void keyOn(int note, float v);
    void keyOff(int note);

    void update(Note::SampleT* samples,
                size_t nSamples,
                const SystemParameters& sysParams,
                const PedalState& pedal);

protected:
    int getNodeIndex(Node* node) const;

    Node* allocateNode();
    void freeNode(Node* node);
    void pushActive(Node* node);
    void pushFrontActive(Node* node);
    Node* popFrontActive();
    void removeActive(Node* node);
};

} // namespace physical_modeling_piano

#endif /* _103DE5E1_1134_152A_154E_889BBA4369C5 */
