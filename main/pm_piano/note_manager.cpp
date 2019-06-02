/*
 * author : Shuichi TAKANO
 * since  : Sat May 11 2019 22:39:33
 */

#include "note_manager.h"
#include <algorithm>
#include <assert.h>

namespace physical_modeling_piano
{

void
NoteManager::initialize(const SystemParameters& sysParams, size_t nPoly)
{
    size_t allocatorSize = 0;

    for (int i = 0; i < N_NOTES; ++i)
    {
        float f = 440 * powf(2.0f, (i + NOTE_BEGIN - 69) / 12.0f);
        notes_[i].initialize(f, sysParams);

        allocatorSize =
            std::max(allocatorSize, notes_[i].computeAllocatorSize());
    }

    printf("note %zd bytes, notes %zd, st %zd, allocator %zd\n",
           sizeof(Note),
           sizeof(notes_),
           sizeof(Note::State),
           allocatorSize);

    std::fill(noteNode_.begin(), noteNode_.end(), -1);

    nodes_.resize(nPoly);
    for (auto&& n : nodes_)
    {
        n.state_.initialize(allocatorSize);
        freeNode(&n);
    }
}

void
NoteManager::update(Note::SampleT* samples,
                    size_t nSamples,
                    const SystemParameters& sysParams,
                    const PedalState& pedal)
{
    auto* node = active_;
    while (node)
    {
        //        printf("update %p, %d\n", node, getNodeIndex(node));
        auto noteIdx = node->noteIndex_;
        notes_[noteIdx].update(
            samples, nSamples, node->state_, sysParams, pedal);

        if (node->state_.idle)
        {
            //            printf("to idle %d\n", noteIdx + 21);
            noteNode_[noteIdx] = -1;

            auto next = node->next_;
            removeActive(node);
            freeNode(node);
            node = next;
        }
        else
        {
            node = node->next_;
        }
    }
}

void
NoteManager::keyOn(int note, float v)
{
    note -= NOTE_BEGIN;

    Node* node;
    int nodeIndex = noteNode_[note];
    if (nodeIndex >= 0)
    {
        node = &nodes_[nodeIndex];
        assert(node->noteIndex_ == note);
    }
    else
    {
        node = allocateNode();
        if (!node)
        {
            node = popFrontActive();

            noteNode_[node->noteIndex_] = -1;
        }
        assert(node);

        node->noteIndex_ = note;
        noteNode_[note]  = getNodeIndex(node);
        pushActive(node);
    }

    notes_[note].keyOn(node->state_, v);

    // printf("allocated node: %p, idx %d, note %d\n",
    //        node,
    //        noteNode_[note],
    //        note + 21);
}

void
NoteManager::keyOff(int note)
{
    note -= NOTE_BEGIN;
    int nodeIndex = noteNode_[note];
    if (nodeIndex < 0)
    {
        return;
    }
    auto* node = &nodes_[nodeIndex];
    assert(node->noteIndex_ == note);

    notes_[note].keyOff(node->state_);

    // 先頭に持っていく
    if (active_ != node)
    {
        removeActive(node);
        pushFrontActive(node);
    }
}

int
NoteManager::getNodeIndex(Node* node) const
{
    return node - nodes_.data();
}

NoteManager::Node*
NoteManager::allocateNode()
{
    if (free_)
    {
        auto r = free_;
        free_  = free_->next_;
        return r;
    }
    return nullptr;
}

void
NoteManager::freeNode(Node* node)
{
    node->next_ = free_;
    free_       = node;
}

void
NoteManager::pushActive(Node* node)
{
    if (activeTail_)
    {
        assert(active_);
        activeTail_->next_ = node;
        node->prev_        = activeTail_;
        node->next_        = nullptr;
        activeTail_        = node;
    }
    else
    {
        active_     = node;
        activeTail_ = node;
        node->prev_ = nullptr;
        node->next_ = nullptr;
    }
}

void
NoteManager::pushFrontActive(Node* node)
{
    if (active_)
    {
        assert(activeTail_);
        active_->prev_ = node;
        node->next_    = active_;
        node->prev_    = nullptr;
        active_        = node;
    }
    else
    {
        active_     = node;
        activeTail_ = node;
        node->prev_ = nullptr;
        node->next_ = nullptr;
    }
}

NoteManager::Node*
NoteManager::popFrontActive()
{
    if (!active_)
    {
        return nullptr;
    }
    auto r  = active_;
    active_ = r->next_;
    if (active_)
    {
        active_->prev_ = nullptr;
    }
    else
    {
        assert(activeTail_ == r);
        activeTail_ = nullptr;
    }
    return r;
}

void
NoteManager::removeActive(Node* node)
{
    if (node->prev_)
    {
        node->prev_->next_ = node->next_;
    }
    else
    {
        assert(active_ == node);
        active_ = node->next_;
    }

    if (node->next_)
    {
        node->next_->prev_ = node->prev_;
    }
    else
    {
        assert(activeTail_ == node);
        activeTail_ = node->prev_;
    }
}

} // namespace physical_modeling_piano
