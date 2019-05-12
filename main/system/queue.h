/*
 * author : Shuichi TAKANO
 * since  : Wed Jan 02 2019 15:26:20
 */
#ifndef EDE9CD63_3134_1397_E8E0_1F358F8E7C1A
#define EDE9CD63_3134_1397_E8E0_1F358F8E7C1A

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace sys
{

template <class T>
class Queue
{
    QueueHandle_t handle_;

public:
    Queue(size_t capacity) { handle_ = xQueueCreate(capacity, sizeof(T)); }
    ~Queue() { vQueueDelete(handle_); }

    bool push(const T& v) { return xQueueSend(handle_, &v, portMAX_DELAY); }
    bool pop(T* v) { return xQueueReceive(handle_, v, portMAX_DELAY); }
    void clear() { xQueueReset(handle_); }

    size_t size() const { return uxQueueMessagesWaiting(handle_); }
    size_t getSpace() const { return uxQueueSpacesAvailable(handle_); }
};

template <class T>
class QueueWithSwitch
{
    Queue<T> queue_;
    bool enabled_;

public:
    QueueWithSwitch(size_t capacity, bool state = true)
        : queue_(capacity)
        , enabled_(state)
    {
    }

    void setEnable(bool f = true) { enabled_ = f; }
    bool isEnabled() const { return enabled_; }

    bool push(const T& v)
    {
        if (enabled_)
        {
            return queue_.push(v);
        }
        return false;
    }

    bool pop(T* v) { return queue_.pop(v); }

    void clear() { queue_.clear(); }
    size_t size() const { return queue_.size(); }
    size_t getSpace() const { return isEnabled() ? queue_.getSpace() : 0; }
};

} // namespace sys

#endif /* EDE9CD63_3134_1397_E8E0_1F358F8E7C1A */
