/*
 * author : Shuichi TAKANO
 * since  : Fri Jan 11 2019 1:9:43
 */
#ifndef _99A8897A_4134_1399_10B0_E9EA535CAB62
#define _99A8897A_4134_1399_10B0_E9EA535CAB62

#include "mutex.h"
#include <deque>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <functional>

namespace sys
{

class JobManager
{
    using Job = std::function<void()>;
    std::deque<Job> jobs_;
    sys::Mutex mutex_;
    TaskHandle_t taskHandle_{};
    EventGroupHandle_t eventGroupHandle_{};

    bool started_ = false;
    bool exitReq_ = false;
    bool idle_    = true;

public:
    ~JobManager();

    void start(int prio         = 0,
               size_t stackSize = 2048,
               const char* name = "JobManager");
    void stop();

    void add(Job&& f);

    void waitIdle();
    bool isIdle() const { return idle_; }

protected:
    void task();
    static void taskEntry(void* p);
}; // namespace sys

JobManager& getDefaultJobManager();

} // namespace sys

#endif /* _99A8897A_4134_1399_10B0_E9EA535CAB62 */
