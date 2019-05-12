/*
 * author : Shuichi TAKANO
 * since  : Fri Jan 11 2019 1:14:37
 */

#include "job_manager.h"
#include <assert.h>
#include <debug.h>
#include <mutex>

namespace sys
{

namespace
{
constexpr int EVENT_ADD_JOB = 1 << 0;
constexpr int EVENT_EXIT    = 1 << 1;
constexpr int EVENT_IDLE    = 1 << 2;
} // namespace

JobManager::~JobManager()
{
    stop();
}

void
JobManager::start(int prio, size_t stackSize, const char* name)
{
    if (started_)
    {
        return;
    }

    eventGroupHandle_ = xEventGroupCreate();
    assert(eventGroupHandle_);

    auto r = xTaskCreate(taskEntry, name, stackSize, this, prio, &taskHandle_);
    assert(r);

    started_ = true;
}

void
JobManager::stop()
{
    if (started_)
    {
        exitReq_ = true;
        xEventGroupWaitBits(eventGroupHandle_,
                            EVENT_EXIT,
                            pdTRUE /* clear */,
                            pdFALSE /* wait for all bit */,
                            portMAX_DELAY);
    }
}

void
JobManager::taskEntry(void* p)
{
    ((JobManager*)p)->task();
}

void
JobManager::add(Job&& f)
{
    std::lock_guard<sys::Mutex> lock(mutex_);
    jobs_.push_back(std::move(f));

    xEventGroupSetBits(eventGroupHandle_, EVENT_ADD_JOB);
    xEventGroupClearBits(eventGroupHandle_, EVENT_IDLE);
}

void
JobManager::waitIdle()
{
    xEventGroupWaitBits(eventGroupHandle_,
                        EVENT_IDLE,
                        pdFALSE /* clear */,
                        pdFALSE /* wait for all bit */,
                        portMAX_DELAY);
}

void
JobManager::task()
{
    mutex_.lock();
    while (!exitReq_)
    {
        if (jobs_.empty())
        {
            idle_ = true;
            mutex_.unlock();

            xEventGroupSetBits(eventGroupHandle_, EVENT_IDLE);
            xEventGroupWaitBits(eventGroupHandle_,
                                EVENT_ADD_JOB,
                                pdTRUE /* clear */,
                                pdFALSE /* wait for all bit */,
                                portMAX_DELAY);

            mutex_.lock();
        }
        else
        {
            idle_ = false;
            mutex_.unlock();
            jobs_.front()();
            mutex_.lock();
            jobs_.pop_front();
        }
    }
    mutex_.unlock();

    xEventGroupSetBits(eventGroupHandle_, EVENT_EXIT);
}

namespace
{
JobManager defaultJobManager_;
}

JobManager&
getDefaultJobManager()
{
    return defaultJobManager_;
}

} // namespace sys
