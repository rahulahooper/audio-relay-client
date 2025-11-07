#ifndef UTIL_H

#define UTIL_H

#include "esp_err.h"
#include "esp_log.h"


////////////////////////////////////////////////////////////////////
// get_system_time()
//
// Return time since epoch in microseconds. This function is mostly
// useful for measuring durations. It doesn't return a true time
// of day.
////////////////////////////////////////////////////////////////////
void get_system_time(int64_t* time_us)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    *time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}


////////////////////////////////////////////////////////////////////
// TrafficMeter 
// 
// Light-weight flowmeter for byte traffic. A function can
// update this meter periodically with the bytes it has processed.
// The TrafficMeter will maintain statistics such as bytes processed
// per second, bytes processed per update, time between updates, etc.
//
////////////////////////////////////////////////////////////////////
typedef struct TrafficMeter
{
    uint64_t totalBytes_;
    int64_t timeNow_;
    int64_t timeElapsed_;
    uint32_t numUpdates_;
    
} TrafficMeter;

void TM_Init(TrafficMeter** tm)
{
    *tm = (TrafficMeter*)malloc(sizeof(TrafficMeter));
}

void TM_Start(TrafficMeter* tm)
{
    memset(tm, 0, sizeof(TrafficMeter));
    get_system_time(&tm->timeNow_);
}

void TM_Update(TrafficMeter* tm, uint32_t bytes)
{
    tm->totalBytes_ += bytes;
    tm->numUpdates_++;

    int64_t prevTime = tm->timeNow_;
    get_system_time(&tm->timeNow_);
    int64_t timeElapsed = (tm->timeNow_ - prevTime);

    tm->timeElapsed_ += timeElapsed;
}

int64_t TM_GetTimeElapsed(TrafficMeter* tm)
{
    return tm->timeElapsed_;
}

float TM_GetAvgTimeBetweenUpdates(TrafficMeter* tm)
{
    return tm->timeElapsed_ / tm->numUpdates_ / 1e6;
}

float TM_GetBytesPerSecond(TrafficMeter* tm)
{
    return 1.0f * tm->totalBytes_ / tm->timeElapsed_ * 1e6;
}

void TM_Print(TrafficMeter* tm, const char* tag)
{
    ESP_LOGI(tag, "%f bytes/sec, %f sec/update",
        TM_GetBytesPerSecond(tm), TM_GetAvgTimeBetweenUpdates(tm));
}


////////////////////////////////////////////////////////////////////
// Timer 
// 
// A lightweight timer struct for tracking and reporting timing
// statistics.
////////////////////////////////////////////////////////////////////
typedef struct _Timer
{
    int64_t timeElapsed_;
    uint16_t numUpdates_;

    int64_t startTime_;
    char name_[32];
} Timer;

void Timer_Init(Timer** timer, const char* name)
{
    *timer = malloc(sizeof(Timer));
    memset((*timer)->name_, 0, sizeof(*timer)->name_);
    strlcpy((*timer)->name_, name, sizeof((*timer)->name_) - 1);
}

void Timer_Reset(Timer* timer)
{
    timer->timeElapsed_ = 0;
    timer->numUpdates_ = 0;
    timer->startTime_ = 0;
}

void Timer_Start(Timer* timer)
{
    get_system_time(&timer->startTime_);
}

void Timer_Stop(Timer* timer)
{
    int64_t stop = 0;
    get_system_time(&stop);

    timer->timeElapsed_ += (stop - timer->startTime_);
    timer->numUpdates_++;
}

int64_t Timer_GetTimeElapsed(Timer* timer)
{
    return timer->timeElapsed_;
}

float Timer_GetAvgTimeBetweenUpdates(Timer* timer)
{
    return 1.0f * timer->timeElapsed_ / timer->numUpdates_ / 1e6;
}

void Timer_Print(Timer* timer, const char* tag)
{
    ESP_LOGI(tag, "%s: Time elapsed: %lld, %f seconds / update\n",
        timer->name_, Timer_GetTimeElapsed(timer), Timer_GetAvgTimeBetweenUpdates(timer));
}
#endif // UTIL_H