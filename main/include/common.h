#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#define AUDIO_PACKET_MAX_SAMPLES 300        // maximum number of audio samples we can send to server at a time 
#define AUDIO_PACKET_BYTES_PER_SAMPLE 3     // size of each sample within an audio packet in bytes
#define MAX_SEND_ATTEMPTS 3                 // maximum attempts to send a single audio packet

#define min(a,b) ((a) < (b) ? (a) : (b))

#define DEBUG 0
#if DEBUG
    #define PRINTF_DEBUG( msg ) ESP_LOGI msg
#else
    #define PRINTF_DEBUG( msg )
#endif

////////////////////////////////////////////////////////////////////
// get_system_time()
//
// Return time since epoch in microseconds. This function is mostly
// useful for measuring durations. It doesn't return a true time
// of day.
////////////////////////////////////////////////////////////////////
static void get_system_time(int64_t* time_us)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    *time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

typedef struct AudioPacket_t
{
    uint16_t seqnum;
    bool     echo;
    uint16_t numSamples;
    uint16_t payloadStart;              // start index of audio data in <payload>
    uint16_t checksum;
    uint8_t  payload[AUDIO_PACKET_MAX_SAMPLES * AUDIO_PACKET_BYTES_PER_SAMPLE];
} AudioPacket_t;

#endif // COMMON_H