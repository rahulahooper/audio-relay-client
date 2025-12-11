#ifndef SAMPLE_H
#define SAMPLE_H

#include "driver/gptimer.h"

#include "common.h"
#include "pure_tones.h"

#define PCM4201_BYTES_PER_SAMPLE 8

typedef enum SamplingTaskAudioSource_t
{
    AUDIO_SOURCE_EXTERNAL_ADC,
    AUDIO_SOURCE_INTERNAL_ADC,
    AUDIO_SOURCE_BUFFER,
} SamplingTaskAudioSource_t;

typedef struct SamplingTaskConfig_t
{
    uint32_t sampleRate;
    SamplingTaskAudioSource_t audioSource;
} SamplingTaskConfig_t;

typedef struct ExternalAdcCircularBuffer_t
{
    uint16_t      start;
    uint16_t      size;
    uint8_t       buffer[AUDIO_PACKET_MAX_SAMPLES * PCM4201_BYTES_PER_SAMPLE];
} ExternalAdcCircularBuffer_t;



////////////////////////////////////////////////////////////////////
// setup_external_adc 
//
// Initialize the PCM4201 analog-to-digital converter
////////////////////////////////////////////////////////////////////
esp_err_t setup_external_adc(i2s_chan_handle_t* i2sHandle, const uint32_t sampleRate);



////////////////////////////////////////////////////////////////////
// setup_esp32_adc 
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
esp_err_t setup_esp32_adc(gptimer_handle_t* gpTimerHandle, const uint32_t sampleRate);



////////////////////////////////////////////////////////////////////
// esp32_adc_collect_samples 
//
// Collect samples using the internal ESP32 ADC. Sample until
// both the sampling and transmit tasks are ready to exchange data.
////////////////////////////////////////////////////////////////////
esp_err_t esp32_adc_collect_samples(gptimer_handle_t* gpTimerHandle);



////////////////////////////////////////////////////////////////////
// external_adc_collect_samples 
//
// Collect samples using the PCM4201 ADC. Sample until both
// the sampling and transmit tasks are ready to exchange data.
////////////////////////////////////////////////////////////////////
esp_err_t external_adc_collect_samples(i2s_chan_handle_t* i2sHandle);



////////////////////////////////////////////////////////////////////
// local_buffer_collect_samples()
//
////////////////////////////////////////////////////////////////////
esp_err_t local_buffer_collect_samples();



////////////////////////////////////////////////////////////////////
// sampling_task_main
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
void sampling_task_main(void* pvParameters);


#endif // SAMPLE_H