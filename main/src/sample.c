///////////////////////////////////////////////////////////////////////
///
/// sample.c
///
/// The sampling task is responsible for receiving audio data from
/// the PCM4201 ADC. The sampling task exchanges data with the transmit
/// task using double-buffering. Specifically, the task loads audio
/// data into a "background packet". When the transmit task signals
/// that it is ready to transmit data to the receiving ESP32, the sampling
/// task swaps the "background" and "active" packets. 
///
///////////////////////////////////////////////////////////////////////
#include "sample.h"

static QueueHandle_t dmaBufferOverflowQueue;

static ExternalAdcCircularBuffer_t adcBuffer;

extern AudioPacket_t * activePacket;     // transmitting task transmits this packet
extern AudioPacket_t * backgroundPacket; // sampling task fills this packet

extern const UBaseType_t transmissionDoneNotifyIndex;      // set by the transmission thread when it is done transmitting data
extern const UBaseType_t dataReadyNotifyIndex;             // set by the sampling thread when there is new data to transmit

extern TaskHandle_t transmitTaskHandle;
extern TaskHandle_t samplingTaskHandle;

////////////////////////////////////////////////////////////////////
// timer_isr_handler 
//
// Timer interrupt handler for sampling task
////////////////////////////////////////////////////////////////////
static bool IRAM_ATTR timer_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{

    (void)timer;
    (void)edata;
    (void)user_ctx;

    static uint32_t i = 0;      // counter is stored in static memory so that it persists across interrupts

    if (backgroundPacket->numSamples == 0) // main sampling_task is responsible for resetting this parameter
    {
        // reset the counter
        i = 0;
    }

    // Add a 3-byte sample to the background packet
    for (int j = 0; j < AUDIO_PACKET_BYTES_PER_SAMPLE; j++)
    {
        backgroundPacket->payload[i] = random() & 0xFF;
        i = (i+1) % AUDIO_PACKET_MAX_SAMPLES;
    }

    // Check for wrap-arounds
    if (backgroundPacket->numSamples == AUDIO_PACKET_MAX_SAMPLES)
    {
        backgroundPacket->payloadStart = (backgroundPacket->payloadStart + AUDIO_PACKET_BYTES_PER_SAMPLE) % AUDIO_PACKET_MAX_SAMPLES;
    }
    else
    {
        backgroundPacket->numSamples++;
    }

    return pdFALSE;
}



////////////////////////////////////////////////////////////////////
// i2s_rx_queue_overflow_callback 
//
// Reports whether the I2S DMA ring buffer overflowed
////////////////////////////////////////////////////////////////////
static IRAM_ATTR bool i2s_rx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    // handle RX queue overflow event ...
    if (pdTRUE == xQueueIsQueueFullFromISR(dmaBufferOverflowQueue))
    {
        size_t dmaBufferSize;
        xQueueReceiveFromISR(dmaBufferOverflowQueue, &dmaBufferSize, NULL);
    }

    xQueueSendFromISR(dmaBufferOverflowQueue, &event->size, NULL);
    return false;
}



////////////////////////////////////////////////////////////////////
// setup_external_adc 
//
// Initialize the PCM4201 analog-to-digital converter
////////////////////////////////////////////////////////////////////
esp_err_t setup_external_adc(i2s_chan_handle_t* i2sHandle, const uint32_t sampleRate)
{
    // set configuration GPIOs
    gpio_num_t PCM4201_GPIO_HIGH_PASS_FILTER_DISABLE = GPIO_NUM_18;
    gpio_num_t PCM4201_GPIO_RESET                    = GPIO_NUM_5;
    gpio_num_t PCM4201_GPIO_RATE                     = GPIO_NUM_4;      // leave this pin floating for normal speed, high perf mode
    gpio_num_t PCM4201_GPIO_MASTER_SLAVE             = GPIO_NUM_12;
    
    gpio_num_t gpios[] = { 
        PCM4201_GPIO_HIGH_PASS_FILTER_DISABLE, 
        PCM4201_GPIO_RESET,
        PCM4201_GPIO_RATE,
        PCM4201_GPIO_MASTER_SLAVE
    };

    for (int i = 0; i < sizeof(gpios) / sizeof(gpio_num_t); i++)
    {
        ESP_ERROR_CHECK(gpio_reset_pin(gpios[i]));
        ESP_ERROR_CHECK(gpio_set_direction(gpios[i], GPIO_MODE_OUTPUT));
    }

    ESP_ERROR_CHECK(gpio_set_level(PCM4201_GPIO_HIGH_PASS_FILTER_DISABLE, 1));           // disable the filter for now
    ESP_ERROR_CHECK(gpio_set_level(PCM4201_GPIO_RESET,                    1));           // reset is active low
    ESP_ERROR_CHECK(gpio_set_level(PCM4201_GPIO_RATE,                     0));           // double speed mode. In this mode, BCK = 64fs
    ESP_ERROR_CHECK(gpio_set_level(PCM4201_GPIO_MASTER_SLAVE,             1));           // ESP32 will provide PCM4201 with timing signals

    // configure I2S interface
    // The PCM4201 sends data in frames, which consist of a single FSYNC cycle. 
    // In Normal speed modes, the PCM4201 expects frames to contain 128 BCK cycles,
    // while in n Double speed mode, a frame contains 64 BCK cycles. Since the ESP32 
    // allows for a maximum of 64 cycles per frame (32 cycles with FSYNC high, 32
    // cycles with FSYNC low), we use Double speed mode. 
    i2s_chan_config_t i2sChanConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&i2sChanConfig, NULL, i2sHandle));

    i2s_std_config_t i2sStdConfig = 
    {
       .clk_cfg = 
       {
           .sample_rate_hz = sampleRate,
           .clk_src = I2S_CLK_SRC_APLL,
           .mclk_multiple = I2S_MCLK_MULTIPLE_128,  // clock seems to be off by factor of 2?
       },
       .slot_cfg =
       {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,

            .ws_width       = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol         = true,
            .bit_shift      = false,
            .msb_right      = false
       },
       .gpio_cfg = 
       {
            .mclk = GPIO_NUM_0,
            .bclk = GPIO_NUM_14,
            .din  = GPIO_NUM_27,
            .ws   = GPIO_NUM_17,
            .dout = I2S_GPIO_UNUSED,
            .invert_flags = 
            {
                .bclk_inv = false,
                .mclk_inv = false,
                .ws_inv   = true             // PCM4201 expects a frame to start off with WS high
            }
       }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*i2sHandle, &i2sStdConfig));

    // Register an event callback that fires when the I2S RX DMA buffer
    // overflows. This generally should not happen and indicates
    // that we are not reading from I2S fast enough.
    i2s_event_callbacks_t cbs = {
        .on_recv = NULL,
        .on_recv_q_ovf = i2s_rx_queue_overflow_callback,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(*i2sHandle, &cbs, NULL));
    return ESP_OK;
}



////////////////////////////////////////////////////////////////////
// setup_esp32_adc 
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
esp_err_t setup_esp32_adc(gptimer_handle_t* gpTimerHandle, const uint32_t sampleRate)
{

    // Set up sampling interrupt
    const uint32_t MICROSECONDS_PER_SAMPLE = (uint32_t)(1.0f / sampleRate * 1000 * 1000);
    const uint32_t CLK_RES = 1 * 1000 * 1000;         // 1 MHz
    const uint32_t CLK_TICKS_PER_MICROSECOND = CLK_RES / 1000 / 1000;

    gptimer_config_t gpTimerConfig = 
    {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = CLK_RES,               // 1 MHz
        .intr_priority = 3,
    };

    gptimer_alarm_config_t gpAlarmConfig = 
    {
        .alarm_count = MICROSECONDS_PER_SAMPLE * CLK_TICKS_PER_MICROSECOND,
        .reload_count = 0,
        .flags = 
        {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_event_callbacks_t gpTimerEventCallback =
    {
        .on_alarm = timer_isr_handler,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&gpTimerConfig, gpTimerHandle));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(*gpTimerHandle, &gpAlarmConfig));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*gpTimerHandle, &gpTimerEventCallback, NULL));
    ESP_ERROR_CHECK(gptimer_enable(*gpTimerHandle));

    // The interrupt is now ready, but the timer associated with the interrupt
    // has not started yet

    return ESP_OK;
}



////////////////////////////////////////////////////////////////////
// esp32_adc_collect_samples 
//
// Collect samples using the internal ESP32 ADC. Sample until
// both the sampling and transmit tasks are ready to exchange data.
////////////////////////////////////////////////////////////////////
esp_err_t esp32_adc_collect_samples(gptimer_handle_t* gpTimerHandle)
{
    // Start the timer, which fires at our desired sampling rate
    ESP_ERROR_CHECK(gptimer_start(*gpTimerHandle));

    // Keep sampling until both of the following is true 
    //    a) the transmit thread is done sending the active packet
    //    b) we've collected sufficient data to send in a packet
    while ((backgroundPacket->numSamples < 256) || ulTaskNotifyTakeIndexed(transmissionDoneNotifyIndex, pdTRUE, portMAX_DELAY))
    {
        PRINTF_DEBUG((__func__, "Waiting on transmission done\n"));
        vTaskDelay(1);
    }

    PRINTF_DEBUG((__func__, "Transmission Done!\n"));
    
    // Freeze the timer
    ESP_ERROR_CHECK(gptimer_stop(*gpTimerHandle));

    PRINTF_DEBUG((__func__, "Stopped timer\n"));

    return ESP_OK;
}



////////////////////////////////////////////////////////////////////
// external_adc_collect_samples 
//
// Collect samples using the PCM4201 ADC. Sample until both
// the sampling and transmit tasks are ready to exchange data.
////////////////////////////////////////////////////////////////////
esp_err_t external_adc_collect_samples(i2s_chan_handle_t* i2sHandle)
{

    // Read samples from the ADC in chunks. By reading in chunks,
    // we can poll intermittently for the transmit task's state.

    const size_t SAMPLES_PER_CHUNK = 20;
    const size_t BYTES_PER_CHUNK = SAMPLES_PER_CHUNK * PCM4201_BYTES_PER_SAMPLE; 
    const size_t ADC_BUFFER_CAPACITY = AUDIO_PACKET_MAX_SAMPLES * PCM4201_BYTES_PER_SAMPLE;

    // Reset the ADC buffer
    adcBuffer.start = 0;
    adcBuffer.size = 0;

    // Keep sampling until both of the following is true 
    //    a) the transmit task is ready to receive data
    //    b) we've collected sufficient data to send in a packet
    const size_t min_bytes_required = ADC_BUFFER_CAPACITY / 2;

    uint16_t totalOverflow = 0;

    while ((adcBuffer.size < min_bytes_required) || !ulTaskNotifyTakeIndexed(transmissionDoneNotifyIndex, pdTRUE, 0))
    {

        // Kick the watchdog timer
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_reset());

        // Since we are repeatedly reading data into the ADC buffer,
        // there is a chance the buffer will overflow. If an overflow
        // is about to happen on this loop iteration, drop the oldest
        // data in the buffer and replace it with a new chunk.
        // The chunk size is chosen to evenly divide the ADC buffer
        
        if (adcBuffer.size + BYTES_PER_CHUNK > ADC_BUFFER_CAPACITY)
        {
            uint16_t overflow = adcBuffer.size + BYTES_PER_CHUNK - ADC_BUFFER_CAPACITY;
            adcBuffer.start = (adcBuffer.start + overflow) % ADC_BUFFER_CAPACITY;
            adcBuffer.size -= overflow;
            totalOverflow += overflow;
        }

        // Compute where to write new data
        uint32_t write_pos = (adcBuffer.start + adcBuffer.size) % ADC_BUFFER_CAPACITY;

        size_t bytesRead = 0, totalBytesRead = 0;
        esp_err_t ret = ESP_OK;

        // Read data from the I2S DM
        do
        {
            uint8_t timeout_ms = 50;
            ret = i2s_channel_read(*i2sHandle, &adcBuffer.buffer[write_pos + totalBytesRead], BYTES_PER_CHUNK, &bytesRead, timeout_ms, &samplingTaskHandle);
            totalBytesRead += bytesRead;
        } while (ret == ESP_OK && totalBytesRead < BYTES_PER_CHUNK);
        
        // If errors occurred, print out what they were
        if (ret != ESP_OK || bytesRead != BYTES_PER_CHUNK)
        {
            ESP_LOGE(__func__, "Error during I2S read: (%u) %s (%u bytes read)!\n",
                ret, esp_err_to_name(ret), bytesRead);
        }

        if (ret != ESP_OK)
        {
            ESP_LOGE(__func__, "Error reading from I2S (errno=%u)\n", ret);
        }
        else
        {
            adcBuffer.size += BYTES_PER_CHUNK;
        }
    }

    // At this point, the transmit task is ready for data.
    // The data in the ADC buffer is 8-bytes per sample, with
    // 3 bytes of real data and 5 dummy bytes. When transferring
    // samples from the ADC buffer to the background packet, strip
    // off the dummy bytes.

    if (totalOverflow > 0)
    {
        PRINTF_DEBUG((__func__, "Overflow of %u bytes occurred on sampling task\n", totalOverflow));
            totalOverflow = 0;
    }

    uint32_t adcBufferNumSamples = adcBuffer.size / PCM4201_BYTES_PER_SAMPLE;

    for (int i = 0; i < adcBufferNumSamples; i++)
    {
        int sampleStart = (adcBuffer.start + i * PCM4201_BYTES_PER_SAMPLE) % ADC_BUFFER_CAPACITY;
        
        // Since adc buffer and background buffer have same sample-capacity
        // there is no risk that the background buffer will overflow
        uint32_t wrPos = (i * AUDIO_PACKET_BYTES_PER_SAMPLE);   

        memcpy(backgroundPacket->payload + wrPos, adcBuffer.buffer + sampleStart + 1, AUDIO_PACKET_BYTES_PER_SAMPLE);

        backgroundPacket->numSamples++;
    }

    backgroundPacket->payloadStart = 0;

    return ESP_OK;
}



////////////////////////////////////////////////////////////////////
// local_buffer_collect_samples()
//
////////////////////////////////////////////////////////////////////
esp_err_t local_buffer_collect_samples()
{

    // We have a buffer containing precomputed audio samples at a particular frequency
    // Store samples from that buffer in the backgroundPacket
    const int32_t* localBuffer = audio_table_500hz;
    const size_t localBufferNumSamples = audio_table_500hz_size;

    static const uint16_t numSamples = AUDIO_PACKET_MAX_SAMPLES;
    static const uint16_t waitTimeUs = 1.0f * numSamples / 48000 * 1000 * 1000;
    static uint16_t bufferIdx = 0;

    int64_t start = 0, stop = 0;
    get_system_time(&start);

    for (int i = 0; i < numSamples; i++)
    {
            // remove the sign-extension bits from the sample
            int32_t sample = (localBuffer[bufferIdx] << 8) >> 8;    

            bufferIdx = (bufferIdx + 1) % localBufferNumSamples;

            // the sample is stored little-endian and we have zero'd out the sign extension bits
            // store the remaining 3 bytes in the backgroundPacket.
            memcpy(backgroundPacket->payload + i * AUDIO_PACKET_BYTES_PER_SAMPLE, &sample, AUDIO_PACKET_BYTES_PER_SAMPLE);
    }

    backgroundPacket->payloadStart = 0;
    backgroundPacket->numSamples = numSamples;

    get_system_time(&stop);

    esp_rom_delay_us(waitTimeUs - (stop - start));

    while (!ulTaskNotifyTakeIndexed(transmissionDoneNotifyIndex, pdTRUE, 0))
    {
        esp_task_wdt_reset();
    }
        
    return ESP_OK;
}



////////////////////////////////////////////////////////////////////
// sampling_task_main
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
void sampling_task_main(void* pvParameters)
{
    SamplingTaskConfig_t* config = (SamplingTaskConfig_t*)pvParameters;

    // Sign up for the task watchdog. This will be useful for knowing whether this
    // task is blocked for extended periods without us knowing about it.
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    // Wait for the transmit task to come up
    while (transmitTaskHandle == NULL)
    {
        ESP_LOGI(__func__, "Waiting for transmit task to be created...\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // Set up the internal or external ADC
    gptimer_handle_t gpTimerHandle = NULL;
    i2s_chan_handle_t i2sHandle    = NULL;
    if (config->audioSource == AUDIO_SOURCE_EXTERNAL_ADC)
    {
        ESP_LOGI(__func__, "Using external adc for audio samples\n");

        ESP_ERROR_CHECK(setup_external_adc(&i2sHandle, config->sampleRate));
        assert(i2sHandle);

        // Set up DMA buffer overflow callback
        dmaBufferOverflowQueue = xQueueCreate(5, sizeof(size_t));
        assert(dmaBufferOverflowQueue);

        // Start the i2s receiver
        ESP_ERROR_CHECK(i2s_channel_enable(i2sHandle));
    }
    else if (config->audioSource == AUDIO_SOURCE_INTERNAL_ADC)
    {
        ESP_LOGI(__func__, "Using internal esp32 adc for audio samples\n");
        ESP_ERROR_CHECK(setup_esp32_adc(&gpTimerHandle, config->sampleRate));
        assert(gpTimerHandle);
    }
    else    // AUDIO_SOURCE_BUFFER
    {
        ESP_LOGI(__func__, "Using local buffer for audio samples\n");
    }

    // In an infinite loop, collect samples from ADC and transfer them to the transmit task  
    while (true)
    {

        // Kick the watchdog timer
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_reset());

        // Collect samples
        if (config->audioSource == AUDIO_SOURCE_EXTERNAL_ADC)
        {
            ESP_ERROR_CHECK(external_adc_collect_samples(&i2sHandle));
        }
        else if (config->audioSource == AUDIO_SOURCE_INTERNAL_ADC)
        {
            ESP_ERROR_CHECK(esp32_adc_collect_samples(&gpTimerHandle));
        }
        else    // AUDIO_SOURCE_BUFFER
        {
            ESP_ERROR_CHECK(local_buffer_collect_samples());
        }

        // At this point, the transmit thread is waiting for a data ready signal and is
        // therefore not sending any data. It is safe to swap the active and background
        // packets.
        AudioPacket_t * tmp = activePacket;
        activePacket = backgroundPacket;
        backgroundPacket = tmp;
        backgroundPacket->seqnum = activePacket->seqnum + 1;

        PRINTF_DEBUG((__func__, "Notifying transmission task that new data is available (%u)\n",
            activePacket->numSamples));
        xTaskNotifyGiveIndexed(transmitTaskHandle, dataReadyNotifyIndex);

        backgroundPacket->checksum = 0;
        backgroundPacket->numSamples = 0;
        backgroundPacket->payloadStart = 0;

        // Check if the I2S DMA buffer overflowed. This can produce
        // noticeable artifacts in the audio.
        if (dmaBufferOverflowQueue && uxQueueMessagesWaiting(dmaBufferOverflowQueue))
        {
            size_t tmp;
            xQueueReceive(dmaBufferOverflowQueue, &tmp, 0);

            static int64_t prevTime = 0;
            static int64_t currTime = 0;
            get_system_time(&currTime);

            static uint8_t numOverflows = 0;
            numOverflows++;

            if (currTime - prevTime > 1e6)
            {
                ESP_LOGE(__func__, "I2S DMA buffer overflowed %u times!\n", numOverflows);
                prevTime = currTime;
                numOverflows = 0;
            }

            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_reset());
        }
    }

    // We should never reach here. The sampling task should run forever.
    ESP_LOGE(__func__, "Terminating sampling task");
    vTaskDelete(NULL);
}

