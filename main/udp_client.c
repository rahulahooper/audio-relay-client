/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h" 
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/gptimer.h"

// -------- Local definitions and macros -------- //

#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

#define AUDIO_NET_WIFI_SSID      "AudioRelayNetwork"
#define AUDIO_NET_WIFI_PASS      "AudioRelayNetworkPassword" 
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK

#define ESP_CORE_0  0           // physical core 0
#define ESP_CORE_1  1           // physical core 1

#define AUDIO_PACKET_MAX_SAMPLES 250        // maximum number of audio samples we can receive from client at a time 
#define AUDIO_PACKET_BYTES_PER_SAMPLE 3     // size of each sample within an audio packet in bytes
#define MAX_SEND_ATTEMPTS 3                 // maximum attempts to send a single audio packet

#define min(a,b) ((a) < (b) ? (a) : (b))

#define DEBUG 0

#if DEBUG
    #define PRINTF_DEBUG( msg ) ESP_LOGI msg
#else
    #define PRINTF_DEBUG( msg )
#endif

typedef struct AudioPacket_t
{
    uint16_t seqnum;
    bool     echo;
    uint16_t numSamples;
    uint16_t payloadStart;              // start index of audio data in <payload>
    uint16_t checksum;
    uint8_t  payload[AUDIO_PACKET_MAX_SAMPLES * AUDIO_PACKET_BYTES_PER_SAMPLE];
} AudioPacket_t;

typedef enum TransmitTaskState_t
{
    XMIT_TASK_STATE_CONNECT_TO_NETWORK,
    XMIT_TASK_STATE_CREATE_SOCKET,
    XMIT_TASK_STATE_STREAM_TO_SERVER,
    XMIT_TASK_STATE_DESTROY_SOCKET,
    XMIT_TASK_STATE_NETWORK_DISCONNECT,
} TransmitTaskState_t;

static TransmitTaskState_t transmitTaskState;

typedef struct SamplingTaskConfig_t
{
    uint32_t sampleRate;
    bool useExternalAdc;
} SamplingTaskConfig_t;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

// Set this bit when we connect to a Wifi network
#define WIFI_CONNECTED_BIT BIT0

static const char *TAG = "wifi station";
static int gWifiConnectAttempts = 0;

static AudioPacket_t gAudioPackets[2];
static AudioPacket_t * activePacket;   // transmitting task transmits this packet
static AudioPacket_t * backgroundPacket; // sampling task fills this packet

static const UBaseType_t transmissionDoneNotifyIndex = 0;      // set by the transmission thread when it is done transmitting data
static const UBaseType_t dataReadyNotifyIndex;                 // set by the sampling thread when there is new data to transmit

static TaskHandle_t samplingTaskHandle = NULL;
static TaskHandle_t transmitTaskHandle = NULL;

#define PCM4201_BYTES_PER_SAMPLE 8
typedef struct ExternalAdcCircularBuffer_t
{
    uint16_t      start;
    uint16_t      size;
    uint8_t       buffer[AUDIO_PACKET_MAX_SAMPLES * PCM4201_BYTES_PER_SAMPLE];
} ExternalAdcCircularBuffer_t;

static ExternalAdcCircularBuffer_t adcBuffer;

////////////////////////////////////////////////////////////////////
// crc16()
//
// Compute a crc16 using polynomial 0x1021 and seed value 0xFFFF
// (see https://en.wikipedia.org/wiki/Cyclic_redundancy_check)
////////////////////////////////////////////////////////////////////
uint16_t crc16(uint8_t* data, uint32_t len)
{
    const uint16_t seed       = 0xFFFF;
    const uint16_t polynomial = 0x1021;

    uint16_t crc = seed;

    for (int i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        for (int j = 0; j < 8; j++)
        {
            if ((byte & 0x80) != 0)
            {
                crc ^= (polynomial << 8);
            }

            crc = (crc << 1) & 0xFFFF;
            byte <<= 1;
        }
    }

    return crc;
}


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
// wifi_event_handler()
//
// Callback function for handling wifi connect / disconnect events
////////////////////////////////////////////////////////////////////
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        gWifiConnectAttempts++;
        ESP_LOGI(TAG, "%s Failed to connect to the AP, retrying (attempt #%u)\n", __func__, gWifiConnectAttempts);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        gWifiConnectAttempts = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


////////////////////////////////////////////////////////////////////
// wifi_setup_driver()
//
// Allocates resources for WiFi driver such as WiFi control
// structures, RX/TX buffers, NVS, etc. This function should
// only be called once at system startup. Call 
// wifi_deinit_driver() on shutdown.
////////////////////////////////////////////////////////////////////
esp_err_t wifi_setup_driver(wifi_init_config_t* cfg)
{
    ESP_LOGI(TAG, "%s: Setting up WiFi driver\n", __func__);        
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t default_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&default_cfg));

    memcpy(cfg, &default_cfg, sizeof(default_cfg));

    return ESP_OK;
}


////////////////////////////////////////////////////////////////////
// wifi_deinit_driver()
//
// Call on shutdown. WiFi operations will not be supported
// after calling this function.
////////////////////////////////////////////////////////////////////
esp_err_t wifi_deinit_driver()
{
    ESP_ERROR_CHECK(esp_wifi_deinit());
    return ESP_OK;
}

////////////////////////////////////////////////////////////////////
// wifi_station_start_and_connect()
//
////////////////////////////////////////////////////////////////////
void wifi_station_start_and_connect(wifi_config_t* wifi_config, bool* error)
{
    *error = true;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "%s Started WiFi!\n", __func__);

    // Wait until either the connection is established (WIFI_CONNECTED_BIT) or 
    // connection failed after the maximum number of re-tries (WIFI_FAIL_BIT). 
    // The bits are set by wifi_event_handler() (see above)
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    
    const char* ssid = (const char*)(wifi_config->sta.ssid);
    const char* password = (const char*)(wifi_config->sta.password);
    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(TAG, "%s: connected to ap SSID:%s password:%s",
                __func__, ssid, password);
        *error = false;
    } 
    else 
    {
        ESP_LOGE(TAG, "%s: UNEXPECTED EVENT: %lx\n",
                __func__, bits);
    }
}


////////////////////////////////////////////////////////////////////
// wifi_station_disconnect_and_stop()
//
// This function will disconnect from whatever network we are
// currently on.
////////////////////////////////////////////////////////////////////
void wifi_station_disconnect_and_stop(bool* error)
{
    ESP_LOGI(TAG, "%s: Disconnecting from home WiFi network\n", __func__);
    *error = true;

    esp_err_t ret = esp_wifi_disconnect();
    if (ret == ESP_ERR_WIFI_NOT_STARTED)
    {
        ESP_LOGI(TAG, "%s ESP32 Wifi station wasn't started wifi_station_start_and_connect()\n", __func__);
        *error = false;
        return;
    }
    else if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGI(TAG, "%s ESP32 Wifi resources were not initialized\n", __func__);
        *error = false;
        return;
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s Unexpected errno: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_wifi_stop();

    if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGE(TAG, "%s ESP32 Wifi resouces were not initialized\n", __func__);
        return;
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s Unexpected errno: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    *error = false;
}


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
// setup_external_dac 
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
    ESP_LOGI(__func__, "sample rate: %lu\n", sampleRate);
    i2s_std_config_t i2sStdConfig = 
    {
       .clk_cfg = 
       {
           .sample_rate_hz = sampleRate,
           .clk_src = I2S_CLK_SRC_DEFAULT,
           .mclk_multiple = I2S_MCLK_MULTIPLE_256
       },
       .slot_cfg =
       {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,

            .ws_width       = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol         = true,           // need to double-check this, "true" seems to mean left channel on WS high
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
    ESP_ERROR_CHECK(i2s_channel_enable(*i2sHandle));
    
    // The I2S receiver is now running and receiving data from the PCM4201

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
        PRINTF_DEBUG((TAG, "%s Waiting on transmission done\n", __func__));
        vTaskDelay(1);
    }

    PRINTF_DEBUG((TAG, "%s Transmission Done!\n", __func__));
    
    // Freeze the timer
    ESP_ERROR_CHECK(gptimer_stop(*gpTimerHandle));

    PRINTF_DEBUG((TAG, "%s Stopped timer\n", __func__));

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

    // TODO: Right now, we store the chunks into an ADC buffer; when
    // the transmit task is ready for new data, we transfer the
    // contents of the ADC buffer to the back buffer, removing
    // padded bytes in the process. In the future, we should
    // just sample direclty into the back buffer to save memory.
    const size_t SAMPLES_PER_CHUNK = 10;
    const size_t BYTES_PER_CHUNK = SAMPLES_PER_CHUNK * PCM4201_BYTES_PER_SAMPLE; 
    const size_t ADC_BUFFER_CAPACITY = AUDIO_PACKET_MAX_SAMPLES * PCM4201_BYTES_PER_SAMPLE;

    // Reset the ADC buffer
    adcBuffer.start = 0;
    adcBuffer.size = 0;

    // Keep sampling until both of the following is true 
    //    a) the transmit task is done sending the active packet
    //    b) we've collected sufficient data to send in a packet
    size_t min_samples_required = AUDIO_PACKET_MAX_SAMPLES / 5;
    size_t min_bytes_required = min_samples_required * PCM4201_BYTES_PER_SAMPLE;

    uint s = 0;
    while ((adcBuffer.size < min_bytes_required) || !ulTaskNotifyTakeIndexed(transmissionDoneNotifyIndex, pdTRUE, portMAX_DELAY))
    {

        // Since we are repeatedly reading data into the ADC buffer,
        // there is a chance the buffer will overflow. If an overflow
        // is about to happen on this loop iteration, drop the oldest
        // data in the buffer and replace it with a new chunk.
        // The chunk size is chosen to evenly divide the ADC buffer
        
        // Overflow check: drop the oldest chunk if necessary
        if (adcBuffer.size + BYTES_PER_CHUNK > ADC_BUFFER_CAPACITY)
        {
            uint overflow = adcBuffer.size + BYTES_PER_CHUNK - ADC_BUFFER_CAPACITY;
            adcBuffer.start = (adcBuffer.start + overflow) % ADC_BUFFER_CAPACITY;
            adcBuffer.size -= overflow;
        }

        // Compute where to write new data
        uint write_pos = (adcBuffer.start + adcBuffer.size) % ADC_BUFFER_CAPACITY;

        esp_err_t ret = i2s_channel_read(*i2sHandle, &adcBuffer.buffer[write_pos], BYTES_PER_CHUNK, NULL, 1);
        for (int i = 0; i < SAMPLES_PER_CHUNK; i++)
        {
            ret = ESP_OK;
            for (int j = 0; j < AUDIO_PACKET_BYTES_PER_SAMPLE; j++)
            {
                adcBuffer.buffer[write_pos + i * PCM4201_BYTES_PER_SAMPLE + 1 + j] = s;
            }
            s++;
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

    // At this point, the transmit task is ready for data
    //
    // The data in the ADC buffer is 8-bytes per sample, with
    // 3 bytes of real data and 5 dummy bytes. When transferring
    // samples from the ADC buffer to the background packet, strip
    // off the dummy bytes.

    uint32_t adcBufferNumSamples = adcBuffer.size / PCM4201_BYTES_PER_SAMPLE;

    // for (int i = 0; i < adcBuffer.size; i++)
    // {
    //     ESP_LOGI(__func__, "[%u] = %x", i, adcBuffer.buffer[i]);
    // }

    for (int i = 0; i < adcBufferNumSamples; i++)
    {
        int sampleStart = (adcBuffer.start + i * PCM4201_BYTES_PER_SAMPLE) % ADC_BUFFER_CAPACITY;
        
        // since adc buffer and background buffer have same sample-capacity
        // there is no risk that the background buffer will overflow
        uint wrPos = (i * AUDIO_PACKET_BYTES_PER_SAMPLE);   

        memcpy(backgroundPacket->payload + wrPos, adcBuffer.buffer + sampleStart + 1, AUDIO_PACKET_BYTES_PER_SAMPLE);

        backgroundPacket->numSamples++;
    }

    backgroundPacket->payloadStart = 0;

    // for (int i = 0; i < backgroundPacket->numSamples; i++)
    // {
    //     ESP_LOGI(__func__, "[%u]: %2x%2x%2x", i,
    //         backgroundPacket->payload[i * AUDIO_PACKET_BYTES_PER_SAMPLE],
    //         backgroundPacket->payload[i * AUDIO_PACKET_BYTES_PER_SAMPLE+1],
    //         backgroundPacket->payload[i * AUDIO_PACKET_BYTES_PER_SAMPLE+2]);
    // }
    return ESP_OK;
}

////////////////////////////////////////////////////////////////////
// sampling_task 
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
static void sampling_task(void* pvParameters)
{
    SamplingTaskConfig_t* config = (SamplingTaskConfig_t*)pvParameters;
    ESP_LOGI(__func__, "[1] sampleRate: %lu\n", config->sampleRate);
    // Wait for the transmit task to come up
    // while (transmitTaskHandle == NULL)
    // {
    //     ESP_LOGI(TAG, "%s: Waiting for transmit task to be created...\n", __func__);
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }

    // Set up the internal or external ADC
    gptimer_handle_t gpTimerHandle = NULL;
    i2s_chan_handle_t i2sHandle    = NULL;
    if (config->useExternalAdc)
    {
        ESP_LOGI(__func__, "[2] sampleRate: %lu\n", config->sampleRate);
        ESP_ERROR_CHECK(setup_external_adc(&i2sHandle, config->sampleRate));
        assert(i2sHandle);
    }
    else
    {
        ESP_ERROR_CHECK(setup_esp32_adc(&gpTimerHandle, config->sampleRate));
        assert(gpTimerHandle);

        // adcBuffer = malloc(sizeof(ExternalAdcCircularBuffer_t));
        // assert(adcBuffer);
    }

    // In an infinite loop, collect samples from ADC and transfer them to the transmit task  
    while (true)
    {

        if (config->useExternalAdc)
        {
            ESP_ERROR_CHECK(external_adc_collect_samples(&i2sHandle));
        }
        else
        {
            ESP_ERROR_CHECK(esp32_adc_collect_samples(&gpTimerHandle));
        }

        // At this point, the transmit thread is waiting for a data ready signal and is
        // therefore not sending any data. It is safe to swap the active and background
        // packets.
        AudioPacket_t * tmp = activePacket;
        activePacket = backgroundPacket;
        backgroundPacket = tmp;
        backgroundPacket->seqnum = activePacket->seqnum + 1;

        PRINTF_DEBUG((TAG, "%s Notifying transmission task that new data is available\n", __func__));
        xTaskNotifyGiveIndexed(transmitTaskHandle, dataReadyNotifyIndex);

        backgroundPacket->checksum = 0;
        backgroundPacket->numSamples = 0;
        backgroundPacket->payloadStart = 0;;
    }

    // We should never reach here. The sampling task should run forever.
    ESP_LOGE(TAG, "%s Terminating sampling task", __func__);
    vTaskDelete(NULL);
}


////////////////////////////////////////////////////////////////////
// relay_network_connect 
//
// Connect to the Audio Relay Network server
////////////////////////////////////////////////////////////////////
void relay_network_connect(bool * error)
{
    ESP_LOGI(TAG, "%s: Connecting to Audio Relay Wifi Network\n", __func__);

    wifi_config_t audio_wifi_config = {
        .sta = {
            .ssid = AUDIO_NET_WIFI_SSID,
            .password = AUDIO_NET_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    wifi_station_start_and_connect(&audio_wifi_config, error);
}

////////////////////////////////////////////////////////////////////
// create_socket() 
//
////////////////////////////////////////////////////////////////////
void create_socket(bool* error, int* sock, struct sockaddr_in* dest_addr)
{
    *error = true;

    // Get IP address of the relay network server
    esp_netif_t* esp_netif = esp_netif_get_default_netif();
    esp_netif_ip_info_t* ip_info = malloc(sizeof(esp_netif_ip_info_t));
    if (!esp_netif || !ip_info)
    {
        ESP_LOGE(TAG, "%s Got a nullptr: esp_netif=%p, ip_info=%p\n", __func__, esp_netif, ip_info);
        vTaskDelete(NULL);
        return;
    }

    ESP_ERROR_CHECK( esp_netif_get_ip_info(esp_netif, ip_info) );
    struct in_addr ipv4_addr;
    char ipv4_str[INET_ADDRSTRLEN];
    ipv4_addr.s_addr = ip_info->gw.addr;
    if (inet_ntop(AF_INET, &ipv4_addr, ipv4_str, INET_ADDRSTRLEN) != NULL)
    {
        ESP_LOGI(TAG, "Gateway IPv4: %s", ipv4_str);
    }

    // Fill in IP address in return object
    dest_addr->sin_addr.s_addr = inet_addr(ipv4_str);
    dest_addr->sin_family = AF_INET;
    dest_addr->sin_port = htons(PORT);

    // Set up socket connection with server
    int addr_family = 0;
    int ip_protocol = 0;

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    
    *sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (*sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    // Set timeout for receiving echoes from server
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(*sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
    *error = false;
}


////////////////////////////////////////////////////////////////////
// stream_audio_to_server 
//
////////////////////////////////////////////////////////////////////
void stream_audio_to_server(bool* error, const int sock, const struct sockaddr_in dest_addr)
{
    *error = true;

    if (sock < 0)
    {
        ESP_LOGI(TAG, "%s Socket was invalid\n", __func__);
        return;
    }

    int64_t timesent, timerecv;

    // Wait for sampling task to come up
    while (samplingTaskHandle == NULL)
    {
        ESP_LOGI(TAG, "%s Waiting for sampling task to be created...\n", __func__);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // Init some statistics-related variables
    uint32_t bytesSinceLastEcho = 0;
    uint32_t packetsSinceLastEcho = 0;
    int64_t timeOfLastEcho;
    get_system_time(&timeOfLastEcho);

    // Stream audio data to server
    while (1) {

        // Indicate to sampling thread that there is no data to transmit
        PRINTF_DEBUG((TAG, "%s Notifying of transmission done.\n", __func__));
        xTaskNotifyGiveIndexed(samplingTaskHandle, transmissionDoneNotifyIndex);

        int64_t timenow;
        get_system_time(&timenow);

        // Wait for the sampling task to indicate that there is new data available
        // (Ideally, this should not block at all because we just blocked earlier)
        PRINTF_DEBUG((TAG, "%s Waiting for data ready.\n", __func__));
        if(ulTaskNotifyTakeIndexed(dataReadyNotifyIndex, pdTRUE, pdMS_TO_TICKS(1000)))
        {
            PRINTF_DEBUG((TAG, "%s Got data ready notification.\n", __func__));
        }
        else
        {
            ESP_LOGE(TAG, "%s Timed out waiting for data ready notification\n", __func__);
            continue;
        }

        // CRC, timestamp, and transmit the audio packet
        activePacket->checksum = crc16(activePacket->payload, activePacket->numSamples * AUDIO_PACKET_BYTES_PER_SAMPLE);
    
        activePacket->echo = !(activePacket->seqnum % 500);      // request an echo from the server 
        uint32_t packetSize = sizeof(AudioPacket_t);

        get_system_time(&timesent);

        for (int i = 0; i < MAX_SEND_ATTEMPTS; i++)
        {
            PRINTF_DEBUG((TAG, "%s Sending packet with seqnum %u, payload len %u, CRC 0x%x to server. Total size = %lu\n",
                __func__, activePacket->seqnum, activePacket->payloadSize, activePacket->checksum, packetSize));

            int err = sendto(sock, activePacket, packetSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            if (err < 0) {
                if (errno == ENOMEM)
                {
                    PRINTF_DEBUG((TAG, "%s Failed to send packet w/seqnum %u over socket. Error is likely full send buffer. Retrying (%u/%u)\n",
                        __func__, activePacket->seqnum, i+1, MAX_SEND_ATTEMPTS));
                    vTaskDelay(1);
                    continue;
                }

                ESP_LOGE(TAG, "%s Error sending packet %u: errno %u (%s)", __func__, activePacket->seqnum, errno, strerror(errno));

                // Check connection with server and exit if we are disconnected
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_ERR_WIFI_NOT_CONNECT)
                {
                    ESP_LOGE(TAG, "%s Disconnected from server. Exiting transmit loop\n", __func__);
                    return;
                }
            }
            else
            {
                break;
            }
        }

        // Update packet statistics
        bytesSinceLastEcho += activePacket->numSamples * AUDIO_PACKET_BYTES_PER_SAMPLE;
        packetsSinceLastEcho++;

        // Optionally receive echo'd packet from server (useful to measuring Wifi speeds)
        // TODO: Make this non-blocking and wait a couple of packets before
        // timing out. This timeout can signify a network disconnection.
        if (activePacket->echo)
        {
            struct AudioPacket_t response;
            struct sockaddr_storage source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, &response, sizeof(response), 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) 
            {
                ESP_LOGE(TAG, "%s Failed to receive echo'd packet %u from server (errno = %s)", __func__, activePacket->seqnum, strerror(errno));
            }
            else 
            {
                get_system_time(&timerecv);
                ESP_LOGI(TAG, "%s Received %d bytes. Seqnum = %u, payload size = %u\n", __func__, len, response.seqnum, response.numSamples);
                ESP_LOGI(TAG, "%s Round trip time: %lld\n", __func__, timerecv - timesent);
            }

            // Report transmit statistics
            uint32_t avgPacketSize = bytesSinceLastEcho / packetsSinceLastEcho;
            float avgThroughput = 1.0f * bytesSinceLastEcho / (timerecv - timeOfLastEcho) * 1000 * 1000;

            ESP_LOGI(TAG, "%s Average packet size = %lu, average throughput (bytes/sec) = %f\n", 
                __func__, avgPacketSize, avgThroughput);

            timeOfLastEcho = timerecv;
            bytesSinceLastEcho = 0;
            packetsSinceLastEcho = 0;
        }
    }
}


////////////////////////////////////////////////////////////////////
// transmit_task 
//
// Main event loop for streaming data to server.
////////////////////////////////////////////////////////////////////
static void transmit_task(void *pvParameters)
{

    transmitTaskState = XMIT_TASK_STATE_CONNECT_TO_NETWORK;


    while (1) {

        bool error;
        static int socket = -1;
        struct sockaddr_in dest_addr;   // stores IP information about server

        switch (transmitTaskState)
        {
            case XMIT_TASK_STATE_CONNECT_TO_NETWORK:
            {
                relay_network_connect(&error);

                if (error)
                {
                    ESP_LOGI(TAG, "%s Failed to connect to relay network. Retrying...\n", __func__);
                    transmitTaskState = XMIT_TASK_STATE_NETWORK_DISCONNECT;
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    transmitTaskState = XMIT_TASK_STATE_CREATE_SOCKET;
                }
                break;
            }
            case XMIT_TASK_STATE_CREATE_SOCKET:
            {
                create_socket(&error, &socket, &dest_addr);

                if (error)
                {
                    ESP_LOGE(TAG, "%s Failed to create socket. Disconnecting from network and retrying...\n", __func__);
                    transmitTaskState = XMIT_TASK_STATE_DESTROY_SOCKET;
                }
                else
                {
                    ESP_LOGI(TAG, "%s transitioning to STREAM_TO_SERVER, sock = %d\n", __func__, socket);
                    transmitTaskState = XMIT_TASK_STATE_STREAM_TO_SERVER;
                }
                break;
            }
            case XMIT_TASK_STATE_STREAM_TO_SERVER:
            {
                // This function should ideally never return
                stream_audio_to_server(&error, socket, dest_addr);
                transmitTaskState = XMIT_TASK_STATE_DESTROY_SOCKET;
                break;
            }
            case XMIT_TASK_STATE_DESTROY_SOCKET:
            {
                if((close(socket) < 0) && (errno == EINTR))
                {
                    // operation was interrupted by signal, retry
                    break;
                }
                else
                {
                    socket = -1;
                    transmitTaskState = XMIT_TASK_STATE_NETWORK_DISCONNECT;
                    break;
                }
            }
            case XMIT_TASK_STATE_NETWORK_DISCONNECT:
            {
                wifi_station_disconnect_and_stop(&error);
                transmitTaskState = XMIT_TASK_STATE_CONNECT_TO_NETWORK;
                break;
            }
            default:
                ESP_LOGE(TAG, "%s, Invalid state %u\n", __func__, transmitTaskState);
                break;
        }
    }
    
    ESP_LOGE(TAG, "%s Terminating transmit task\n", __func__);
    vTaskDelete(NULL);
}


void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    #if CONFIG_ESP_WIFI_AUTH_WPA3_PSK             // use this one
    ESP_LOGI(TAG, "WPA3_PSK selected!");
    #endif

    uint32_t num_cores = configNUM_CORES;
    ESP_LOGI(TAG, "%s Number of cores: %lu\n", __func__, num_cores);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "%s Setting up WiFi driver", __func__);
    wifi_init_config_t cfg;
    ESP_ERROR_CHECK(wifi_setup_driver(&cfg));

    activePacket = &gAudioPackets[0];
    backgroundPacket = &gAudioPackets[1];
    memset(activePacket, 0, sizeof(AudioPacket_t));
    memset(backgroundPacket, 0, sizeof(AudioPacket_t));

    /// Create transmit and sampling tasks
    // 
    // The sampling task is responsible for sampling audio
    // data from the guitar and loading this data into a background
    // packet. This background packet that is invisible to the transmit 
    // task. When the transmit tasks signals that it is waiting for new
    // data to send over the network, the sampling task will swap the
    // background and active packets. This task lives in an infinite loop; 
    // it ideally never returns.
    //
    // The transmit task is responsible for transmitting audio
    // data to a server (which in this case is another ESP32). The
    // task reads data out of an active packet. When it is done 
    // transmitting the active buffer, the transmit task will notify
    // the sampling task. The sampling task then swaps the background 
    // and active buffers and notifies the transmit task that new data 
    // is available. The task also sets up the connection with the 
    // server.
    //
    ESP_LOGI(TAG, "%s Creating tasks\n", __func__); 
    BaseType_t status;
   
    SamplingTaskConfig_t samplingTaskConfig =
    {
        .sampleRate = 48000,
        .useExternalAdc = true
    };

    status = xTaskCreatePinnedToCore(sampling_task, "sampling_task", 8192, (void*)&samplingTaskConfig, 5, &samplingTaskHandle, ESP_CORE_0);

    if (status != pdPASS)
    {
        ESP_LOGE(TAG, "%s Failed to create sampling task!\n", __func__);
        return;
    }
    
    status = xTaskCreatePinnedToCore(transmit_task, "transmit_task", 8192, NULL, 5, &transmitTaskHandle, ESP_CORE_1);

    if (status != pdPASS)
    {
        ESP_LOGE(TAG, "%s Failed to create transmit task!\n", __func__);
        return;
    }

}
