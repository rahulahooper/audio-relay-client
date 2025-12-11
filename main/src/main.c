#include "nvs_flash.h"

#include "common.h"
#include "transmit.h"
#include "sample.h"
#include "util.h"

// -------- Local definitions and macros -------- //

#define ESP_CORE_0  0           // physical core 0
#define ESP_CORE_1  1           // physical core 1

AudioPacket_t * activePacket;                           // the transmit task transmits this packet
AudioPacket_t * backgroundPacket;                       // the sampling task fills this packet

const UBaseType_t transmissionDoneNotifyIndex = 0;      // set by the transmit task to signal that it is done transmitting data
const UBaseType_t dataReadyNotifyIndex = 0;             // set by the sampling thread to signal that there is new data to transmit

TaskHandle_t samplingTaskHandle = NULL;                  // FIXME: If these task handles are declared
TaskHandle_t transmitTaskHandle = NULL;                  // before the adcBuffer, we get memory corruption in the BSS
                                                         // segment of the ESP32 executable. I've traced it back to 
                                                         //  some kind of buffer overflow in i2s_channel_read()

void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    activePacket = (AudioPacket_t*)malloc(sizeof(AudioPacket_t)); 
    backgroundPacket = (AudioPacket_t*)malloc(sizeof(AudioPacket_t)); 

    memset(activePacket, 0, sizeof(AudioPacket_t));
    memset(backgroundPacket, 0, sizeof(AudioPacket_t));

    /// Create transmit and sampling tasks
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
    static SamplingTaskConfig_t samplingTaskConfig =
    {
        .sampleRate  = 48000,
        .audioSource = AUDIO_SOURCE_EXTERNAL_ADC,
    };

    BaseType_t status;
    status = xTaskCreatePinnedToCore(sampling_task_main, "sampling_task", 8192, (void*)&samplingTaskConfig, 5, &samplingTaskHandle, ESP_CORE_0);

    if (status != pdPASS)
    {
        ESP_LOGE(__func__, "Failed to create sampling task!\n");
        return;
    }

    status = xTaskCreatePinnedToCore(transmit_task_main, "transmit_task", 8192, NULL, 5, &transmitTaskHandle, ESP_CORE_1);

    if (status != pdPASS)
    {
        ESP_LOGE(__func__, "Failed to create transmit task!\n");
        return;
    }

}
