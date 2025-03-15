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

#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

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

#define ESP_CORE_0  0       // physical core 0
#define ESP_CORE_1  1       // physical core 1
#define PAYLOAD_MAX_LEN 996

#define min(a,b) ((a) < (b) ? (a) : (b))

typedef struct AudioPacket_t
{
    uint16_t seqnum;
    bool     echo;
    uint16_t checksum;
    uint16_t payloadSize;
    uint8_t  payload[PAYLOAD_MAX_LEN];
} AudioPacket_t;

typedef enum TransmitTaskState_t
{
    RELAY_NETWORK_CONNECT,
    STREAM_TO_SERVER,
    NETWORK_DISCONNECT,
} TransmitTaskState_t;

static TransmitTaskState_t transmitTaskState;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";
static int gWifiConnectAttempts = 0;

static AudioPacket_t gAudioPackets[2];
static AudioPacket_t * activePacket;   // transmitting task transmits this packet
static AudioPacket_t * backgroundPacket; // sampling task fills this packet

static const UBaseType_t transmissionDoneNotifyIndex = 0;      // set by the transmission thread when it is done transmitting data
static const UBaseType_t dataReadyNotifyIndex;                 // set by the sampling thread when there is new data to transmit

static TaskHandle_t samplingTaskHandle = NULL;
static TaskHandle_t transmitTaskHandle = NULL;


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
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (gWifiConnectAttempts < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            gWifiConnectAttempts++;
            ESP_LOGI(TAG, "%s Failed to connect to the AP, retrying (attempt #%u)\n", __func__, gWifiConnectAttempts);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            gWifiConnectAttempts = 0;
        }
        ESP_LOGI(TAG,"connect to the AP fail");
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
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
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
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGI(TAG, "%s: Failed to connect to SSID:%s, password:%s",
                __func__, ssid, password);
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
// sampling_task 
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
void sampling_task()
{
    const uint32_t SAMPLE_RATE = 44100; // hz
    const uint32_t SAMPLE_PERIOD_MS = 10; //(uint32_t)(1.0f / SAMPLE_RATE / 1000);

    while (transmitTaskHandle == NULL)
    {
        ESP_LOGI(TAG, "%s: Waiting for transmit task to be created...\n", __func__);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    uint16_t seqnum = 0;

    while (true)
    {

        vTaskDelay(10 / portTICK_PERIOD_MS);
        backgroundPacket->seqnum = ++seqnum;
        backgroundPacket->checksum = 0;
        backgroundPacket->payloadSize = 0;

        uint32_t i = 0;

        // keep sampling until the transmit thread is done sending the active packet
        while (!ulTaskNotifyTakeIndexed(transmissionDoneNotifyIndex, pdTRUE, 0))
        {
            backgroundPacket->payload[i] = (uint8_t)(random() & 0xFF);
            backgroundPacket->payloadSize = min(backgroundPacket->payloadSize+1, PAYLOAD_MAX_LEN);
            i = (i+1) % PAYLOAD_MAX_LEN;

            // sampling should probably be interrupt driven to allow for precise timing.
            // this thread can just block until a sample is ready / all samples are ready.
            // for now just sleep
            vTaskDelay(SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "%s Transmission Done!\n", __func__);
        
        // At this point, the transmit thread is waiting for a data ready signal and is
        // therefore not sending any data. It is safe to swap the active and background
        // packets.
        AudioPacket_t * tmp = activePacket;
        activePacket = backgroundPacket;
        backgroundPacket = tmp;

        ESP_LOGI(TAG, "%s Notifying transmission task that new data is available\n", __func__);
        xTaskNotifyGiveIndexed(transmitTaskHandle, dataReadyNotifyIndex);

    }

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
// stream_audio_to_server 
//
////////////////////////////////////////////////////////////////////
void stream_audio_to_server(bool* error)
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

    // Set up socket connection with server
    int addr_family = 0;
    int ip_protocol = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(ipv4_str);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    // TODO: See if this helps xmit time at all
    // bool dont_route = 1;
    // setsockopt(sock, SOL_SOCKET, SO_DONTROUTE, &dont_route, sizeof(dont_route));

    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    int64_t timesent, timerecv;

    // Wait for sampling task to come up
    while (samplingTaskHandle == NULL)
    {
        ESP_LOGI(TAG, "%s Waiting for sampling task to be created...\n", __func__);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // Stream audio data to server
    while (1) {

        // Indicate to sampling thread that there is not data to transmit
        ESP_LOGI(TAG, "%s Notifying of transmission done.\n", __func__);
        xTaskNotifyGiveIndexed(samplingTaskHandle, transmissionDoneNotifyIndex);

        // Wait for the sampling task to indicate that there is new data available
        ESP_LOGI(TAG, "%s Waiting for data ready.\n", __func__);
        if(ulTaskNotifyTakeIndexed(dataReadyNotifyIndex, pdTRUE, pdMS_TO_TICKS(1000)))
        {
            ESP_LOGI(TAG, "%s Got data ready notification.\n", __func__);
        }
        else
        {
            ESP_LOGE(TAG, "%s Timed out waiting for data ready notification\n", __func__);
            continue;
        }

        // CRC and transmit the audio packet
        activePacket->checksum = crc16(activePacket->payload, activePacket->payloadSize);
        activePacket->echo = true;      // request an echo from the server
        uint32_t packetSize = sizeof(AudioPacket_t) - (PAYLOAD_MAX_LEN - activePacket->payloadSize);
        get_system_time(&timesent);

        ESP_LOGI(TAG, "%s Sending packet with payload len %u and CRC 0x%x to server.\n",
            __func__, activePacket->payloadSize, activePacket->checksum);
        int err = sendto(sock, activePacket, packetSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return;
        }
        ESP_LOGI(TAG, "Message sent");

        // Receive echo from server (useful to measuring Wifi speeds)
        if (activePacket->echo)
        {
            struct AudioPacket_t response;
            struct sockaddr_storage source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, &response, sizeof(response), 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) {
                ESP_LOGE(TAG, "Failed to receive response from server (errno = %d)", errno);
                return;
            }
            else {
                get_system_time(&timerecv);
                ESP_LOGI(TAG, "%s Received %d bytes from %s. Seqnum = %u\n", __func__, len, ipv4_str, response.seqnum);

                struct sockaddr_in * source_addr_in = (struct sockaddr_in *)&source_addr;
                ESP_LOGI(TAG, "%s Source address: %s\n", __func__, inet_ntoa(source_addr_in->sin_addr));

                ESP_LOGI(TAG, "%s Round trip time: %lld\n", __func__, timerecv - timesent);
            }
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

}


////////////////////////////////////////////////////////////////////
// transmit_task 
//
// Main event loop for streaming data to server.
////////////////////////////////////////////////////////////////////
static void transmit_task(void *pvParameters)
{

    transmitTaskState = RELAY_NETWORK_CONNECT;

    while (1) {

        bool error;
        switch (transmitTaskState)
        {
            case RELAY_NETWORK_CONNECT:
            {
                relay_network_connect(&error);

                if (error)
                {
                    ESP_LOGI(TAG, "%s Failed to connect to relay network. Retrying...\n", __func__);
                    transmitTaskState = NETWORK_DISCONNECT;
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    transmitTaskState = STREAM_TO_SERVER;
                }
                break;
            }
            case STREAM_TO_SERVER:
            {
                // This function should ideally never return
                stream_audio_to_server(&error);
                transmitTaskState = NETWORK_DISCONNECT;
                break;
            }
            case NETWORK_DISCONNECT:
            {
                wifi_station_disconnect_and_stop(&error);
                transmitTaskState = RELAY_NETWORK_CONNECT;
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

    /// Create transmit and sampling tasks
    // 
    // The sampling task is responsible for sampling audio
    // data from the guitar and loading this data into a background
    // buffer that is invisible to the transmit task.
    //
    // The transmit task is responsible for transmitting audio
    // data to a server (which in this case is another ESP32). The
    // task reads data out of an "active" buffer. When it is done 
    // transmitting the active buffer, the transmit task will notify
    // the sampling task. The sampling task then swaps the background 
    // and active buffers and notifies the transmit task that new data 
    // is available. 
    //
    ESP_LOGI(TAG, "%s Creating tasks\n", __func__); 
    BaseType_t status;
    
    status = xTaskCreatePinnedToCore(sampling_task, "sampling_task", 8192, NULL, 5, &samplingTaskHandle, ESP_CORE_0);

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
