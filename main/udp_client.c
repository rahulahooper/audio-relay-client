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
    uint16_t checksum;
    uint16_t payloadSize;
    uint8_t  payload[PAYLOAD_MAX_LEN];
} AudioPacket_t;


typedef struct ResponsePacket_t
{
    uint16_t seqnum;
    char     response[5];   // large enough to hold the "NACK" string
} ResponsePacket_t;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

esp_err_t wifi_station_start_and_connect(wifi_config_t* wifi_config, bool* error);



////////////////////////////////////////////////////////////////////
// crc16()
//
////////////////////////////////////////////////////////////////////
uint16_t crc16(uint8_t* data, uint32_t len)
{
    // Compute a crc16 using polynomial 0x1021 and seed value 0xFFFF
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
// set_system_time()
//
// Connects to a Network Time Protocol (NTP) server for setting the
// ESP32 clock. The function disconnects from the server at exit.
// The ESP32 must be connected to the Internet before entering this
// function (see wifi_station_start_and_connect()).
////////////////////////////////////////////////////////////////////
esp_err_t set_system_time()
{
    ESP_LOGI(TAG, "%s Initializing sntp network interface\n", __func__);

    // TODO: Verify that we are connected to a Wifi network before connecting
    //       to the NTP server
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.start = false;               // wait for Wifi connection before starting SNTP service
    config.server_from_dhcp = true;     // use DHCP server to get SNTP IP address
    config.smooth_sync = true;          // gradually reduce time error
    ESP_ERROR_CHECK(esp_netif_sntp_init(&config));

    ESP_LOGI(TAG, "%s Starting SNTP client\n", __func__);
    ESP_ERROR_CHECK(esp_netif_sntp_start());

    time_t now = 0;
    struct tm timeinfo = { 0 };
    esp_err_t ret = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: Failed to update system time within 10s timeout! error = %s\n", __func__, esp_err_to_name(ret));
        return ESP_OK;
    }

    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "%s: Connected to SNTP server!\n", __func__);

    esp_netif_sntp_deinit();

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in New York is: %s", strftime_buf);

    return ESP_OK;
}   

////////////////////////////////////////////////////////////////////
// get_system_time()
//
// Return time since epoch in microseconds. If system time is not
// set properly (see set_system_time), results won't reflect the 
// actual time.
////////////////////////////////////////////////////////////////////
void get_system_time(int64_t* time_us)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    *time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


////////////////////////////////////////////////////////////////////
// wifi_setup_driver()
//
// Allocates resources for WiFi driver such as WiFi control
// structures, RX/TX buffers, NVS, etc. This function should
// only be called once. Call wifi_deinit_driver() on shutdown.
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
    ESP_LOGE(TAG, "%s Not implemented yet\n", __func__);
    return ESP_ERR_NOT_SUPPORTED;
}

////////////////////////////////////////////////////////////////////
// wifi_station_start_and_connect()
//
////////////////////////////////////////////////////////////////////
esp_err_t wifi_station_start_and_connect(wifi_config_t* wifi_config, bool* error)
{
    *error = true;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_station_start_and_connect() finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
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

    return ESP_OK;
}


////////////////////////////////////////////////////////////////////
// wifi_station_disconnect_and_stop()
//
////////////////////////////////////////////////////////////////////
esp_err_t wifi_station_disconnect_and_stop()
{
    esp_err_t ret = esp_wifi_disconnect();
    if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGE(TAG, "%s ESP32 Wifi was not initialized by wifi_station_start_and_connect()\n", __func__);
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s Unexpected errno: %s\n", __func__, esp_err_to_name(ret));
    }

    ret = esp_wifi_stop();

    if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGE(TAG, "%s ESP32 Wifi was not initialized by wifi_station_start_and_connect()\n", __func__);
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s Unexpected errno: %s\n", __func__, esp_err_to_name(ret));
    }

    return ret;
}

static AudioPacket_t gAudioPackets[2];
static AudioPacket_t * activePacket;   // transmitting task transmits this packet
static AudioPacket_t * backgroundPacket; // sampling task fills this packet

static const UBaseType_t transmissionDoneNotifyIndex = 0;      // set by the transmission thread when it is done transmitting data
static const UBaseType_t dataReadyNotifyIndex;                 // set by the sampling thread when there is new data to transmit

static TaskHandle_t samplingTaskHandle = NULL;
static TaskHandle_t transmitTaskHandle = NULL;


////////////////////////////////////////////////////////////////////
// sampling_task 
//
// Main event loop for sampling audio data
////////////////////////////////////////////////////////////////////
void sampling_task()
{
    activePacket = &gAudioPackets[0];
    backgroundPacket = &gAudioPackets[1];

    const uint32_t SAMPLE_RATE = 44100; // hz
    const uint32_t SAMPLE_PERIOD_MS = (uint32_t)(1.0f / SAMPLE_RATE / 1000);

    while (transmitTaskHandle == NULL)
    {
        ESP_LOGI(TAG, "%s: Waiting for transmit task to be created...\n", __func__);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    uint16_t seqnum = 0;

    while (true)
    {

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

        ESP_LOGI(TAG, "%s Swapped buffers, notifying transmission task...\n", __func__);
        xTaskNotifyGiveIndexed(transmitTaskHandle, dataReadyNotifyIndex);

        // Q: How do we ensure that the sampling thread doesn't run through this loop multiple times before
        //      the xmitter thread gets time to run?
    }

    vTaskDelete(NULL);
}


////////////////////////////////////////////////////////////////////
// transmit_task 
//
// Main event loop for streaming data to server.
////////////////////////////////////////////////////////////////////
static void transmit_task(void *pvParameters)
{
    struct ResponsePacket_t response;
    int addr_family = 0;
    int ip_protocol = 0;

    // I guess we're using the default one?
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

    while (samplingTaskHandle == NULL)
    {
        ESP_LOGI(TAG, "%s Waiting for sampling task to be created...\n", __func__);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(ipv4_str);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif


        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        int64_t timesent, timerecv;

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
            uint32_t packetSize = sizeof(AudioPacket_t) - (PAYLOAD_MAX_LEN - activePacket->payloadSize);
            get_system_time(&timesent);

            ESP_LOGI(TAG, "%s Sending packet with payload len %u and CRC 0x%x to server.\n",
                __func__, activePacket->payloadSize, activePacket->checksum);
            int err = sendto(sock, activePacket, packetSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            // Receive ACK from server (this can be deleted later)
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, &response, sizeof(struct ResponsePacket_t) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            
            if (len < 0) {          // Error occurred during receiving
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            else {                  // Data received
                get_system_time(&timerecv);
                response.response[4] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "%s Received %d bytes from %s:", __func__, len, ipv4_str);

                struct sockaddr_in * source_addr_in = (struct sockaddr_in *)&source_addr;
                ESP_LOGI(TAG, "%s Source address: %s\n", __func__, inet_ntoa(source_addr_in->sin_addr));
                ESP_LOGI(TAG, "%s Message content: %s", __func__, response.response);


                ESP_LOGI(TAG, "%s Round trip time: %lld\n", __func__, timerecv - timesent);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    #if CONFIG_ESP_WIFI_AUTH_WPA3_PSK             // use this one
    ESP_LOGI(TAG, "WPA3_PSK selected!");
    #endif

    #ifdef CONFIG_LWIP_DHCP_GET_NTP_SRV
    ESP_LOGI(TAG, "%s LWIP config'd\n", __func__);
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

    ESP_LOGI(TAG, "%s Connecting to home WIFI network\n", __func__);
    bool error = true;


    wifi_config_t home_wifi_config = {
        .sta = {
            .ssid = "NETGEAR56",
            .password = "livelyoctopus070",
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            // .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            // .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            // .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    ESP_ERROR_CHECK(wifi_station_start_and_connect(&home_wifi_config, &error));
    if (error)
    {
        ESP_LOGE(TAG, "%s: Failed to connect to home WIFI network\n", __func__);
        return;
    }
    else
    {
        ESP_LOGI(TAG, "%s Connected to home WIFI network\n", __func__);
    }

    ESP_ERROR_CHECK(set_system_time());

    ESP_LOGI(TAG, "%s: Disconnecting from home WiFi network\n", __func__);
    ESP_ERROR_CHECK(wifi_station_disconnect_and_stop());

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

    // TODO: This needs to go in a try-loop in case the client comes up
    //       before the server, or in case the server powers down, then
    //       comes back up.
    wifi_station_start_and_connect(&audio_wifi_config, &error);
    BaseType_t status = xTaskCreatePinnedToCore(transmit_task, "transmit_task", 4096, NULL, 5, &transmitTaskHandle, ESP_CORE_0);

    if (status != pdPASS)
    {
        ESP_LOGE(TAG, "%s Failed to create transmit task!\n", __func__);
        return;
    }

    status = xTaskCreatePinnedToCore(sampling_task, "sampling_task", 4096, NULL, 5, &samplingTaskHandle, ESP_CORE_1);

    if (status != pdPASS)
    {
        ESP_LOGE(TAG, "%s Failed to create sampling task!\n", __func__);
        return;
    }
}
