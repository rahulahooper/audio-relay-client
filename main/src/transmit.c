//////////////////////////////////////////////////////////////
///
/// transmit.c
///
/// The transmit task is responsible for setting up a Wifi
/// connection with and sending audio packets to the receiving 
/// ESP32 and sending audio. This task periodically
/// exchanges data with the sampling task using double-
/// buffering. Specifically, the playback task sends audio data
/// in an "active packet" to the receiving ESP32. When it is done
/// doing so, it signals to the sampling task that it is ready 
/// to transmit another packet. The sampling task swaps the
/// background and active packets, then signals to the transmit
/// task that new data is available.
/// 
//////////////////////////////////////////////////////////////

#include "transmit.h"

static TransmitTaskState_t transmitTaskState;

static EventGroupHandle_t s_wifi_event_group; // FreeRTOS event group to signal when we are connected

static int gWifiConnectAttempts = 0;

extern AudioPacket_t * activePacket;     // transmitting task transmits this packet
extern AudioPacket_t * backgroundPacket; // sampling task fills this packet

extern const UBaseType_t transmissionDoneNotifyIndex;      // set by the transmission thread when it is done transmitting data
extern const UBaseType_t dataReadyNotifyIndex;             // set by the sampling thread when there is new data to transmit

extern TaskHandle_t samplingTaskHandle;                    // FIXME: If these task handles are declared

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
        xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(__func__, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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
    ESP_LOGI(__func__, "Setting up WiFi driver\n");        
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t default_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&default_cfg));

    memcpy(cfg, &default_cfg, sizeof(default_cfg));

    return ESP_OK;
}



////////////////////////////////////////////////////////////////////
// wifi_setup_cnx_gpio()
//
// Configure the "Wifi connected" GPIO pin
////////////////////////////////////////////////////////////////////
esp_err_t wifi_setup_cnx_led()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1 << WIFI_CNX_GPIO, // Bit mask for the GPIO
        .mode = GPIO_MODE_OUTPUT,           // Set as output mode
        .pull_up_en = 0,                    // No pull-up
        .pull_down_en = 0,                  // No pull-down
        .intr_type = GPIO_INTR_DISABLE      // Disable interrupts
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(WIFI_CNX_GPIO, 0));

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
// _wifi_station_start()
//
////////////////////////////////////////////////////////////////////
void _wifi_station_start(bool* error)
{
    ESP_LOGI(__func__, "Connecting to Audio Relay Wifi Network\n");

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
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &audio_wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    
    ESP_LOGI(__func__, "Started WiFi!\n");
    *error = false;
}



////////////////////////////////////////////////////////////////////
// wifi_station_connect()
//
////////////////////////////////////////////////////////////////////
void wifi_station_connect(bool* error, bool* connected)
{
    ESP_LOGI(__func__, "Connecting to Wifi station\n");
    *error = true;

    // Wait until either the connection is established (WIFI_CONNECTED_BIT) or 
    // connection failed after the maximum number of re-tries (WIFI_FAIL_BIT). 
    // The bits are set by wifi_event_handler() (see above)
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(1000));
    
    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(__func__, "connected to Wifi!\n");

        // Turn on the "Wifi connected" LED
        gpio_set_level(WIFI_CNX_GPIO, 1);

        *connected = true;
        *error = false;
    } 
    else if (bits & WIFI_DISCONNECTED_BIT)
    {
        ESP_LOGI(__func__, "Failed to connect to the AP, retrying (attempt #%u)\n", gWifiConnectAttempts);
        *connected = false;
        *error = false;
    }
    else 
    {
        ESP_LOGE(__func__, "UNEXPECTED EVENT: %lx\n", bits);
        *connected = false;
        *error = false;
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
    ESP_LOGI(__func__, "Disconnecting from home WiFi network\n");
    *error = true;

    // Turn off the "Wifi connected" LED
    ESP_ERROR_CHECK(gpio_set_level(WIFI_CNX_GPIO, 0));

    esp_err_t ret = esp_wifi_disconnect();
    if (ret == ESP_ERR_WIFI_NOT_STARTED)
    {
        ESP_LOGI(__func__, "ESP32 Wifi station wasn't started wifi_station_start_and_connect()\n");
        *error = false;
        return;
    }
    else if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGI(__func__, "ESP32 Wifi resources were not initialized\n");
        *error = false;
        return;
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "Unexpected errno: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_wifi_stop();

    if (ret == ESP_ERR_WIFI_NOT_INIT)
    {
        ESP_LOGE(__func__, "ESP32 Wifi resouces were not initialized\n");
        return;
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "Unexpected errno: %s\n", esp_err_to_name(ret));
        return;
    }

    *error = false;
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
        ESP_LOGE(__func__, "Got a nullptr: esp_netif=%p, ip_info=%p\n", esp_netif, ip_info);
        vTaskDelete(NULL);
        return;
    }

    ESP_ERROR_CHECK( esp_netif_get_ip_info(esp_netif, ip_info) );
    struct in_addr ipv4_addr;
    char ipv4_str[INET_ADDRSTRLEN];
    ipv4_addr.s_addr = ip_info->gw.addr;
    if (inet_ntop(AF_INET, &ipv4_addr, ipv4_str, INET_ADDRSTRLEN) != NULL)
    {
        ESP_LOGI(__func__, "Gateway IPv4: %s", ipv4_str);
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
        ESP_LOGE(__func__, "Unable to create socket: errno %d", errno);
        return;
    }

    // Set timeout for receiving echoes from server
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(*sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    ESP_LOGI(__func__, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
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
        ESP_LOGI(__func__, "Socket was invalid\n");
        return;
    }

    int64_t timesent, timerecv;

    // Wait for sampling task to come up
    while (samplingTaskHandle == NULL)
    {
        ESP_LOGI(__func__, "Waiting for sampling task to be created...\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // Init some statistics-related variables
    uint32_t bytesSinceLastEcho = 0;
    uint32_t packetsSinceLastEcho = 0;
    int64_t timeOfLastEcho;
    get_system_time(&timeOfLastEcho);

    // Stream audio data to server

    while (1) {

        // Kick the watchdog timer
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_reset());

        // Indicate to sampling thread that there is no data to transmit
        xTaskNotifyGiveIndexed(samplingTaskHandle, transmissionDoneNotifyIndex);

        // Wait for the sampling task to indicate that there is new data available
        if(ulTaskNotifyTakeIndexed(dataReadyNotifyIndex, pdTRUE, pdMS_TO_TICKS(1000)))
        {
            PRINTF_DEBUG((__func__, "Got data ready notification.\n"));
        }
        else
        {
            ESP_LOGE(__func__, "Timed out waiting for data ready notification\n");
            continue;
        }
        // Timestamp and transmit the audio packet
    
        activePacket->echo = !(activePacket->seqnum % 5000);      // request an echo from the server 
        uint32_t packetSize = sizeof(AudioPacket_t);

        get_system_time(&timesent);

        for (int i = 0; i < MAX_SEND_ATTEMPTS; i++)
        {
            PRINTF_DEBUG((__func__, "Sending packet with seqnum %u, payload len %u, CRC 0x%x to server. Total size = %lu\n",
                activePacket->seqnum, activePacket->payloadSize, activePacket->checksum, packetSize));

            int err = sendto(sock, activePacket, packetSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            if (err < 0) {
                if (errno == ENOMEM)
                {
                    PRINTF_DEBUG((__func__, "Failed to send packet w/seqnum %u over socket. Error is likely full send buffer. Retrying (%u/%u)\n",
                        activePacket->seqnum, i+1, MAX_SEND_ATTEMPTS));
                    vTaskDelay(1);
                    continue;
                }

                ESP_LOGE(__func__, "Error sending packet %u: errno %u (%s)", activePacket->seqnum, errno, strerror(errno));

                // Check connection with server and exit if we are disconnected
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_ERR_WIFI_NOT_CONNECT)
                {
                    ESP_LOGE(__func__, "Disconnected from server. Exiting transmit loop\n");
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
                ESP_LOGE(__func__, "Failed to receive echo'd packet %u from server (errno = %s)", activePacket->seqnum, strerror(errno));
                continue;
            }
            
            // We successfully received an echo'd packet. Report statistics
            get_system_time(&timerecv);
            ESP_LOGI(__func__, "Received %d bytes. Seqnum = %u, payload size = %u\n", len, response.seqnum, response.numSamples);
            ESP_LOGI(__func__, "Round trip time: %lld usec\n", timerecv - timesent);

            uint32_t avgPacketSize = bytesSinceLastEcho / packetsSinceLastEcho / AUDIO_PACKET_BYTES_PER_SAMPLE;
            float avgThroughputBytes = 1.0f * bytesSinceLastEcho / (timerecv - timeOfLastEcho) * 1000 * 1000;
            float avgThroughputPackets = 1.0f * packetsSinceLastEcho / (timerecv - timeOfLastEcho) * 1000 * 1000;

            ESP_LOGI(__func__, "%s average packet size = %lu samples, average throughput (bytes/sec) = %f, (packets/sec) = %f\n\n", 
                __func__, avgPacketSize, avgThroughputBytes, avgThroughputPackets);

            timeOfLastEcho = timerecv;
            bytesSinceLastEcho = 0;
            packetsSinceLastEcho = 0;
        }
    }
}


////////////////////////////////////////////////////////////////////
// transmit_task_main
//
// Main event loop for streaming data to server.
////////////////////////////////////////////////////////////////////
void transmit_task_main(void *pvParameters)
{

    // Sign up for the task watchdog. This will be useful for knowing whether this
    // task is blocked for extended periods without us knowing about it.
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    // Initialize the TCP/IP stack, Wifi driver, and connection LED
    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t cfg;
    ESP_ERROR_CHECK(wifi_setup_driver(&cfg));
    ESP_ERROR_CHECK(wifi_setup_cnx_led());

    transmitTaskState = XMIT_TASK_INITIAL_STATE;

    while (1) 
    {

        // Kick the watchdog timer
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_reset());

        bool error;
        static int socket = -1;
        struct sockaddr_in dest_addr;   // stores IP information about server

        switch (transmitTaskState)
        {
            case XMIT_TASK_INITIAL_STATE:
            case XMIT_TASK_STATE_START_WIFI:
            {
                _wifi_station_start(&error);

                if (error)
                {
                    ESP_LOGI(__func__, "Failed to connect to relay network. Retrying...\n");
                    transmitTaskState = XMIT_TASK_STATE_NETWORK_DISCONNECT;
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    transmitTaskState = XMIT_TASK_STATE_CONNECT_TO_NETWORK;
                }
                break;
            }
            case XMIT_TASK_STATE_CONNECT_TO_NETWORK:
            {
                bool connected = false;
                wifi_station_connect(&error, &connected);

                if (error)
                {
                    ESP_LOGI(__func__, "Error connecting to network!\n");
                    transmitTaskState = XMIT_TASK_STATE_NETWORK_DISCONNECT;
                }
                else if (connected)
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
                    ESP_LOGE(__func__, "Failed to create socket. Disconnecting from network and retrying...\n");
                    transmitTaskState = XMIT_TASK_STATE_DESTROY_SOCKET;
                }
                else
                {
                    ESP_LOGI(__func__, "Transitioning to STREAM_TO_SERVER, sock = %d\n", socket);
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
                transmitTaskState = XMIT_TASK_INITIAL_STATE;
                break;
            }
            default:
                ESP_LOGE(__func__, "Invalid state %u\n", transmitTaskState);
                break;
        }
    }
    
    ESP_LOGE(__func__, "Terminating transmit task\n");
    vTaskDelete(NULL);
}

