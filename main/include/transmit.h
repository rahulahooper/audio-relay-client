#ifndef TRANSMIT_H
#define TRANSMIT_H

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h" 

#include "esp_system.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "common.h"

#define HOST_IP_ADDR                      CONFIG_EXAMPLE_IPV4_ADDR
#define AUDIO_NET_WIFI_SSID               "AudioRelayNetwork"
#define AUDIO_NET_WIFI_PASS               "AudioRelayNetworkPassword" 
#define EXAMPLE_ESP_MAXIMUM_RETRY         10
#define PORT                              CONFIG_EXAMPLE_PORT

#define ESP_WIFI_SAE_MODE                 WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER            ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK

#define WIFI_CONNECTED_BIT      BIT0    // Set this bit when we connect to a Wifi network
#define WIFI_DISCONNECTED_BIT   BIT1    // Set this bit when we disconnect from a Wifi network
 

#define WIFI_CNX_GPIO           GPIO_NUM_16

typedef enum TransmitTaskState_t
{
    XMIT_TASK_INITIAL_STATE,
    XMIT_TASK_STATE_START_WIFI,
    XMIT_TASK_STATE_CONNECT_TO_NETWORK,
    XMIT_TASK_STATE_CREATE_SOCKET,
    XMIT_TASK_STATE_STREAM_TO_SERVER,
    XMIT_TASK_STATE_DESTROY_SOCKET,
    XMIT_TASK_STATE_NETWORK_DISCONNECT,
} TransmitTaskState_t;


////////////////////////////////////////////////////////////////////
// wifi_setup_driver()
//
// Allocates resources for WiFi driver such as WiFi control
// structures, RX/TX buffers, NVS, etc. This function should
// only be called once at system startup. Call 
// wifi_deinit_driver() on shutdown.
////////////////////////////////////////////////////////////////////
esp_err_t wifi_setup_driver(wifi_init_config_t* cfg);



////////////////////////////////////////////////////////////////////
// wifi_setup_cnx_gpio()
//
// Configure the "Wifi connected" GPIO pin
////////////////////////////////////////////////////////////////////
esp_err_t wifi_setup_cnx_led();



////////////////////////////////////////////////////////////////////
// wifi_deinit_driver()
//
// Call on shutdown. WiFi operations will not be supported
// after calling this function.
////////////////////////////////////////////////////////////////////
esp_err_t wifi_deinit_driver();



////////////////////////////////////////////////////////////////////
// _wifi_station_start()
//
////////////////////////////////////////////////////////////////////
void _wifi_station_start(bool* error);



////////////////////////////////////////////////////////////////////
// wifi_station_connect()
//
////////////////////////////////////////////////////////////////////
void wifi_station_connect(bool* error, bool* connected);



////////////////////////////////////////////////////////////////////
// wifi_station_disconnect_and_stop()
//
// This function will disconnect from whatever network we are
// currently on.
////////////////////////////////////////////////////////////////////
void wifi_station_disconnect_and_stop(bool* error);



////////////////////////////////////////////////////////////////////
// create_socket() 
//
////////////////////////////////////////////////////////////////////
void create_socket(bool* error, int* sock, struct sockaddr_in* dest_addr);



////////////////////////////////////////////////////////////////////
// stream_audio_to_server 
//
////////////////////////////////////////////////////////////////////
void stream_audio_to_server(bool* error, const int sock, const struct sockaddr_in dest_addr);



////////////////////////////////////////////////////////////////////
// transmit_task_main
//
// Main event loop for streaming data to server.
////////////////////////////////////////////////////////////////////
void transmit_task_main(void *pvParameters);


#endif // TRANSMIT_H