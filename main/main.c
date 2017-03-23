#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* ble includes */
#include "controller.h"
#include "bt.h"
#include "bt_trace.h"
#include "bt_types.h"
#include "btm_api.h"
#include "bta_api.h"
#include "bta_gatt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
const int BLE_CONNECTED_BIT = BIT1;

static const char *DEBUG_TAG = "UDPC";

static bool is_ble_initialized = false;

bool is_wifi_connected();
bool is_ble_connected();
void start_ble_scan();
void stop_ble_scan();

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type          = ESP_PUBLIC_ADDR,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x25
};

int send_to_server(const char* packet, int len);
static void esp_gap_cb(uint32_t event, void *param);

#define MAX_PK_LEN 		31
struct BLEPacketExt 
{
	uint16_t	_id;
	uint16_t	_size;
	uint8_t 	_deviceAddress[6];
	char 		_packetBuffer[31];
	int8_t 		_rssi;
	
} __attribute__((packed));

static struct BLEPacketExt _packet;
#define SCAN_DURATION 20

static void esp_gap_cb(uint32_t event, void *param)
{
	uint8_t *adv_data = NULL;
	uint8_t adv_data_len = 0;
    
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second        
        esp_ble_gap_start_scanning(SCAN_DURATION);
        break;
    }
    
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
			LOG_INFO("Found %02x:%02x:%02x:%02x:%02x:%02x - RSSI: %d\n",  
								scan_result->scan_rst.bda[0],
								scan_result->scan_rst.bda[1],
								scan_result->scan_rst.bda[2], 
								scan_result->scan_rst.bda[3], 
								scan_result->scan_rst.bda[4], 
								scan_result->scan_rst.bda[5], 
								scan_result->scan_rst.rssi);
								
            /* Get MNF data (VNGpacket) */
			adv_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
												ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &adv_data_len);
												
			if (adv_data_len > 0) // VNG Packet hehe
			{
				_packet._id = 0;
				_packet._size = 37;
				
				// copy MAC
				memcpy((void*) &_packet._deviceAddress, (void*) &scan_result->scan_rst.bda, 6);
				
				// copy data 
				memcpy((void*) &_packet._packetBuffer, (void*) scan_result->scan_rst.ble_adv, MAX_PK_LEN);
				
				// rssi
				_packet._rssi = scan_result->scan_rst.rssi;
				
				// send here
				send_to_server((const char*) &_packet, sizeof (_packet));
				
			}
            break;
            
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
			if (is_wifi_connected())
				esp_ble_gap_start_scanning(SCAN_DURATION);
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        // if ble initilized, start scan
        if (is_ble_initialized)
        {
			start_ble_scan();
		}
        break;
    
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    
		// stop ble scan
        if (is_ble_initialized)
        {
			stop_ble_scan();
		}
        
        break;        
    default:
        break;
    }
    return ESP_OK;
}

void start_ble_scan() {
	LOG_INFO("register callback\n");
	int status;

    //register the scan callback function to the gap moudule
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        LOG_ERROR("gap register error, error code = %x\n", status);
        return;
    }

	LOG_INFO("starting ble scan\n");
    esp_ble_gap_set_scan_params(&ble_scan_params);
}

void stop_ble_scan() {
	LOG_INFO("stop ble scan\n");
	esp_ble_gap_stop_scanning();	
}

void init_ble_subsystem() {
	LOG_INFO("init ble subsystem - api changed\n");
	esp_bt_controller_init();
	esp_bluedroid_init();
	esp_bluedroid_enable();
	
	is_ble_initialized = true;
}

/* 
 * error - wrapper for perror
 */
void app_error(char *msg) {
    perror(msg);
    exit(0);
}

#define SERVERHOST "192.168.0.100"
#define SERVERPORT 8888

int send_to_server(const char* packet, int len) {
    int sockfd, n;
    uint32_t serverlen;
    struct sockaddr_in serveraddr;
    
    ESP_LOGI(DEBUG_TAG, "wait connect to AP");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
					false, true, portMAX_DELAY);
	ESP_LOGI(DEBUG_TAG, "Connected to AP");

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) 
    {
        ESP_LOGI(DEBUG_TAG, "ERROR opening socket");
        return -1;
	}

    /* build the server's Internet address */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = inet_addr(SERVERHOST);
    serveraddr.sin_port = htons(SERVERPORT);

	do
	{
		/* send the message to the server */
		ESP_LOGI(DEBUG_TAG, "do sendto");
		serverlen = sizeof(serveraddr);
		n = sendto(sockfd, packet, len, 0, &serveraddr, serverlen);
		if (n < 0) {
		  ESP_LOGI(DEBUG_TAG, "ERROR in sendto");
		  goto quit;
		}
		
    } while (0);
    
quit:
	close(sockfd);
	
    return 0;
}

static void udpclient_thread(void *p)
{
	while (true) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }	
}

static void counter_thread(void *p)
{
	static int count = 0;
	while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(DEBUG_TAG, "------uptime: %d s\r\n", count++); 
    }	
}

#define WIFIHOTSPOT "mywifi"
#define WIFIPASSWORD "mypassword"

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFIHOTSPOT,
            .password = WIFIPASSWORD,
        },
    };
    ESP_LOGI(DEBUG_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

bool is_wifi_connected()
{
	EventBits_t connected = xEventGroupGetBits(wifi_event_group);
	return (connected & CONNECTED_BIT);
}

bool is_ble_connected()
{
	EventBits_t connected = xEventGroupGetBits(wifi_event_group);
	return (connected & BLE_CONNECTED_BIT);
}

void wait_to_connect()
{
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
					false, true, portMAX_DELAY);
}
	
void app_main(void)
{
    nvs_flash_init();
    initialise_wifi();
    
    wait_to_connect();
    
    init_ble_subsystem();
    start_ble_scan();
    

	xTaskCreate(counter_thread, "counter", 512, NULL, 5, NULL);
	xTaskCreate(udpclient_thread, "udpfwd", 4096, NULL, 5, NULL);
    
}
