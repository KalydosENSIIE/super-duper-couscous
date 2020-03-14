/* 
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "pzem004tv30.h"

#define TAG "Main"

#define CPU_0   0
#define CPU_1   1

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID 
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static int s_retry_num = 0;

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
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } 
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } 
    else 
    {
        ESP_LOGE( TAG, "UNEXPECTED EVENT" );
    }

    ESP_ERROR_CHECK( esp_event_handler_unregister( IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler ) );
    ESP_ERROR_CHECK( esp_event_handler_unregister( WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler ) );
    vEventGroupDelete( s_wifi_event_group );
}



void vMeasurementTask ( void *pvParameters )
{
    pzem004tv30_t pzem004t;
    esp_err_t rslt;
    
    pzem004t = (pzem004tv30_t) { .modbus_address = PZEM004TV30_MODBUS_DEFAULT_ADDRESS,
                                 .uart_num = UART_NUM_2,
                                 .tx_io_num = UART_NUM_2_TXD_DIRECT_GPIO_NUM,
                                 .rx_io_num = UART_NUM_2_RXD_DIRECT_GPIO_NUM,
                                 .measurements = {  .voltage = 0.0,
                                                    .current = 0.0,
                                                    .power = 0.0,
                                                    .energy = 0.0,
                                                    .frequency = 0.0,
                                                    .powerFactor = 0.0
                                                 }
                               };

    rslt = pzem004tv30_initialize_UART( &pzem004t );
    if ( ESP_OK != rslt )
    {
        ESP_LOGE( TAG, "Failed to initialize pzem004t UART.\n");
        while ( 1 ) { ; }
    }

    ESP_LOGD( TAG, "Entering vMeasurementTask infinite loop\n" );
    while ( 1 )
    {
        rslt = pzem004tv30_update_measurements( &pzem004t );
        
        if ( ESP_OK == rslt )
        {
            ESP_LOGI( TAG, "%.1fV, %.2fA, %.2fW, %.2fWh, %.2fHz, pf=%.2f\n", \
                            pzem004t.measurements.voltage, \
                            pzem004t.measurements.current, \
                            pzem004t.measurements.power, \
                            pzem004t.measurements.energy, \
                            pzem004t.measurements.frequency, \
                            pzem004t.measurements.powerFactor \
                    );
        }
        else
        {
            ESP_LOGE( TAG, "Error communicating with the PZEM004t module.\n" );
        }
        
        vTaskDelay( pdMS_TO_TICKS(500) );
    }
}

void app_main()
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%luMB %s flash\n", (unsigned long) (spi_flash_get_chip_size() / (1024 * 1024) ),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("PZEM004TV30", ESP_LOG_DEBUG);

    xTaskCreatePinnedToCore( vMeasurementTask,  "measure", 4096, NULL, 1, NULL, CPU_0);


    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();


    // app_main task is killed when we let it return
}

