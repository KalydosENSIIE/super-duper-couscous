/* 
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"


#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "pzem004tv30.h"
#include "mqtt.h"

#define TAG "MAIN"

#define CPU_0   0
#define CPU_1   1

#define WIFI_SSID               CONFIG_ESP_WIFI_SSID 
#define WIFI_PASS               CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY      CONFIG_ESP_WIFI_RECONNECT_MAXIMUM_RETRY
#define MEASUREMENT_INTERVAL_MS CONFIG_ESP_MEASUREMENT_INTERVAL_MS



/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static int s_wifi_connect_retry_num = 0;


/* Local functions prototypes */
static void event_handler( void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data );
static void wifi_init_sta( void );
void vMeasurementTask ( void *pvParameters );







static void event_handler( void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data )
{
    if ( event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START ) {
        esp_wifi_connect();
    } 
    else if ( ( event_base == WIFI_EVENT ) && ( event_id == WIFI_EVENT_STA_DISCONNECTED ) )
    {
        ESP_LOGW( TAG, "Failed connecting to the AP" );

        if ( s_wifi_connect_retry_num < WIFI_MAXIMUM_RETRY ) {
            ESP_LOGI( TAG, "Retrying connecting to the AP" );
            esp_wifi_connect();
            s_wifi_connect_retry_num++;
        } 
        else
        {
            ESP_LOGW( TAG, "Failed to connect to the AP after %d retries", WIFI_MAXIMUM_RETRY );
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } 
    else if ( ( event_base == IP_EVENT ) && ( event_id == IP_EVENT_STA_GOT_IP ) ) 
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI( TAG, "got ip:%s",
                  ip4addr_ntoa( &( event->ip_info.ip ) ) );
        s_wifi_connect_retry_num = 0;
        xEventGroupSetBits( s_wifi_event_group, WIFI_CONNECTED_BIT );
        /* 
        TODO s_wifi_event_group is deleted at end of wifi_init_sta.
        Need to investigate why, and what happens when the AP reboots or something.
        */
    }
}

static void wifi_init_sta( void )
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );

    ESP_ERROR_CHECK( esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL ) );
    ESP_ERROR_CHECK( esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL ) );

    wifi_config_t wifi_config = 
    {
        .sta = 
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
    ESP_ERROR_CHECK( esp_wifi_set_config( ESP_IF_WIFI_STA, &wifi_config ) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_LOGI( TAG, "wifi_init_sta finished. Waiting for successful connection." );

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits( s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY );

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if ( bits & WIFI_CONNECTED_BIT ) 
    {
        ESP_LOGI( TAG, "connected to ap SSID:%s password:%s",
                  WIFI_SSID, WIFI_PASS );
    } 
    else if ( bits & WIFI_FAIL_BIT ) 
    {
        ESP_LOGI( TAG, "Failed to connect to SSID:%s, password:%s",
                  WIFI_SSID, WIFI_PASS );
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
    TickType_t xLastWakeTime;

    pzem004t = (pzem004tv30_t) { .modbus_address = PZEM004TV30_MODBUS_DEFAULT_ADDRESS,
                                 .uart_num = UART_NUM_2,
                                 .tx_io_num = UART_NUM_2_TXD_DIRECT_GPIO_NUM,
                                 .rx_io_num = UART_NUM_2_RXD_DIRECT_GPIO_NUM,
                                 .measurements = 
                                 {
                                     .voltage = 0.0,
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
        ESP_LOGE( TAG, "Failed to initialize pzem004t UART.");
        while ( 1 ) { ; }
    }

    ESP_LOGD( TAG, "Entering vMeasurementTask infinite loop" );
    while ( 1 )
    {
        xLastWakeTime = xTaskGetTickCount();

        rslt = pzem004tv30_update_measurements( &pzem004t );
        
        if ( ESP_OK == rslt )
        {
            ESP_LOGI( TAG, "%.1fV, %.2fA, %.2fW, %.2fWh, %.2fHz, pf=%.2f", \
                            pzem004t.measurements.voltage, \
                            pzem004t.measurements.current, \
                            pzem004t.measurements.power, \
                            pzem004t.measurements.energy, \
                            pzem004t.measurements.frequency, \
                            pzem004t.measurements.powerFactor \
                    );
            // TODO here, send values to a queue
        }
        else
        {
            ESP_LOGE( TAG, "Error communicating with the PZEM004t module." );
        }
        
        // TODO use a timer / eventGroup thing to avoid having this wait ?
        // would that really be better though ?
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( MEASUREMENT_INTERVAL_MS ) );
    }
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("PZEM004TV30", ESP_LOG_DEBUG);
    esp_log_level_set("MQTT", ESP_LOG_DEBUG);
    esp_log_level_set("MAIN", ESP_LOG_DEBUG);
    

    xTaskCreatePinnedToCore( vMeasurementTask,  "measure", 4096, NULL, 1, NULL, CPU_0);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();


    // app_main task is killed when we let it return
}

