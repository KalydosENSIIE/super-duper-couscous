/* 
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "pzem004tv30.h"

#define TAG "Main"

void vMeasurementTask( void *pvParameters )
{
    printf("Entering vMeasurementTask\n");

    pzem004tv30_t pzem004t = {  .modbus_address = PZEM004TV30_MODBUS_DEFAULT_ADDRESS,
                                .uart_num = UART_NUM_2,
                                .tx_io_num = UART_NUM_2_TXD_DIRECT_GPIO_NUM,
                                .rx_io_num = UART_NUM_2_RXD_DIRECT_GPIO_NUM,
                                .measurements = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
                             };

    pzem004tv30_initialize_UART( &pzem004t );

    

    for( ;; )
    {
        
        printf("Entering vMeasurementTask infinite loop\n");

        pzem004tv30_update_measurements( &pzem004t );
        
        printf("%.2fV, %.2fA, %.2fW, %.2fWh, %.2fHz, pf=%.2f\n", pzem004t.measurements.voltage, pzem004t.measurements.current, pzem004t.measurements.power, pzem004t.measurements.energy, pzem004t.measurements.frequency, pzem004t.measurements.powerFactor);
        vTaskDelay( pdMS_TO_TICKS(1000) );
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
    esp_log_level_set("pzem004tv30", ESP_LOG_DEBUG);

    xTaskCreate( vMeasurementTask,  "measure", 4096, NULL, 30, NULL);

    // app_main task is killed when we let it return
}

