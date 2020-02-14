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

    float fVoltage = 0.0, fCurrent = 0.0, fPower = 0.0, fEnergy = 0.0, fFrequency = 0.0, fPower_factor = 0.0;

    initialize_UART();

    

    for( ;; )
    {
        
        printf("Entering vMeasurementTask infinite loop\n");

        pzem004tv30_read_all(&fVoltage, &fCurrent, &fPower, &fEnergy, &fFrequency, &fPower_factor) ;
        printf("%.2fV, %.2fA, %.2fW, %.2fWh, %.2fHz, pf=%.2f\n", fVoltage, fCurrent, fPower, fEnergy, fFrequency, fPower_factor);
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

    xTaskCreate( vMeasurementTask,  "measure", 4096, NULL, 30, NULL);
    printf("end of app_main\n");
    /*for(;;)
    {
        vTaskDelay( pdMS_TO_TICKS(1000) );
    }
    */
}

