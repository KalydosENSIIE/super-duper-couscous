#ifndef _PZEM004Tv30_H
#define _PZEM004Tv30_H

#include "driver/gpio.h"
#include "driver/uart.h"

#define     PZEM004TV30_MODBUS_DEFAULT_ADDRESS     ( (uint16_t) 0xF8 )

// alarm isn't there coz I have no use for it =/
typedef struct pzem004tv30_measurements
{
    float voltage;      // V
    float current;      // A
    float power;        // W
    float energy;       // Wh
    float frequency;    // Hz
    float powerFactor; // .
} pzem004tv30_measurements_t;


typedef struct pzem004tv30
{
    uint16_t modbus_address;
    uart_port_t uart_num;
    int tx_io_num;
    int rx_io_num;
    pzem004tv30_measurements_t measurements;
} pzem004tv30_t;

esp_err_t pzem004tv30_initialize_UART( pzem004tv30_t * pzem004t );

esp_err_t pzem004tv30_update_measurements( pzem004tv30_t * const pzem004t );
uint16_t pzem004tv30_receive( const pzem004tv30_t * const pzem004t, uint8_t * const response, const uint16_t maxLength );
bool pzem004tv30_checkCRC( const uint8_t *buf, uint16_t len );
void pzem004tv30_search( void );
esp_err_t pzem004tv30_sendCmd8( const pzem004tv30_t * const pzem004t, const uint8_t cmd, const uint16_t rAddr, const uint16_t regCount, const uint8_t slave_addr );



#endif // _PZEM004Tv30_H