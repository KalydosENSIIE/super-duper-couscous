
#include "driver/gpio.h"
#include "driver/uart.h"
#include "rom/crc.h"
#include "esp_log.h"

#include "pzem004tv30.h"


#define     UART_BUF_SIZE   150
#define     PZEM004T_MODBUS_ADDRESS     0xF8

#define     VOLTAGE_LL  

typedef enum {
    PZEM004T_FUNCTION_READ_HOLDING_REG  = 0x03,
    PZEM004T_FUNCTION_READ_INPUT_REG    = 0x04,
    PZEM004T_FUNCTION_WRITE_SINGLE_REG  = 0x06,
    PZEM004T_FUNCTION_CALIBRATION       = 0x41,
    PZEM004T_FUNCTION_RESET_ENERGY      = 0x42
} pzem004t_modbus_function_t;


typedef enum {
    PZEM004T_ERROR_ILLEGAL_FUNCTION     = 0x01,
    PZEM004T_ERROR_ILLEGAL_ADDRESS      = 0x02,
    PZEM004T_ERROR_ILLEGAL_DATA         = 0x03,
    PZEM004T_ERROR_SLAVE_ERROR          = 0x04,
} pzem004t_modbus_error_t;





void initialize_UART(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .use_ref_tick = 1,
    };
    
    ESP_ERROR_CHECK( uart_param_config(UART_NUM_2, &uart_config) );
    //uart_set_pin(UART_NUM_2, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_2, UART_BUF_SIZE, 0, 0, NULL, 0) );

    ESP_ERROR_CHECK( uart_set_pin(UART_NUM_2, UART_NUM_2_TXD_DIRECT_GPIO_NUM, UART_NUM_2_RXD_DIRECT_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );

    ESP_ERROR_CHECK( uart_set_mode(UART_NUM_2, UART_MODE_UART) );
}

esp_err_t pzem004tv30_read_all(float *pfVoltage, float *pfCurrent, float *pfPower, float *pfEnergy, float *pfFrequency, float *pfPower_factor)
{
    uint16_t rawVoltage;
    uint32_t rawCurrent, rawPower, rawEnergy;
    uint16_t rawFrequency, rawPower_Factor;

    uint8_t response[25];

    PZEM004Tv30_sendCmd8(PZEM004T_FUNCTION_READ_INPUT_REG, 0x00, 0x0A, 0xF8);
    PZEM004Tv30_receive(response, 25);

    //uart_read_bytes( UART_NUM_2, response, 20, pdMS_TO_TICKS(50) );
    if( response[1] != (uint8_t) 0x04 )
    {
        return ESP_FAIL;
    }

    printf("Response :");
    int i;
    for (i=0 ; i<25 ; i++)
    {
        printf("%02X, ", response[i]);
    }
    printf("\n");


    rawVoltage = ( ((uint16_t) response[3]) << 8 ) +
                 (  (uint16_t) response[4]  << 0 );

    rawCurrent = ( ((uint32_t) response[5]) <<  8 ) +
                 ( ((uint32_t) response[6]) <<  0 ) +
                 ( ((uint32_t) response[7]) << 24 ) +
                 ( ((uint32_t) response[8]) << 16 );
    printf("raw current : %02X %02X %02X %02X : %u\n", response[5], response[6], response[7], response[8], rawCurrent);
    
    rawPower   = ( ((uint32_t) response[9])  <<  8 ) +
                 ( ((uint32_t) response[10]) <<  0 ) +
                 ( ((uint32_t) response[11]) << 24 ) +
                 ( ((uint32_t) response[12]) << 16 );

    rawEnergy  = ( ((uint32_t) response[13]) <<  8 ) +
                 ( ((uint32_t) response[14]) <<  0 ) +
                 ( ((uint32_t) response[15]) << 24 ) +
                 ( ((uint32_t) response[16]) << 16 );

    rawFrequency = ( ((uint16_t) response[17]) << 8 ) +
                   (  (uint16_t) response[18]  << 0 );

    rawPower_Factor = ( ((uint16_t) response[19]) << 8 ) +
                      (  (uint16_t) response[20]  << 0 );

    // TODO check float is big enough for 20 odd years of energy 
    *pfVoltage      = rawVoltage     / 10.0; // Raw voltage in 0.1V
    *pfCurrent      = rawCurrent     / 1000.0; // Raw current in 0.001A
    *pfPower        = rawPower       / 10.0; // Raw power in   0.1W
    *pfEnergy       = rawEnergy      / 1.0; // Raw energy in  1Wh
    *pfFrequency    = rawFrequency   / 10.0; // Raw frequency in 0.1Hz
    *pfPower_factor = rawPower_Factor/ 100.0; // Raw power factor in 0.01

    return ESP_OK;
}


/*!
 * PZEM004Tv30::sendCmd8
 *
 * Prepares the 8 byte command buffer and sends
 *
 * @param[in] cmd - Command to send (position 1)
 * @param[in] rAddr - Register address (positions 2-3)
 * @param[in] regCount - Number of registers to read (positions 4-5)
 *
 * @return success
*/
esp_err_t PZEM004Tv30_sendCmd8(const uint8_t cmd, const uint16_t rAddr, const uint16_t regCount, const uint8_t slave_addr)
{
    uint8_t command [8] = { 0 };
    uint16_t crc;

    // boundaries check for slave_addr
    if( (0xF8 < slave_addr) )
    {
        return ESP_FAIL;
    }



    /* slave address and modbus function number */
    command[0] = slave_addr;
    command[1] = cmd;
    
    /* starting register address */
    command[2] = (uint8_t) (rAddr >> 8) & 0xFF; // high byte
    command[3] = (uint8_t) rAddr & 0xFF; // low byte

    /* registers count */
    command[4] = (uint8_t) (regCount >> 8) & 0xFF; // high byte
    command[5] = (uint8_t) regCount & 0xFF; // low byte

    crc = PZEM004Tv30_CRC16(command, 6);
    command[6] = (uint8_t) ( crc >> 8 ) & 0xFF;
    command[7] = (uint8_t) ( crc >> 0 ) & 0xFF;

    printf("Sending command to PZEM : %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                                                                    command[0], command[1],
                                                                    command[2], command[3], 
                                                                    command[4], command[5], 
                                                                    command[6], command[7]);
    if ( -1 == uart_write_bytes( UART_NUM_2, (char *) command, 8 ) )
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void PZEM004Tv30_search(void){
    static uint8_t response[7];

    for(uint16_t addr = 0x01; addr <= 0xF8; addr++){

        PZEM004Tv30_sendCmd8((uint8_t) PZEM004T_FUNCTION_READ_INPUT_REG, 0x00, 0x01, addr);

        if(PZEM004Tv30_receive(response, 7) != 7){ // Something went wrong
            continue;
        } else {
            printf("Device on addr %02X", addr);
        }
    }
}

/*!
 * PZEM004Tv30::receive
 *
 * Receive data from serial with buffer limit and timeout
 *
 * @param[out] resp Memory buffer to hold response. Must be at least `len` long
 * @param[in] len Max number of bytes to read
 *
 * @return number of bytes read
*/
uint16_t PZEM004Tv30_receive(uint8_t *resp, uint16_t maxLength)
{
/*    unsigned long startTime = millis(); // Start time for Timeout
    uint8_t index = 0; // Bytes we have read
    while((index < len) && (millis() - startTime < READ_TIMEOUT))
    {
        if(_serial->available() > 0)
        {
            uint8_t c = (uint8_t)_serial->read();

            resp[index++] = c;
        }
        yield();	// do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
    }
*/
    uint8_t length;
    length = uart_read_bytes( UART_NUM_2, resp, maxLength, pdMS_TO_TICKS(100) );

    // Check CRC with the number of bytes read
    if(!PZEM004Tv30_checkCRC(resp, length)){
        return 0;
    }

    return length;
}


/*!
 * PZEM004Tv30::checkCRC
 *
 * Performs CRC check of the buffer up to len-2 and compares this checksum to the last two bytes
 *
 * @param[in] data Memory buffer containing the frame to check
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
 * @return is the buffer check sum valid
*/
bool PZEM004Tv30_checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = PZEM004Tv30_CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}


// Pre computed CRC table
static const uint16_t crcTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};


/*!
 * PZEM004Tv30::CRC16
 *
 * Calculate the CRC16-Modbus for a buffer
 * Based on https://www.modbustools.com/modbus_crc16.html
 *
 * @param[in] data Memory buffer containing the data to checksum
 * @param[in] len  Length of the respBuffer
 *
 * @return Calculated CRC
*/
uint16_t PZEM004Tv30_CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t) crcTable[nTemp];
    }
    return crc;
}