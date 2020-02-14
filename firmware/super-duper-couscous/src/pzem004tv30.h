#pragma once

void initialize_UART(void);

esp_err_t pzem004tv30_read_all(float *pfVoltage, float *pfCurrent, float *pfPower, float *pfEnergy, float *pfFrequency, float *pfPower_factor);

uint16_t PZEM004Tv30_receive(uint8_t *resp, uint16_t maxLength);
bool PZEM004Tv30_checkCRC(const uint8_t *buf, uint16_t len);
void PZEM004Tv30_search(void);
esp_err_t PZEM004Tv30_sendCmd8(const uint8_t cmd, const uint16_t rAddr, const uint16_t regCount, const uint8_t slave_addr);
uint16_t PZEM004Tv30_CRC16(const uint8_t *data, uint16_t len);