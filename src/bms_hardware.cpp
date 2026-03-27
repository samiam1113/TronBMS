#include <Arduino.h>
#include <SPI.h>
#include "bms_config.h"
extern SPIClass ltc_spi;


void cs_low(uint8_t /*pin*/) {
    digitalWrite(LTC_CS_PIN, LOW);
}

void cs_high(uint8_t /*pin*/) {
    digitalWrite(LTC_CS_PIN, HIGH);
}

// Delay 
// delay_u is used for the isoSPI wake-up pulse timing (µs).

void delay_u(uint16_t us) {
    delayMicroseconds(us);
}

// SPI write array 
// Writes `len` bytes from `data` with no read-back.

void spi_write_array(uint8_t len, uint8_t* data) {
    ltc_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    for (uint8_t i = 0; i < len; i++) {
        ltc_spi.transfer(data[i]);
    }
    ltc_spi.endTransaction();
}

// SPI write then read 
// Writes tx_data (tx_len bytes) then reads back rx_len bytes into rx_data.

void spi_write_read(uint8_t* tx_data, uint8_t tx_len,
                    uint8_t* rx_data, uint8_t rx_len) {
    ltc_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    for (uint8_t i = 0; i < tx_len; i++) {
        ltc_spi.transfer(tx_data[i]);
    }
    for (uint8_t i = 0; i < rx_len; i++) {
        rx_data[i] = ltc_spi.transfer(0xFF);
    }
    ltc_spi.endTransaction();
}

// SPI read single byte 
// Used by LTC681x_pollAdc() to check the SDO line.

uint8_t spi_read_byte(uint8_t tx_data) {
    ltc_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    uint8_t rx = ltc_spi.transfer(tx_data);
    ltc_spi.endTransaction();
    return rx;
}