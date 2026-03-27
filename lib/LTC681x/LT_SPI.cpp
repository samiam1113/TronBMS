/*!
LT_SPI.cpp - Adapted for ESP32 HSPI driving LTC6811 on custom PCB.
Original Linduino version targeted ATmega328P hardware SPI.
This version uses the ESP32 HSPI peripheral on pins defined in bms_config.h.

Original Copyright 2018(c) Analog Devices, Inc. All rights reserved.
*/

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "bms_config.h"

// HSPI instance — extern so bms_hardware.cpp and main can share it
// Add near the top, after the #includes
extern SPIClass *hspi;

void spi_transfer_byte(uint8_t cs_pin, uint8_t tx, uint8_t *rx)
{
    hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    output_low(cs_pin);
    *rx = hspi->transfer(tx);
    output_high(cs_pin);
    hspi->endTransaction();
}

void spi_transfer_word(uint8_t cs_pin, uint16_t tx, uint16_t *rx)
{
    union { uint8_t b[2]; uint16_t w; } data_tx, data_rx;
    data_tx.w = tx;
    hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    output_low(cs_pin);
    data_rx.b[1] = hspi->transfer(data_tx.b[1]);
    data_rx.b[0] = hspi->transfer(data_tx.b[0]);
    *rx = data_rx.w;
    output_high(cs_pin);
    hspi->endTransaction();
}

void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t length)
{
    hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    output_low(cs_pin);
    for (int8_t i = (length - 1); i >= 0; i--)
        rx[i] = hspi->transfer(tx[i]);
    output_high(cs_pin);
    hspi->endTransaction();
}

void spi_enable(uint8_t spi_clock_divider)
{
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
}

void spi_disable()
{
    hspi->end();
}

void spi_write(int8_t data)
{
    ltc_spi.transfer(data);
}

int8_t spi_read(int8_t data)
{
    return ltc_spi.transfer(data);
}

void quikeval_SPI_init(void)
{
    spi_enable(0); // divider arg unused on ESP32
}