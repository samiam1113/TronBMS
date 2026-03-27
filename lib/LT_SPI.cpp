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
SPIClass ltc_spi(HSPI);

// Reads and sends a byte
void spi_transfer_byte(uint8_t cs_pin, uint8_t tx, uint8_t *rx)
{
    output_low(cs_pin);
    *rx = ltc_spi.transfer(tx);
    output_high(cs_pin);
}

// Reads and sends a word
void spi_transfer_word(uint8_t cs_pin, uint16_t tx, uint16_t *rx)
{
    union { uint8_t b[2]; uint16_t w; } data_tx;
    union { uint8_t b[2]; uint16_t w; } data_rx;
    data_tx.w = tx;

    output_low(cs_pin);
    data_rx.b[1] = ltc_spi.transfer(data_tx.b[1]);
    data_rx.b[0] = ltc_spi.transfer(data_tx.b[0]);
    *rx = data_rx.w;
    output_high(cs_pin);
}

// Reads and sends a byte array
void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t length)
{
    output_low(cs_pin);
    for (int8_t i = (length - 1); i >= 0; i--)
        rx[i] = ltc_spi.transfer(tx[i]);
    output_high(cs_pin);
}

// Initialise HSPI on LTC pins at 1MHz MODE3
// Called once before any LTC communication
void spi_enable(uint8_t spi_clock_divider)
{
    // cs_pin and SCLK idle state setup
    pinMode(LTC_CS_PIN,   OUTPUT); digitalWrite(LTC_CS_PIN,   HIGH);
    pinMode(LTC_SCLK_PIN, OUTPUT); digitalWrite(LTC_SCLK_PIN, HIGH); // MODE3 idle high

    // -1 for CS: LTC681x library manages CS manually via cs_low()/cs_high()
    ltc_spi.begin(LTC_SCLK_PIN, LTC_MISO_PIN, LTC_MOSI_PIN, -1);
    ltc_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
}

void spi_disable()
{
    ltc_spi.endTransaction();
    ltc_spi.end();
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