/*!
bms_hardware.cpp - Hardware abstraction layer for LTC681x on ESP32.
Adapted from Linduino/ADI reference for ESP32 HSPI peripheral.

Original Copyright 2018(c) Analog Devices, Inc. All rights reserved.
*/

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "bms_hardware.h"
#include "bms_config.h"
#include "Linduino.h"
#include "LT_SPI.h"

// ltc_spi is defined in LT_SPI.cpp
extern SPIClass ltc_spi;

// CS control — LTC681x library calls these with CS_PIN from LTC681x.h
// which is defined as LTC_CS_PIN in bms_config.h
void cs_low(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void cs_high(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
}

void delay_u(uint16_t micro)
{
    delayMicroseconds(micro);
}

void delay_m(uint16_t milli)
{
    delay(milli);
}

// Writes len bytes from data[] over SPI with no readback
// CS is managed by the caller (LTC681x.cpp calls cs_low before this)
void spi_write_array(uint8_t len, uint8_t data[])
{
    for (uint8_t i = 0; i < len; i++)
    {
        ltc_spi.transfer(data[i]);
    }
}

// Writes tx_len bytes then reads rx_len bytes
// CS is managed by the caller
void spi_write_read(uint8_t *tx_data, uint8_t tx_len,
                    uint8_t *rx_data, uint8_t rx_len)
{
    for (uint8_t i = 0; i < tx_len; i++)
    {
        ltc_spi.transfer(tx_data[i]);
    }
    for (uint8_t i = 0; i < rx_len; i++)
    {
        rx_data[i] = ltc_spi.transfer(0xFF);
    }
}

// Single byte read — used by pollAdc()
uint8_t spi_read_byte(uint8_t tx_dat)
{
    return ltc_spi.transfer(tx_dat);
}