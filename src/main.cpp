#include <Arduino.h>
#include <SPI.h>
#include <esp_system.h>
#include "bms_config.h"
#include "bms_thresholds.h"
#include <LTC681x.h>
#include <LTC6811.h>

 
 
void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("Hello World!");
    delay(1000);
}
 
