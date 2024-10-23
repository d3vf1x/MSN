/*
 *  Copyright 2024 Tom Wahl 
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This project is based on 
 *  - https://github.com/TobleMiner/lmic-esp-idf
 *  - https://github.com/matthijskooijman/arduino-lmic
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "lmic.h"
#include "hal.h"
#include "esp_sleep.h"

#include "board.h"

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf);


// This should also be in little endian format, see above.
static const u1_t DEVEUI[8] = {0x27, 0xF8, 0x4C, 0x72, 0x50, 0xFC, 0x0F, 0x03};
void os_getDevEui(u1_t *buf);

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = {0x98, 0x25, 0xa5, 0x14, 0x43, 0x7c, 0x57, 0x0d, 0x66, 0xb8, 0x23, 0x0d, 0x3c, 0xc8, 0x0c, 0x0c};

void os_getDevKey(u1_t *buf);

int16_t getSnrTenfold();

int16_t getRssi(int8_t snr);