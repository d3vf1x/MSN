/*
 * Copyright Tom Wahl 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This file is based on work by Espressif Systems and is subject to the Espressif Systems License Agreement.
 *  https://github.com/espressif/esp-now/
 */
#ifndef ESPNOW_CONFIG_H
#define ESPNOW_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

typedef struct __attribute__((packed)) {
    int16_t temperature;
    uint16_t humidity;
    int16_t voltage;
} data_package_t;

// Destination MAC address
// The default address is the broadcast address, which will work out of the box, but the slave will assume every tx succeeds.
// Setting to the master's address will allow the slave to determine if sending succeeded or failed.
//   note: with default config, the master's WiFi driver will log this for you. eg. I (721) wifi:mode : sta (12:34:56:78:9a:bc)
#define GATEWAY_MAC {0x40, 0x4c, 0xca, 0x57, 0xee, 0xec}

#define ESPNOW_PMK "pmk1234567890123"
#define LMK {0x70, 0x6d, 0x6b, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33}   

#define ESPNOW_CHANNEL 1


#endif // ESPNOW_CONFIG_H