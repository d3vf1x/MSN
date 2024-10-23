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
 */
#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#include <string.h>
#include <time.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

i2c_master_dev_handle_t* i2c_add_device(i2c_master_bus_handle_t* bus, uint8_t addr, uint32_t clk_speed);

i2c_master_bus_handle_t* i2c_configure_bus(uint8_t i2c_port, gpio_num_t scl_pin, gpio_num_t sda_pin);

esp_err_t i2c_read_bytes(i2c_master_dev_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

esp_err_t i2c_write_bytes(i2c_master_dev_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
#endif // __I2C_MASTER_H__