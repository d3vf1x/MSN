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
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

i2c_master_bus_handle_t* i2c_configure_bus(uint8_t i2c_port, gpio_num_t scl_pin, gpio_num_t sda_pin)
{

    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t* bus_handle;
    if ((bus_handle = malloc (sizeof(i2c_master_bus_handle_t))) == NULL)
        return NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, bus_handle));

    return bus_handle;
}

i2c_master_dev_handle_t* i2c_add_device(i2c_master_bus_handle_t* bus, uint8_t addr, uint32_t clk_speed)
{

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = clk_speed,
    };

    i2c_master_dev_handle_t* dev_handle;
     if ((dev_handle = malloc (sizeof(i2c_master_dev_handle_t))) == NULL)
        return NULL;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &dev_cfg, dev_handle));

    return dev_handle;
}


esp_err_t i2c_write_bytes(i2c_master_dev_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !data)
        return false;

   
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    esp_err_t result = i2c_master_transmit(*dev, buf, len + 1, 100);

    return result;
}

esp_err_t i2c_read_bytes(i2c_master_dev_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!dev || !data)
        return false;

   
    esp_err_t result = i2c_master_transmit(*dev, &reg, 1, 100);
    if (result != ESP_OK)
    {
        return result;
    }

    result = i2c_master_receive(*dev, data, len, 100);
    if (result != ESP_OK)
    {
        return result;
    }

    return ESP_OK;
}
