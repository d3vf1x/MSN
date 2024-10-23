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
 *  This file is based on work by Espressif Systems and is subject to the Espressif Systems License Agreement.
 *  https://github.com/espressif/esp-zigbee-sdk
 */
#ifndef __ZIGBEE_H__
#define __ZIGBEE_H__

#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "driver/spi_common.h"
#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "esp_zigbee_core.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#endif

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

//Zigbee configuration
#define INSTALLCODE_POLICY_ENABLE false     
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 4000                                               // in milliseconds
#define HA_ESP_SENSOR_ENDPOINT 10                                        // esp temperature sensor device endpoint
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK 

#define ESP_TEMP_SENSOR_UPDATE_INTERVAL (1) /* Local sensor update interval (second) */
#define ESP_TEMP_SENSOR_MIN_VALUE (-10)     /* Local sensor min measured value (degree Celsius) */
#define ESP_TEMP_SENSOR_MAX_VALUE (80)      /* Local sensor max measured value (degree Celsius) */


#define ESP_ZB_ZED_CONFIG()                               \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,             \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = {                              \
            .ed_timeout = ED_AGING_TIMEOUT,               \
            .keep_alive = ED_KEEP_ALIVE,                  \
        },                                                \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()       \
    {                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                          \
    {                                                         \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }


/**
 * @brief sets the attributes for the clusters and reports the attributes
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_update(int16_t *voltage, int16_t *temperature, uint16_t *humidity);

/**
 * @brief Init the cluster list
 * @return the cluster list
 */
esp_zb_cluster_list_t *init_cluster();

/**
 * @brief writes the values to the attributes
 * @param endpoint the endpoint of the device
 * @param clusterID the cluster ID
 * @param attributeID the attribute ID
 * @param value the value to write
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t write_attribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value);

/**
 * @brief reports the attributes
 * @param endpoint the endpoint of the device
 * @param clusterID the cluster ID
 * @param attributeID the attribute ID
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t report_attribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID);

/**
 * @brief Initialize zigbee
 */
esp_err_t init_zigbee();

/**
 * @brief Check if the initialization of the zigbee is done
 */
bool is_ready();

#endif