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
 *  This file is based on work by Espressif Systems (Shanghai) CO LTD and is subject to the Espressif Systems License Agreement.
 *  https://github.com/espressif/esp-idf
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "bme680.h"
#include "pcf85363a.h"
#include "adc.h"
#include "data.h"
#include "ble.h"

#include "mbedtls/aes.h"

payload_t payload = {
    .package_counter = 0,
    .temperature = 0,
    .humidity = 0,
    .voltage = 0
};

/**
 * @brief Encrypts the input bytes with AES-256 in CBC mode
 */
void encrypt_bytes(const uint8_t *input, uint8_t *output, const uint8_t len, uint8_t *key, uint8_t *iv)
{
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, key, 256);
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, len, iv, (unsigned char *)input, output);
    ESP_LOG_BUFFER_HEX("cbc_encrypt", output, len);
    mbedtls_aes_free(&aes);
}


esp_err_t load_session_data(){
    ESP_LOGI(TAG, "Reading current sequence number from rtc:");
    uint16_t stored_count = 0;
    uint8_t crc_stored = 0;
    RETURN_ON_ERROR(read_from_rtc_ram(0, (uint8_t *)&stored_count, 2));
    RETURN_ON_ERROR(read_from_rtc_ram(2, (uint8_t *)&crc_stored, 1));
    ESP_LOG_BUFFER_HEX(TAG, &stored_count, sizeof(stored_count));
    
    uint8_t crc_calculated = calculate_crc8((uint8_t *)&stored_count, sizeof(stored_count));


    if (stored_count == 0 || crc_stored != crc_calculated)
    {
        ESP_LOGI(TAG, "No data stored in rtc");
        reset_rtc();
        payload.package_counter = 0;
        return ESP_ERR_NOT_FOUND;
    }
    else{
        payload.package_counter = stored_count;
        return ESP_OK;
    }
}

esp_err_t update_seq_number()
{
    payload.package_counter++;
    ESP_LOGI(TAG, "Saving current count:");
    ESP_LOG_BUFFER_HEX(TAG, (uint8_t *)&payload.package_counter, sizeof(payload.package_counter));
    RETURN_ON_ERROR(write_to_rtc_ram(0, (uint8_t *)&payload.package_counter, sizeof(payload.package_counter)));

    uint8_t crc_calculated = calculate_crc8((uint8_t *)&payload.package_counter, sizeof(payload.package_counter));
    RETURN_ON_ERROR(write_to_rtc_ram(2, (uint8_t *)&crc_calculated, sizeof(crc_calculated)));

    return ESP_OK;
}


/**
 * @brief Initialize the bme680 sensor
 */
esp_err_t config_sensor(bme680_sensor_t *sensor)
{
    CHECK_ARG(sensor);
    esp_err_t err = ESP_OK;

    if (bme680_set_oversampling_rates(sensor, osr_16x, osr_none, osr_16x) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set oversampling rates");
        err = ESP_FAIL;
    }

    // Change the IIR filter size for temperature and pressure to 7.
    if (bme680_set_filter_size(sensor, iir_size_7) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set filter size");
        err = ESP_FAIL;
    }

    // Change the heater profile 0 to 200 degree Celcius for 100 ms.
    if (bme680_set_heater_profile(sensor, 0, 200, 100) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set heater profile");
        err = ESP_FAIL;
    }

    if (bme680_use_heater_profile(sensor, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not use heater profile");
        err = ESP_FAIL;
    }
    return err;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(TAG, "GAP_EVT, event %d", event);
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        ESP_LOGI(TAG, "Advertisement terminated. Stopping further advertisements.");
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "advertising start failed");
        }
        else
        {
            ESP_LOGI(TAG, "advertising start successfully");
            vTaskDelay(100 / portTICK_PERIOD_MS);// Wait for the advertisement to be send
            ESP_LOGI(TAG, "Advertisement terminated. Stopping further advertisements.");
            esp_ble_gap_stop_advertising(); // Stop further advertising
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(TAG, "Stop adv successfully");
            update_seq_number();
            shutdown_board();
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        /*esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }*/
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        break;
    }
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gatt_profile[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gatt_profile[idx].gatts_if)
            {
                if (gatt_profile[idx].gatts_cb)
                {
                    gatt_profile[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}



void app_main(void)
{
    SHUTDOWN_ON_FAIL(init_board(), "Could not initialize board");

    SHUTDOWN_ON_FAIL(init_rtc(), "Could not initialize RTC");

    //led_on();

    esp_err_t ret;

    // Init NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    SHUTDOWN_ON_FAIL(ret, "Could not initialize NVS");

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .sclk_io_num = SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};
    SHUTDOWN_ON_FAIL(spi_bus_initialize(SPI_BUS, &buscfg, SPI_DMA_CH_AUTO), "Could not initialize SPI bus");
    bme680_sensor_t *sensor = bme680_init_sensor(SPI_BUS, 0, SPI_CS_BME_GPIO);
    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "Could not initialize BME680 sensor");
        blinking();
        shutdown_board();
        return;
    }
    SHUTDOWN_ON_FAIL(config_sensor(sensor), "Could not configure BME680 sensor");

    bme680_values_float_t values;
    SHUTDOWN_ON_FAIL(bme680_force_measurement(sensor), "Could not configure BME680 sensor");

    SHUTDOWN_ON_FAIL(init_adc(), "Could not initialize ADC");
    int16_t voltage;
    SHUTDOWN_ON_FAIL(read_battery_voltage((int *)&voltage), "Could not read battery voltage");
    SHUTDOWN_ON_FAIL(deinit_adc(), "Could not deinitialize ADC");

    ESP_LOGI(TAG, "Current Battery Voltage: %d", voltage);


    if (load_session_data() == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded session data from RTC: current seqno. %d", payload.package_counter);
    }

    while (bme680_is_measuring(sensor))
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for sensor");
    }

    SHUTDOWN_ON_FAIL(bme680_get_results_float(sensor, &values), "Could not get results from BME680 sensor");
    payload.humidity = (uint16_t)(values.humidity * 100);
    payload.temperature = (int16_t)(values.temperature * 100);
    payload.voltage = voltage;

    ESP_LOGI(TAG, "BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
             values.temperature, values.humidity,
             values.pressure, values.gas_resistance);

    //Fill Payload
    raw_adv_data[offset_adv_payload] = (uint8_t)(payload.package_counter >> 8);
    raw_adv_data[offset_adv_payload + 1] = (uint8_t)(payload.package_counter & 0xFF);

    raw_adv_data[offset_adv_payload + 2] = (uint8_t)(payload.temperature >> 8);
    raw_adv_data[offset_adv_payload + 3] = (uint8_t)(payload.temperature & 0xFF);

    raw_adv_data[offset_adv_payload + 4] = (uint8_t)(payload.humidity >> 8);
    raw_adv_data[offset_adv_payload + 5] = (uint8_t)(payload.humidity & 0xFF);

    raw_adv_data[offset_adv_payload + 6] = (uint8_t)(payload.voltage >> 8);
    raw_adv_data[offset_adv_payload + 7] = (uint8_t)(payload.voltage & 0xFF);

    //ESP_LOG_BUFFER_HEX("raw_adv_data", (uint8_t *)raw_adv_data + offset_adv_data, 16);

    //Encrypt Payload with AES
    uint8_t encrypted[16] = {0};
    encrypt_bytes((uint8_t *)raw_adv_data + offset_adv_data, encrypted, 16, (uint8_t *)enc_key, (uint8_t *)enc_iv);
    memcpy((uint8_t *)raw_adv_data + offset_adv_data, encrypted, 16);
    
    //ESP_LOG_BUFFER_HEX("encrypted raw_adv_data", (uint8_t *)raw_adv_data + offset_adv_data, 16);

    SHUTDOWN_ON_FAIL(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT), "Could not release memory for BT");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        blinking();
        shutdown_board();
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret)
    {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
        blinking();
        shutdown_board();
        return;
    }

}
