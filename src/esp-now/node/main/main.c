/*
 * Copyright 2024 Tom Wahl 
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
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"

#include "espnow_config.h"
#include "board.h"
#include "bme680.h"
#include "pcf85363a.h"
#include "adc.h"

static const char *TAG = "ESP-NOW_Node";

static EventGroupHandle_t s_evt_group;

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA

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

static void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    assert(status == ESP_NOW_SEND_SUCCESS || status == ESP_NOW_SEND_FAIL);
    xEventGroupSetBits(s_evt_group, BIT(status));
}

esp_err_t init_espnow_slave(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    RETURN_ON_ERROR(ret);
    RETURN_ON_ERROR(esp_netif_init());
    RETURN_ON_ERROR(esp_event_loop_create_default());
    RETURN_ON_ERROR(esp_wifi_init(&cfg));
    RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    RETURN_ON_ERROR(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    RETURN_ON_ERROR(esp_wifi_start());

    RETURN_ON_ERROR(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));

    RETURN_ON_ERROR(esp_now_init());
    RETURN_ON_ERROR(esp_now_register_send_cb(packet_sent_cb));
    RETURN_ON_ERROR(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));

    const esp_now_peer_info_t destination = {
        .peer_addr = GATEWAY_MAC,
        .channel = ESPNOW_CHANNEL,
        .ifidx = ESPNOW_WIFI_IF,
        .encrypt = true,
        .lmk = LMK
    };
    RETURN_ON_ERROR(esp_now_add_peer(&destination));

    return ESP_OK;
}

static esp_err_t send_espnow_data(data_package_t *data)
{
    const uint8_t destination_mac[] = GATEWAY_MAC;

    // Send it
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             destination_mac[0], destination_mac[1], destination_mac[2], destination_mac[3], destination_mac[4], destination_mac[5]);
    ESP_LOGI(TAG, "Sending %u bytes to %s", sizeof(data_package_t), macStr);

    esp_err_t err = esp_now_send(destination_mac, (uint8_t *)data, sizeof(data_package_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error (%d)", err);
        return ESP_FAIL;
    }

    // Wait for callback function to set status bit
    EventBits_t bits = xEventGroupWaitBits(s_evt_group, BIT(ESP_NOW_SEND_SUCCESS) | BIT(ESP_NOW_SEND_FAIL), pdTRUE, pdFALSE, 2000 / portTICK_PERIOD_MS);
    if (!(bits & BIT(ESP_NOW_SEND_SUCCESS)))
    {
        if (bits & BIT(ESP_NOW_SEND_FAIL))
        {
            ESP_LOGE(TAG, "Send error");
            return ESP_FAIL;
        }
        ESP_LOGE(TAG, "Send timed out");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Sent!");
    return ESP_OK;
}

void app_main(void)
{
    SHUTDOWN_ON_FAIL(init_board(), "Could not initialize board");

    SHUTDOWN_ON_FAIL(init_rtc(), "Could not initialize RTC");

    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

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

    init_espnow_slave();

    SHUTDOWN_ON_FAIL(bme680_get_results_float(sensor, &values), "Could not get results from BME680 sensor");

    static data_package_t payload;
    payload.humidity = (uint16_t)(values.humidity * 100);
    payload.temperature = (int16_t)(values.temperature * 100);
    payload.voltage = voltage;

    ESP_ERROR_CHECK(send_espnow_data(&payload));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    shutdown_board();
}