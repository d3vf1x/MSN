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
#include "esp_check.h"
#include "esp_log.h"
#include "pcf85363a.h"
#include "adc.h"
#include "board.h"
#include "zigbee.h"
#include "bme680.h"

static const char *TAG = "ESP_ZB_MULTI_SENSOR_MAIN";

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

void app_main(void)
{
    SHUTDOWN_ON_FAIL(init_board(), "Could not initialize board");
    SHUTDOWN_ON_FAIL(init_rtc(), "Could not initialize RTC");

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

    SHUTDOWN_ON_FAIL(bme680_force_measurement(sensor), "Could not configure BME680 sensor");

    SHUTDOWN_ON_FAIL(init_adc(), "Could not initialize ADC");
    int16_t voltage;
    SHUTDOWN_ON_FAIL(read_battery_voltage((int *)&voltage), "Could not read battery voltage");
    deinit_adc();

    SHUTDOWN_ON_FAIL(init_zigbee(), "Could not initialize Zigbee");

    while (bme680_is_measuring(sensor))
    {
        vTaskDelay(1);
    }

    bme680_values_float_t values;
    SHUTDOWN_ON_FAIL(bme680_get_results_float(sensor, &values), "Could not get results from BME680 sensor");
    ESP_LOGI(TAG, "BME680 Sensor: %.2f°C, %.2f%%,\n", values.temperature, values.humidity);

    int16_t temperature = (int16_t)(values.temperature * 100);
    uint16_t humidity = (uint16_t)(values.humidity * 100);
    ESP_LOGI(TAG, "Data ready to send: Temperature: %d, Humidity: %d, Voltage: %d", temperature, humidity, voltage);
    while (!is_ready())
    {
        vTaskDelay(1);
    }
    send_update(&voltage, &temperature, &humidity);
    ESP_LOGI(TAG, "Update sent");
}
