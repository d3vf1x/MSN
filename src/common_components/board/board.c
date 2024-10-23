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
#include "board.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *BOARD_TAG = "BOARD_SETUP";
i2c_master_dev_handle_t dev_handle;

// CRC-8 polynomial (0x07 is a commonly used polynomial for CRC-8)
#define CRC8_POLYNOMIAL 0x07

// Function to calculate the CRC-8 checksum of a byte array
uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00; // Initial value
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR the data with the CRC
        
        // Process each bit
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL; // Shift and XOR with polynomial
            } else {
                crc <<= 1; // Shift left
            }
        }
    }
    
    return crc; // Return the calculated CRC-8 value
}


esp_err_t init_board()
{
    if (esp_reset_reason() == ESP_RST_BROWNOUT)
    {
        ESP_LOGI(BOARD_TAG, "Brownout detected");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_direction(CLR_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(CLR_PIN, 0);
        while (1)
        {
            vTaskDelay(1);
        }
    }

    gpio_set_level(CLR_PIN, 1);
    gpio_set_direction(CLR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    //Set CS for LoRa to disable module by default
    //gpio_set_level(SPI_CS_RF_GPIO, 1);
    //gpio_set_direction(SPI_CS_RF_GPIO, GPIO_MODE_OUTPUT);
    
    return ESP_OK;
}

esp_err_t init_rtc()
{
    i2c_master_bus_handle_t *bus_handle = i2c_configure_bus(0, SCL_GPIO, SDA_GPIO);
    ESP_ERROR_CHECK(pcf85363a_init(bus_handle, &dev_handle));
    if (dev_handle == NULL)
    {
        ESP_LOGE(BOARD_TAG, "Could not init device descriptor.");
        return ESP_FAIL;
    }

    esp_err_t error = pcf85363a_clear_alarm_1_flag(&dev_handle);
    if (error != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not clear alarm flag");
        return error;
    }

    error = pcf85363a_config(&dev_handle);
    if (error != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not configure rtc.");
        return error;
    }


    return ESP_OK;
}

esp_err_t reset_rtc()
{
    if (pcf85363a_reset(&dev_handle))
    {
        ESP_LOGE(BOARD_TAG, "Could not reset device.");
        return ESP_FAIL;
    }
    if (pcf85363a_config(&dev_handle))
    {
        ESP_LOGE(BOARD_TAG, "Could not config device.");
        return ESP_FAIL;
    }

    struct tm time = {
        .tm_year =  2000,
        .tm_mon =   0, // 0-based
        .tm_mday =  0,
        .tm_hour =  0,
        .tm_min =   0,
        .tm_sec =   0
    };
    if (pcf85363a_set_time(&dev_handle, &time) != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not set time.");
        return ESP_FAIL;
    }
    ESP_LOGD(BOARD_TAG, "Set initial date time done");
    return ESP_OK;
}

esp_err_t write_to_rtc_ram(uint8_t addr, uint8_t *data, size_t size)
{
    esp_err_t error = pcf85363a_write_ram(&dev_handle, addr, data, size);
    if (error != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not write data to RAM.");
        return error;
    }
    ESP_LOGD(BOARD_TAG, "Write data to  addr=%02x RAM:", PCF85363A_ADDR_RAM_START + 0);
    ESP_LOG_BUFFER_HEX_LEVEL(BOARD_TAG, data, size, ESP_LOG_DEBUG);
    return ESP_OK;
}

esp_err_t read_from_rtc_ram(uint8_t addr, uint8_t *data, size_t size)
{
    esp_err_t error = pcf85363a_read_ram(&dev_handle, addr, data, size);
    if (error != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not read data from addr=%02x RAM:", PCF85363A_ADDR_RAM_START + 0);
        return error;
    }
    ESP_LOGD(BOARD_TAG, "Read data from RAM:");
    ESP_LOG_BUFFER_HEX_LEVEL(BOARD_TAG, data, size, ESP_LOG_DEBUG);
    return ESP_OK;
}


esp_err_t load_data_from_rtc_ram()
{
    uint8_t data[3] = {0x55, 0x22, 0x33};
    uint8_t receive[3];
    esp_err_t error;

    error = pcf85363a_read_ram(&dev_handle, 0x00, receive, 3);
    if (error != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not read data from addr=%02x RAM:", PCF85363A_ADDR_RAM_START + 0);
        ESP_ERROR_CHECK(error);
        while (1)
        {
            vTaskDelay(1);
        }
    }
    ESP_LOGD(BOARD_TAG, "Read data from RAM:");
    ESP_LOG_BUFFER_HEX_LEVEL(BOARD_TAG, receive, 3, ESP_LOG_DEBUG);

    if (data[0] != receive[0] || data[1] != receive[1] || data[2] != receive[2])
    {
        ESP_LOGE(BOARD_TAG, "RAM not equal to expected data");

        struct tm rtcinfo;
        if (pcf85363a_get_time(&dev_handle, &rtcinfo) != ESP_OK)
        {
            ESP_LOGE(BOARD_TAG, "Could not get time.");
            return ESP_FAIL;
        }
        ESP_LOGI(BOARD_TAG, "Initial Time: %04d-%02d-%02d %02d:%02d:%02d",
                 rtcinfo.tm_year, rtcinfo.tm_mon + 1,
                 rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);

        ESP_LOGD(BOARD_TAG, "Write data to  addr=%02x RAM:", PCF85363A_ADDR_RAM_START + 0);
        ESP_LOG_BUFFER_HEX_LEVEL(BOARD_TAG, data, 3, ESP_LOG_DEBUG);
        error = pcf85363a_write_ram(&dev_handle, 0x00, data, 3);
        if (error != ESP_OK)
        {
            ESP_LOGE(BOARD_TAG, "Could not write data to RAM.");
            return error;
        }

        uint8_t receive[3];
        error = pcf85363a_read_ram(&dev_handle, 0x00, receive, 3);
        if (error != ESP_OK)
        {
            ESP_LOGE(BOARD_TAG, "Could not read data from addr=%02x RAM:", PCF85363A_ADDR_RAM_START + 0);
            return error;
        }
        ESP_LOGD(BOARD_TAG, "Read data that was written to/from RAM:");
        ESP_LOG_BUFFER_HEX_LEVEL(BOARD_TAG, receive, 3, ESP_LOG_DEBUG);

        struct tm time = {
            .tm_year = 2000,
            .tm_mon = 0, // 0-based
            .tm_mday = 0,
            .tm_hour = 0,
            .tm_min = 0,
            .tm_sec = 0};

        if (pcf85363a_set_time(&dev_handle, &time) != ESP_OK)
        {
            ESP_LOGE(BOARD_TAG, "Could not set time.");
            while (1)
            {
                vTaskDelay(1);
            }
        }
        ESP_LOGI(BOARD_TAG, "Set initial date time done");
    }
    else
    {
        ESP_LOGD(BOARD_TAG, "Data still in RAM");
        struct tm rtcinfo;

        if (pcf85363a_get_time(&dev_handle, &rtcinfo) != ESP_OK)
        {
            ESP_LOGE(BOARD_TAG, "Could not get time.");
            while (1)
            {
                vTaskDelay(1);
            }
        }
        ESP_LOGD(BOARD_TAG, "got Time: %04d-%02d-%02d %02d:%02d:%02d",
                 rtcinfo.tm_year, rtcinfo.tm_mon + 1,
                 rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);
    }

    return ESP_OK;
}


esp_err_t set_wakup_alarm(struct tm *wakeup_alarm)
{
    struct tm time = {
        .tm_year = 2000,
        .tm_mon = 0, // 0-based
        .tm_mday = 0,
        .tm_hour = 0,
        .tm_min = 0,
        .tm_sec = 0
    };
    if (pcf85363a_set_time(&dev_handle, &time) != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not set time.");
        return ESP_FAIL;
    } 
    ESP_LOGD(BOARD_TAG, "set Time: %04d-%02d-%02d %02d:%02d:%02d",
             time.tm_year, time.tm_mon + 1,
             time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);


    if (pcf85363a_set_alarm1(&dev_handle, wakeup_alarm) != ESP_OK)
    {
        ESP_LOGE(BOARD_TAG, "Could not set alarm.");
        return ESP_FAIL;
    }
    ESP_LOGD(BOARD_TAG, "set alarm to: %04d-%02d-%02d %02d:%02d:%02d",
             wakeup_alarm->tm_hour, wakeup_alarm->tm_mon + 1,
             wakeup_alarm->tm_mday, wakeup_alarm->tm_hour, wakeup_alarm->tm_min, wakeup_alarm->tm_sec);
  
    return ESP_OK;
}

void shutdown_board()
{
    ESP_LOGI(BOARD_TAG, "Enabling the alarm to wake up the device again...");

    // Time/Date is always set to 2000-01-01 00:00:00
    // The alarm determines the time to wake up the device (-1 means don't care)
    struct tm wakeup_alarm = {
        .tm_year = -1,
        .tm_mon = -1,
        .tm_mday = -1,
        .tm_hour = -1,
        .tm_min = 2,
        .tm_sec = -1    
    };
    set_wakup_alarm(&wakeup_alarm);//set alarms so that the device is woken up again if any error causes the device to shutdown

    ESP_LOGI(BOARD_TAG, "Turning off the device...");
    gpio_set_level(CLR_PIN, 0);
}

void blinking()
{
    ESP_LOGI(BOARD_TAG, "Blinking LED...");
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 1);
}

void led_on()
{
    gpio_set_level(LED_PIN, 1);
}

void led_off()
{
    gpio_set_level(LED_PIN, 0);
}