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
#ifndef BOARD_H_
#define BOARD_H_

#include "i2c.h"
#include "driver/i2c_master.h"
#include "pcf85363a.h"

#include "esp_system.h"
#include "driver/gpio.h"

#define CLR_PIN         GPIO_NUM_1
#define LED_PIN         GPIO_NUM_11
#define LMIC_RST_PIN    GPIO_NUM_19
#define LMIC_DIO0_PIN   GPIO_NUM_3
#define LMIC_DIO1_PIN   GPIO_NUM_10
#define LMIC_DIO2_PIN   GPIO_NUM_23


#define SDA_GPIO GPIO_NUM_6
#define SCL_GPIO GPIO_NUM_7

// SPI interface definitions for ESP32
#define SPI_BUS SPI2_HOST
#define SPI_SCK_GPIO GPIO_NUM_20
#define SPI_MOSI_GPIO GPIO_NUM_21
#define SPI_MISO_GPIO GPIO_NUM_22
#define SPI_CS_RF_GPIO GPIO_NUM_18
#define SPI_CS_BME_GPIO GPIO_NUM_15



// if error occurs print error details, blink the LED and shutdown the board
#define ERROR_CHECK_SHUTDOWN(x) ({                                         \
    esp_err_t err_rc_ = (x);                                               \
    if (unlikely(err_rc_ != ESP_OK))                                       \
    {                                                                      \
        _esp_error_check_failed_without_abort(err_rc_, __FILE__, __LINE__, \
                                              __ASSERT_FUNC, #x);          \
        blinking();                                                        \
        shutdown_board();                                                        \
    }                                                                      \
})

// if error occurs print the msg as error, blink the LED and shutdown the board
#define SHUTDOWN_ON_FAIL(x, msg) ({  \
    esp_err_t err_rc_ = (x);         \
    if (unlikely(err_rc_ != ESP_OK)) \
    {                                \
        ESP_LOGE(TAG, msg);          \
        blinking();                  \
        shutdown_board();            \
    }                                \
})

/**
 * @brief Initialize rtc ic (initialize i2c bus, add device, configure rtc)
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t init_rtc();

/**
 * @brief Reset rtc ic
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t reset_rtc();

/**
 * @brief load config data from rtc ram
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t load_data_from_rtc_ram();

esp_err_t write_to_rtc_ram(uint8_t addr, uint8_t *data, size_t size);

esp_err_t read_from_rtc_ram(uint8_t addr, uint8_t *data, size_t size);

/**
 * @brief save config data to rtc ram
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t set_wakup_alarm();

/**
 * @brief Initialize board
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t init_board();

/**
 * @brief This will set the alarm of the RTC to wake up the board before turning of the power to the board
 */
void shutdown_board();

/**
 * @brief This will blink the board LED
 */
void blinking();

void led_on();

void led_off();

uint8_t calculate_crc8(const uint8_t *data, size_t length);

#endif /* BOARD_H_ */