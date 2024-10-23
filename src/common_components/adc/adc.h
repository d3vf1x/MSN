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
#ifndef ADC_H_
#define ADC_H_

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

/**
 * @brief Read the battery voltage. But first, you need to call init_adc() to initialize the ADC.
 * @return The battery voltage in mV
 */
esp_err_t read_battery_voltage(int *voltage);


/**
 * @brief Initialize the ADC and the calibration scheme
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t init_adc();

/**
 * @brief Deinitialize the ADC and the calibration scheme
 */
esp_err_t deinit_adc();
#endif