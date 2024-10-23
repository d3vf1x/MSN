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
#ifndef MAIN_PCF85363A_H_
#define MAIN_PCF85363A_H_

#include <time.h>
#include <stdbool.h>
#include "i2c.h"

#define PCF85363A_I2C_ADDR      0x51 //!< I2C address
#define PCF85363A_I2C_FREQ_HZ   400000

//Register Addresses
#define PCF85363A_ADDR_100TH_SEC        0x00
#define PCF85363A_ADDR_SECONDS          0x01
#define PCF85363A_ADDR_MINUTES          0x02
#define PCF85363A_ADDR_HOURS            0x03
#define PCF85363A_ADDR_DAYS             0x04
#define PCF85363A_ADDR_WEEKDAYS         0x05
#define PCF85363A_ADDR_MONTHS           0x06
#define PCF85363A_ADDR_YEARS            0x07

#define PCF85363A_ADDR_SECOND_ALARM1    0x08
#define PCF85363A_ADDR_MINUTE_ALARM1    0x09
#define PCF85363A_ADDR_HOUR_ALARM1      0x0a
#define PCF85363A_ADDR_DAY_ALARM1       0x0b
#define PCF85363A_ADDR_MONTH_ALARM1     0x0c

#define PCF85363A_ADDR_MINUTE_ALARM2    0x0d
#define PCF85363A_ADDR_HOUR_ALARM2      0x0e
#define PCF85363A_ADDR_WEEKDAY_ALARM2   0x0f

#define PCF85363A_ADDR_ALARM_ENABLES    0x10

#define PCF85363A_ADDR_TSR1_SECONDS     0x11
#define PCF85363A_ADDR_TSR1_MINUTES     0x12
#define PCF85363A_ADDR_TSR1_HOURS       0x13
#define PCF85363A_ADDR_TSR1_DAYS        0x14
#define PCF85363A_ADDR_TSR1_MONTHS      0x15
#define PCF85363A_ADDR_TSR1_YEARS       0x16

#define PCF85363A_ADDR_TSR2_SECONDS     0x17
#define PCF85363A_ADDR_TSR2_MINUTES     0x18
#define PCF85363A_ADDR_TSR2_HOURS       0x19
#define PCF85363A_ADDR_TSR2_DAYS        0x1a
#define PCF85363A_ADDR_TSR2_MONTHS      0x1b
#define PCF85363A_ADDR_TSR2_YEARS       0x1c

#define PCF85363A_ADDR_TSR3_SECONDS     0x1d
#define PCF85363A_ADDR_TSR3_MINUTES     0x1e
#define PCF85363A_ADDR_TSR3_HOURS       0x1f
#define PCF85363A_ADDR_TSR3_DAYS        0x20
#define PCF85363A_ADDR_TSR3_MONTHS      0x21
#define PCF85363A_ADDR_TSR3_YEARS       0x22

#define PCF85363A_ADDR_TSR_MODE         0x23

#define PCF85363A_ADDR_OFFSET           0x24

#define PCF85363A_ADDR_OSCCR            0x25
#define PCF85363A_ADDR_BAT              0x26
#define PCF85363A_ADDR_PIN_IO           0x27
#define PCF85363A_ADDR_FUNCTION         0x28
#define PCF85363A_ADDR_INTA_EN          0x29
#define PCF85363A_ADDR_INTB_EN          0x2a
#define PCF85363A_ADDR_FLAGS            0x2b

#define PCF85363A_ADDR_RAM_SINGLE       0x2c

#define PCF85363A_ADDR_WD               0x2d

#define PCF85363A_ADDR_STOP_EN          0x2e

#define PCF85363A_ADDR_RST              0x2f

#define PCF85363A_ADDR_RAM_START        0x40
#define PCF85363A_ADDR_RAM_END          0x7f

// Register values
#define PCF85363A_SR    0x2c
#define PCF85363A_CPR   0xa4
#define PCF85363A_CTS   0x25

// Alarm enable register values
#define PCF85363A_WDAY_A2E  0x80
#define PCF85363A_HR_A2E    0x40
#define PCF85363A_MIN_A2E   0x20
#define PCF85363A_MON_A1E   0x10
#define PCF85363A_DAY_A1E   0x08
#define PCF85363A_HR_A1E    0x04
#define PCF85363A_MIN_A1E   0x02
#define PCF85363A_SEC_A1E   0x01

// Interrupt enable register values
#define PCF85363A_ILPA      0x80
#define PCF85363A_PIEA      0x40
#define PCF85363A_OIEA      0x20
#define PCF85363A_A1IEA     0x10
#define PCF85363A_A2IEA     0x08
#define PCF85363A_TSRIEA    0x04
#define PCF85363A_BSIEA     0x02
#define PCF85363A_WDIEA     0x01

// Flags register values
#define PCF85363A_PIF       0x80
#define PCF85363A_A2F       0x40
#define PCF85363A_A1F       0x20
#define PCF85363A_WDF       0x10
#define PCF85363A_BSF       0x08
#define PCF85363A_TSR3F     0x04
#define PCF85363A_TSR2F     0x02
#define PCF85363A_TSR1F     0x01

// PIN_IO register Values
#define PCF85363A_INTAPM_CLK    0x00
#define PCF85363A_INTAPM_BT     0x01
#define PCF85363A_INTAPM_INTA   0x02
#define PCF85363A_INTAPM_HIZ    0x03
#define PCF85363A_TSPM_DE       0x00
#define PCF85363A_TSPM_INTB     0x04
#define PCF85363A_TSPM_CLK      0x80
#define PCF85363A_TSPM_INPUT    0xc0
#define PCF85363A_TSIM          0x10
#define PCF85363A_TSL           0x20
#define PCF85363A_TSPULL        0x40
#define PCF85363A_CLKPM         0x80

// Oscillator control register values
#define PCF85363A_OSCCR_CLKIV    0x80
#define PCF85363A_OSCCR_OFFM     0x40
#define PCF85363A_OSCCR_12_24    0x20
#define PCF85363A_OSCCR_LOWJ     0x10
#define PCF85363A_OSCCR_OSCD_NORMAL     0x00
#define PCF85363A_OSCCR_OSCD_LOW        0x04
#define PCF85363A_OSCCR_OSCD_HIGH       0xc0
#define PCF85363A_OSCCR_OSCD_CL_7PF     0x00
#define PCF85363A_OSCCR_OSCD_CL_6PF     0x01
#define PCF85363A_OSCCR_OSCD_CL_125PF   0x03


#define CHECK_ARG(ARG) do { if (!ARG) return ESP_ERR_INVALID_ARG; } while (0)
#define RETURN_ON_ERROR(RES) do { if (RES != ESP_OK) return RES; } while (0)

/**
 * @brief Converts a BCD representation to decimal representation
 * @param val BCD value
 * @return decimal representation
 */
uint8_t bcd2dec(uint8_t val);

/**
 * @brief Converts a decimal representation to BCD representation
 * @param val decimal value
 * @return BCD representation
 */
uint8_t dec2bcd(uint8_t val);

/**
 * @brief Initializes the RTC device
 * @param bus I2C bus handle
 * @param dev_handle Pointer to the device handle
 * @return ESP_OK on success, ESP erroers on failure
 */
esp_err_t pcf85363a_init(i2c_master_bus_handle_t *bus,  i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Configures the RTC device (setting the oscillator control register and the pin I/O register)
 * @param dev I2C device handle
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_config(i2c_master_dev_handle_t* dev);

/**
 * @brief Resets the RTC device (clears all registers)
 * @param dev I2C device handle
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_reset(i2c_master_dev_handle_t* dev);

/**
 * @brief Sets the time on the RTC device
 * @param dev I2C device handle
 * @param time Pointer to the time structure to be set
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_set_time(i2c_master_dev_handle_t* dev, const struct tm *time);

/**
 * @brief Gets the time from the RTC device
 * @param dev I2C device handle
 * @param time Pointer to the time structure
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_get_time(i2c_master_dev_handle_t* dev, struct tm *time);

/**
 * @brief Reads the RAM from the RTC device
 * @param dev I2C device handle
 * @param addr Address of the RAM to read
 * @param data Pointer to the data buffer
 * @param len Length of the data buffer
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_read_ram(i2c_master_dev_handle_t* dev, uint8_t addr, uint8_t *data, uint8_t len);

/**
 * @brief Writes the RAM to the RTC device
 * @param dev I2C device handle
 * @param addr Address of the RAM to write
 * @param data Pointer to the data buffer
 * @param len Length of the data buffer
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_write_ram(i2c_master_dev_handle_t* dev, uint8_t addr, const uint8_t *data, uint8_t len);

/**
 * @brief clear alarm 1 flag
 * @param dev I2C device handle
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_clear_alarm_1_flag(i2c_master_dev_handle_t* dev);

/**
 * @brief set the alarm 1 of the rtc
 * @param dev I2C device handle
 * @param time Pointer to the time structure
 * @return ESP_OK on success, ESP errors on failure
 */
esp_err_t pcf85363a_set_alarm1(i2c_master_dev_handle_t* dev, struct tm *alrm_time);
#endif /* MAIN_PCF85363A_H_ */
