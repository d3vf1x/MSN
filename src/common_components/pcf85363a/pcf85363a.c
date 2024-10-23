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
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pcf85363a.h"

/**
 * @brief Convert BCD to decimal
 * 
 */
uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

esp_err_t pcf85363a_init(i2c_master_bus_handle_t *bus,  i2c_master_dev_handle_t *dev_handle)
{
    CHECK_ARG(bus);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF85363A_I2C_ADDR,
        .scl_speed_hz = PCF85363A_I2C_FREQ_HZ,
    };

    return i2c_master_bus_add_device(*bus, &dev_cfg, dev_handle);
}

esp_err_t pcf85363a_reset(i2c_master_dev_handle_t *dev_handle)
{
	CHECK_ARG(dev_handle);

	uint8_t data[1];
	data[0] = PCF85363A_SR;
	
	return i2c_write_bytes(dev_handle, PCF85363A_ADDR_RST, data, 1);
}

esp_err_t pcf85363a_set_time(i2c_master_dev_handle_t *dev, const struct tm *time)
{
    CHECK_ARG(dev);
    CHECK_ARG(time);

    uint8_t data[8];
    esp_err_t status;
   
    //write stop
    data[0] = 0x01;
    status = i2c_write_bytes(dev, PCF85363A_ADDR_STOP_EN, data, 1);
    if(status != ESP_OK) return status;

    //clear presacaler
    data[0] = PCF85363A_CPR;
    status = i2c_write_bytes(dev, PCF85363A_ADDR_RST, data, 1);
    if(status != ESP_OK) return status;

    //wrote start

     /* time/date data */
    data[0] = 0; //reset 100th seconds
    data[1] = dec2bcd(time->tm_sec);
    data[2] = dec2bcd(time->tm_min);
    data[3] = dec2bcd(time->tm_hour);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_wday);		// tm_wday is 0 to 6
    data[6] = dec2bcd(time->tm_mon + 1);	// tm_mon is 0 to 11
    data[7] = dec2bcd(time->tm_year - 2000);
    status = i2c_write_bytes(dev, PCF85363A_ADDR_100TH_SEC, data, 8);
    if(status != ESP_OK) return status;

    //write stop
    data[0] = 0x00;
    return i2c_write_bytes(dev, PCF85363A_ADDR_STOP_EN, data, 1); 
}

esp_err_t pcf85363a_get_time(i2c_master_dev_handle_t* dev, struct tm *time)
{
    CHECK_ARG(dev);
    CHECK_ARG(time);

    uint8_t data[7];

    /* read time */
    RETURN_ON_ERROR(i2c_read_bytes(dev, PCF85363A_ADDR_SECONDS, data, 7));

    /* convert to unix time structure */
    ESP_LOGD("", "data=%02x %02x %02x %02x %02x %02x %02x",
                 data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    time->tm_sec = bcd2dec(data[0] & 0x7F);
    time->tm_min = bcd2dec(data[1] & 0x7F);
    time->tm_hour = bcd2dec(data[2] & 0x3F);
    time->tm_mday = bcd2dec(data[3] & 0x3F);
    time->tm_wday = bcd2dec(data[4] & 0x07);		// tm_wday is 0 to 6
    time->tm_mon  = bcd2dec(data[5] & 0x1F) - 1;	// tm_mon is 0 to 11
    time->tm_year = bcd2dec(data[6]) + 2000;
    time->tm_isdst = 0;

    return ESP_OK;
}

esp_err_t pcf85363a_clear_alarm_1_flag(i2c_master_dev_handle_t* dev){
    uint8_t flags[1] = {0};
    RETURN_ON_ERROR(i2c_read_bytes(dev, PCF85363A_ADDR_FLAGS, flags, 1));
    ESP_LOGI(pcTaskGetName(0), "Flags at startup %#02x", flags[0]);
    flags[0] &= ~(PCF85363A_A1F); //clear alarm1 flag 
    RETURN_ON_ERROR(i2c_write_bytes(dev, PCF85363A_ADDR_FLAGS, flags, 1)); 

    return ESP_OK;
}


esp_err_t pcf85363a_set_alarm1(i2c_master_dev_handle_t* dev, struct tm *time){
    uint8_t data[5] = {0};
    uint8_t alarm_enables = 0;
    if(time->tm_sec != -1){
        data[0] = dec2bcd(time->tm_sec);
        alarm_enables |= PCF85363A_SEC_A1E;
    }
    if(time->tm_min != -1){
        data[1] = dec2bcd(time->tm_min);
        alarm_enables |= PCF85363A_MIN_A1E;
    }
    if(time->tm_hour != -1){
        data[2] = dec2bcd(time->tm_hour);
        alarm_enables |= PCF85363A_HR_A1E;
    }
    if(time->tm_mday != -1){
        data[3] = dec2bcd(time->tm_mday);
        alarm_enables |= PCF85363A_DAY_A1E;
    }
    if(time->tm_wday != -1){
        data[4] = dec2bcd(time->tm_wday);
        alarm_enables |= PCF85363A_WDAY_A2E;
    }
    ESP_LOGI(pcTaskGetName(0), "Setting alarm1 enabled %#02x",  alarm_enables);
    RETURN_ON_ERROR(i2c_write_bytes(dev, PCF85363A_ADDR_SECOND_ALARM1, data, 5));
 
    //read current alarm enables
    data[0] = 0x00;
    RETURN_ON_ERROR(i2c_read_bytes(dev, PCF85363A_ADDR_ALARM_ENABLES, data, 1));
    ESP_LOGI(pcTaskGetName(0), "reading alarm1 enable bites %#02x", data[0]);
    data[0] &= 0xe0; //clear alarm1 bits
    data[0] |= alarm_enables; //set new alarm1 bits to current alarm enable bits
    ESP_LOGI(pcTaskGetName(0), "writing alarm1 enable bites %#02x", data[0]);
    RETURN_ON_ERROR(i2c_write_bytes(dev, PCF85363A_ADDR_ALARM_ENABLES, data, 1));


    //activate alarm1 interrupt
    data[0] = 0x00;
    RETURN_ON_ERROR(i2c_read_bytes(dev, PCF85363A_ADDR_INTA_EN, data, 1));
    data[0] |= (PCF85363A_A1IEA | PCF85363A_ILPA); //activate alarm1 interrupt
    ESP_LOGI(pcTaskGetName(0), "writing alarm1 interrupt bites %#02x", data[0]);
    RETURN_ON_ERROR(i2c_write_bytes(dev, PCF85363A_ADDR_INTA_EN, data, 1));

    data[0] = 0x00;
    RETURN_ON_ERROR(i2c_read_bytes(dev, PCF85363A_ADDR_FLAGS, data, 1));
    data[0] &= ~(PCF85363A_A1F); //clear alarm1 flag 
    ESP_LOGI(pcTaskGetName(0), "clear alarm1 flag %#02x", data[0]);
    RETURN_ON_ERROR(i2c_write_bytes(dev, PCF85363A_ADDR_FLAGS, data, 1)); 

    return ESP_OK;
}

esp_err_t pcf85363a_write_ram(i2c_master_dev_handle_t* dev, uint8_t addr, const uint8_t *data, uint8_t len)
{
    CHECK_ARG(dev);
    CHECK_ARG(data);
    CHECK_ARG(len);
    addr = PCF85363A_ADDR_RAM_START + addr;
    ESP_LOGI(pcTaskGetName(0), "Writing data to addr=%02x", addr);
    if (addr + len > PCF85363A_ADDR_RAM_END + 1){
        ESP_LOGE("RAM", "Address out of range");
        return ESP_ERR_INVALID_ARG;
    };


    return i2c_write_bytes(dev, addr, (uint8_t *) data, len);
}

esp_err_t pcf85363a_read_ram(i2c_master_dev_handle_t* dev, uint8_t addr, uint8_t *data, uint8_t len)
{
    CHECK_ARG(dev);
    CHECK_ARG(data);
    CHECK_ARG(len);
    addr = PCF85363A_ADDR_RAM_START + addr;
    ESP_LOGI(pcTaskGetName(0), "Reading data from addr=%02x", addr);
    if (addr + len > PCF85363A_ADDR_RAM_END + 1){
        ESP_LOGE("RAM", "Address out of range");
        return ESP_ERR_INVALID_ARG;
    };


    return i2c_read_bytes(dev, addr, data, len);
}

esp_err_t pcf85363a_config(i2c_master_dev_handle_t* dev)
{
    CHECK_ARG(dev);

   	uint8_t data[1];
	data[0] = PCF85363A_OSCCR_OSCD_CL_6PF | PCF85363A_OSCCR_OSCD_LOW; // load capacitance 6pF(1:0), low drive (3:2)
    RETURN_ON_ERROR(i2c_write_bytes(dev, PCF85363A_ADDR_OSCCR, data, 1));

    data[0] = 0x02;
    //data[0] = PCF85363A_INTAPM_INTA | PCF85363A_CLKPM; //Pin INTA is used as interrupt output, clock output is disabled
	return i2c_write_bytes(dev, PCF85363A_ADDR_PIN_IO, data, 1);
}
