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
 *  This project is based on
 *  - https://github.com/TobleMiner/lmic-esp-idf
 *  - https://github.com/matthijskooijman/arduino-lmic
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "lmic.h"
#include "hal.h"
#include "esp_sleep.h"

#include "bme680.h"
#include "board.h"
#include "adc.h"
#include "lora.h"
#include "data.h"

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

static const char *TAG = "lora_node";

const lmic_pinmap lmic_pins = {
    .nss = SPI_CS_RF_GPIO,
    .rst = LMIC_RST_PIN,
    .dio = {LMIC_DIO0_PIN, LMIC_DIO1_PIN, LMIC_DIO2_PIN},
    .spi = {SPI_MISO_GPIO, SPI_MOSI_GPIO, SPI_SCK_GPIO},
    .rxtx = LMIC_UNUSED_PIN,
};

static bme680_sensor_t *sensor = 0;

payload_t payload = {
    .temperature = 0,
    .humidity = 0,
    .voltage = 0,
};

static osjob_t sendjob;
void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        ESP_LOGI(TAG, "OP_TXRXPEND, not sending");
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (xref2u1_t)&payload, sizeof(payload_t), 0);
        ESP_LOG_BUFFER_HEX(TAG, &payload, sizeof(payload_t));
        ESP_LOGI(TAG, "Packet queued");
    }
}
uint32_t otaa_transmission_counter = 0;

esp_err_t update_seq_number()
{
    otaa_transmission_counter++;
    ESP_LOGI(TAG, "Saving current count:");
    ESP_LOG_BUFFER_HEX(TAG, &otaa_transmission_counter, sizeof(otaa_transmission_counter));
    RETURN_ON_ERROR(write_to_rtc_ram(0, (uint8_t *)&otaa_transmission_counter, 4));

    ESP_LOGI(TAG, "Saving seqno down: %lu", LMIC.seqnoDn);
    ESP_LOG_BUFFER_HEX(TAG, &LMIC.seqnoDn, 4);
    RETURN_ON_ERROR(write_to_rtc_ram(4, (uint8_t *)&LMIC.seqnoDn, 4));

    ESP_LOGI(TAG, "updating seqno to %lu", LMIC.seqnoUp);
    ESP_LOG_BUFFER_HEX(TAG, &LMIC.seqnoUp, 4);
    RETURN_ON_ERROR(write_to_rtc_ram(8, (uint8_t *)&LMIC.seqnoUp, 4));
    return ESP_OK;
}


esp_err_t save_session_data()
{
    RETURN_ON_ERROR(reset_rtc());
    ESP_LOGI(TAG, "Saving session data");

    int8_t offset = 0;
    ESP_LOGI(TAG, "Saving current count:");
    ESP_LOG_BUFFER_HEX(TAG, &otaa_transmission_counter, sizeof(otaa_transmission_counter));
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&otaa_transmission_counter, 4));
    offset += 4;

    ESP_LOGI(TAG, "Saving seqno down:");
    ESP_LOG_BUFFER_HEX(TAG, &LMIC.seqnoDn, 4);
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&LMIC.seqnoDn, 4));
    offset += 4;

    ESP_LOGI(TAG, "Saving seqno up:");
    ESP_LOG_BUFFER_HEX(TAG, &LMIC.seqnoUp, 4);
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&LMIC.seqnoUp, 4));
    offset += 4;

    ESP_LOGI(TAG, "Saving Dev Addr:");
    ESP_LOG_BUFFER_HEX(TAG, &LMIC.devaddr, 4);
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&LMIC.devaddr, 4));
    offset += 4;

    ESP_LOGI(TAG, "Saving nwkKey:");
    ESP_LOG_BUFFER_HEX(TAG, LMIC.nwkKey, 16);
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&LMIC.nwkKey, 16));
    offset += 16;

    ESP_LOGI(TAG, "Saving artKey:");
    ESP_LOG_BUFFER_HEX(TAG, LMIC.artKey, 16);
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&LMIC.artKey, 16));
    offset += 16;

    ESP_LOGI(TAG, "Saving netid:");
    ESP_LOG_BUFFER_HEX(TAG, &LMIC.netid, 4);
    RETURN_ON_ERROR(write_to_rtc_ram(offset, (uint8_t *)&LMIC.netid, 4));


    return ESP_OK;
}

esp_err_t load_session_data()
{
    uint32_t stored_count = 0;

    int8_t offset = 0;
    ESP_LOGI(TAG, "Reading current count from rtc:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&stored_count, 4));
    ESP_LOG_BUFFER_HEX(TAG, &stored_count, sizeof(stored_count));
    offset += 4;

    if (stored_count == 0)
    {
        ESP_LOGI(TAG, "No data stored in rtc");
        reset_rtc();
        return ESP_ERR_NOT_FOUND;
    }


    devaddr_t dev_addr;
    u4_t net_id;
    u1_t nwk_key[16];
    u1_t art_key[16];
    u4_t seq_no_up;
    u4_t seq_no_down;
 

    ESP_LOGI(TAG, "Reading seqno down:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&seq_no_down, 4));
    ESP_LOG_BUFFER_HEX(TAG, &seq_no_down, 4);
    offset += 4;

    ESP_LOGI(TAG, "Reading seqno:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&seq_no_up, 4));
    ESP_LOG_BUFFER_HEX(TAG, &seq_no_up, 4);
    offset += 4;

    if (stored_count !=  seq_no_up)
    {
        ESP_LOGE(TAG, "Seqno down does not match %lu != %lu", stored_count, seq_no_up);
        return ESP_ERR_NOT_FOUND;
    }
    otaa_transmission_counter = stored_count;


    ESP_LOGI(TAG, "Reading dev addr:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&dev_addr, 4));
    ESP_LOG_BUFFER_HEX(TAG, &dev_addr, 4);
    offset += 4;

    ESP_LOGI(TAG, "Reading nwkKey:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&nwk_key, 16));
    ESP_LOG_BUFFER_HEX(TAG, &nwk_key, 16);
    offset += 16;

    ESP_LOGI(TAG, "Reading artKey:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&art_key, 16));
    ESP_LOG_BUFFER_HEX(TAG, &art_key, 16);
    offset += 16;

    ESP_LOGI(TAG, "Reading netid:");
    RETURN_ON_ERROR(read_from_rtc_ram(offset, (uint8_t *)&net_id, 4));
    ESP_LOG_BUFFER_HEX(TAG, &net_id, 4);

    if (dev_addr == 0x000000)
    {
        ESP_LOGI(TAG, "No data stored in rtc");
        reset_rtc();
        return ESP_ERR_NOT_FOUND;
    }

    LMIC_setSession(net_id, dev_addr, nwk_key, art_key);
    LMIC.seqnoUp = seq_no_up;
    LMIC.seqnoDn = seq_no_down;
    return ESP_OK;
}

void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        ESP_LOGI(TAG, "EV_SCAN_TIMEOUT");
        break;
    case EV_BEACON_FOUND:
        ESP_LOGI(TAG, "EV_BEACON_FOUND");
        break;
    case EV_BEACON_MISSED:
        ESP_LOGI(TAG, "EV_BEACON_MISSED");
        break;
    case EV_BEACON_TRACKED:
        ESP_LOGI(TAG, "EV_BEACON_TRACKED");
        break;
    case EV_JOINING:
        ESP_LOGI(TAG, "EV_JOINING");
        break;
    case EV_JOINED:
        ESP_LOGI(TAG, "EV_JOINED");

        save_session_data();
        LMIC_setAdrMode(false);
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        ESP_LOGI(TAG, "EV_RFU1");
        break;
    case EV_JOIN_FAILED:
        ESP_LOGI(TAG, "EV_JOIN_FAILED");
        blinking();
        shutdown_board();
        break;
    case EV_REJOIN_FAILED:
        ESP_LOGI(TAG, "EV_REJOIN_FAILED");
        blinking();
        shutdown_board();
        break;
    case EV_TXCOMPLETE:
        ESP_LOGI(TAG, "EV_TXCOMPLETE (includes waiting for RX windows)");
        update_seq_number();
        if (LMIC.txrxFlags & TXRX_ACK)
            ESP_LOGI(TAG, "Received ack");

        if (LMIC.dataLen > 0)
        {
            uint8_t dataLength = LMIC.dataLen;
            // bool ackReceived = LMIC.txrxFlags & TXRX_ACK;

            int16_t snrTenfold = getSnrTenfold();
            int8_t snr = snrTenfold / 10;
            int8_t snrDecimalFraction = snrTenfold % 10;
            int16_t rssi = getRssi(snr);

            uint8_t fPort = 0;
            if (LMIC.txrxFlags & TXRX_PORT)
            {
                fPort = LMIC.frame[LMIC.dataBeg - 1];
            }
            ESP_LOGI(TAG, "Received %d bytes of payload", LMIC.dataLen);
            ESP_LOGI(TAG, "Downlink received RSSI: %d dBm,  SNR: %d.%d dB, Port %d ", rssi, snr, snrDecimalFraction, fPort);

            ESP_LOGI(TAG, "Seq. Down: %d", (int)LMIC.seqnoDn);
            ESP_LOGI(TAG, "Length:  %d", (int)LMIC.dataLen);

            if (dataLength != 0)
            {
                ESP_LOGI(TAG, "Data:");
                ESP_LOG_BUFFER_HEX(TAG, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
                printf("%02x\n", LMIC.frame[LMIC.dataBeg]);
            }
        }

        LMIC_reset();
        LMIC_shutdown();
        ESP_LOGI(TAG, "SLEEP");
        shutdown_board();
        break;
    case EV_LOST_TSYNC:
        ESP_LOGI(TAG, "EV_LOST_TSYNC");
        break;
    case EV_RESET:
        ESP_LOGI(TAG, "EV_RESET");
        break;
    case EV_RXCOMPLETE:
        ESP_LOGI(TAG, "EV_RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        ESP_LOGI(TAG, "EV_LINK_DEAD");
        break;
    case EV_LINK_ALIVE:
        ESP_LOGI(TAG, "EV_LINK_ALIVE");
        break;
    default:
        ESP_LOGI(TAG, "Unknown event");
        break;
    }
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

void app_main(void)
{
    led_on();
    SHUTDOWN_ON_FAIL(init_board(), "Could not initialize board");

    SHUTDOWN_ON_FAIL(init_rtc(), "Could not initialize RTC");

    // LMIC init
    os_init(); // this will also init the spi bus that is used by the bme680 sensor

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();



    // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    sensor = bme680_init_sensor(SPI_BUS, 0, SPI_CS_BME_GPIO);
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

    LMIC_setLinkCheckMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 0.1); // Compensate for Clock Skew

    LMIC.dn2Dr = DR_SF7; // Downlink Band Data Rate
    LMIC.dn2Freq = 868100000; // Downlink Band Frequency
    LMIC.rxDelay = 3;            // Delay between TX und RX1
    LMIC_setDrTxpow(DR_SF7, 2); // Set TX Power 

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);

    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(false);    

    if (load_session_data() == ESP_OK)
    {
        ESP_LOGI(TAG, "Using saved session data (ABR mode) ");
    }
    else
    {
        ESP_LOGI(TAG, "Saved session data invalid (OTAA mode) ");
    }

    while (bme680_is_measuring(sensor))
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for sensor");
    }

    SHUTDOWN_ON_FAIL(bme680_get_results_float(sensor, &values), "Could not get results from BME680 sensor");
    ESP_LOGI(TAG, "BME680 Sensor: %.2f°C, %.2f%%", values.temperature, values.humidity);

    // pack the payload
    payload.humidity = (uint16_t)(values.humidity * 100);
    payload.temperature = (int16_t)(values.temperature * 100);
    payload.voltage = voltage;

    do_send(&sendjob);
    ESP_LOGI(TAG, "Sending data with job:%p, cb %p", &sendjob,do_send) ;
    //os_setCallback(&sendjob, do_send);

    printf("Send done\n");
    
    while (1)
    {
        os_runloop_once();
        vTaskDelay(1);
    }
}
