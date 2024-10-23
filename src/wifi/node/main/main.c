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
 *  https://github.com/espressif/esp-idf
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "board.h"
#include "bme680.h"
#include "pcf85363a.h"
#include "adc.h"
#include "wifi.h"
#include "data.h"

static const char *TAG = "sensor_board";

#define HOST_IP_ADDR "192.168.4.1" // IP address of the server
#define HOST_PORT 3333             // Port of the server

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_H2E_IDENTIFIER ""
#define ESP_WIFI_MAXIMUM_RETRY 5
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

/**
 * @brief Event handler for WiFi events
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
        blinking();
        shutdown_board();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize the wifi station and connect to the access point
 *
 */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    SHUTDOWN_ON_FAIL(esp_netif_init(), "Could not initialize TCP/IP network interface");

    SHUTDOWN_ON_FAIL(esp_event_loop_create_default(), "Could not create default event loop");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    SHUTDOWN_ON_FAIL(esp_wifi_init(&cfg), "Could not initialize WiFi stack");

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    SHUTDOWN_ON_FAIL(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &event_handler,
                                                         NULL,
                                                         &instance_any_id),
                     "Could not register event handler");
    SHUTDOWN_ON_FAIL(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &event_handler,
                                                         NULL,
                                                         &instance_got_ip),
                     "Could not register event handler");

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = ESP_WIFI_H2E_IDENTIFIER,
        },
    };
    SHUTDOWN_ON_FAIL(esp_wifi_set_mode(WIFI_MODE_STA), "Could not set WiFi mode");
    SHUTDOWN_ON_FAIL(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), "Could not set WiFi configuration");
    SHUTDOWN_ON_FAIL(esp_wifi_start(), "Could not start WiFi");

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 ESP_WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ESP_WIFI_SSID);
        blinking();
        shutdown_board();
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        blinking();
        shutdown_board();
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

static int16_t float_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static uint16_t float_to_u16(float temp)
{
    return (uint16_t)(temp * 100);
}

void app_main(void)
{
    SHUTDOWN_ON_FAIL(init_board(), "Could not initialize board");

    SHUTDOWN_ON_FAIL(init_rtc(), "Could not initialize RTC");
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    while (bme680_is_measuring(sensor))
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for sensor");
    }

    SHUTDOWN_ON_FAIL(bme680_get_results_float(sensor, &values), "Could not get results from BME680 sensor");
    ESP_LOGI(TAG, "%.3f BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
             (double)sdk_system_get_time() * 1e-3,
             values.temperature, values.humidity,
             values.pressure, values.gas_resistance);

    payload_t payload = {
        .temperature = float_to_s16(values.temperature),
        .humidity = float_to_u16(values.humidity),
        .voltage = voltage};

    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(HOST_PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        blinking();
        shutdown_board();
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, HOST_PORT);

    int err = sendto(sock, &payload, sizeof(payload_t), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        blinking();
        shutdown_board();
    }
    ESP_LOGI(TAG, "Message sent");

    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

    // Error occurred during receiving
    if (len < 0)
    {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        blinking();
        shutdown_board();
    }
    // Data received
    else
    {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
        ESP_LOGI(TAG, "%s", rx_buffer);
        if (strncmp(rx_buffer, "OK: ", 4) == 0)
        {
            ESP_LOGI(TAG, "Received expected message, reconnecting");
            blinking();
            shutdown_board();
        }
    }

    if (sock != -1)
    {
        ESP_LOGE(TAG, "Shutting down socket...");
        shutdown(sock, 0);
        close(sock);
    }
    shutdown_board();
}
