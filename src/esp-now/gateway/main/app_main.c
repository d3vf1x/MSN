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
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"

#include "sdkconfig.h"
#include "espnow_config.h"

static const char *TAG = "ESP-NOW_Gateway";

// Your task to handle received my_data_t
void my_data_receive(const uint8_t *sender_mac_addr, const data_package_t *data)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             sender_mac_addr[0], sender_mac_addr[1], sender_mac_addr[2], sender_mac_addr[3], sender_mac_addr[4], sender_mac_addr[5]);
    ESP_LOGI(TAG, "Data from %s: Temperature %.2f°C, Humidity %.2f%%, Voltage - %.2fV",
             macStr,
             1.0 * data->temperature / 100,
             1.0 * data->humidity / 100,
             1.0 * data->voltage / 1000);
}



static QueueHandle_t s_recv_queue;

typedef struct
{
    uint8_t sender_mac_addr[ESP_NOW_ETH_ALEN];
    data_package_t data;
} recv_packet_t;

static void queue_process_task(void *p)
{
    static recv_packet_t recv_packet;

    ESP_LOGI(TAG, "Listening");
    for (;;)
    {
        if (xQueueReceive(s_recv_queue, &recv_packet, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        // Refer to user function
        my_data_receive(recv_packet.sender_mac_addr, &recv_packet.data);
    }
}

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
// #define MY_ESPNOW_WIFI_MODE WIFI_MODE_AP
// #define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
void recv_cb(const uint8_t * mac_addr, const uint8_t *data, int len)
{
    //uint8_t *mac_addr = packet_info->src_addr;
    static recv_packet_t recv_packet;
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "%d bytes incoming from %s", len, macStr);

    if (len != sizeof(data_package_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(data_package_t));
        return;
    }

    memcpy(&recv_packet.sender_mac_addr, mac_addr, sizeof(recv_packet.sender_mac_addr));
    memcpy(&recv_packet.data, data, len);
    if (xQueueSend(s_recv_queue, &recv_packet, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Queue full, discarded");
        return;
    }
}

static void init_espnow_master(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_now_init());

    // Register the master as peer
    const esp_now_peer_info_t peerInfo = {
        .channel = ESPNOW_CHANNEL,
        .ifidx = ESPNOW_WIFI_IF,
        .encrypt = true,
        .peer_addr = NODE_MAC,
        .lmk = LMK,
    };
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));

    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));

    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));


    uint8_t mac_addr[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Mac Addr: %s", macStr);
}

void app_main(void)
{
    s_recv_queue = xQueueCreate(10, sizeof(recv_packet_t));
    assert(s_recv_queue);
    BaseType_t err = xTaskCreate(queue_process_task, "recv_task", 8192, NULL, 4, NULL);
    assert(err == pdPASS);

    init_espnow_master();
}