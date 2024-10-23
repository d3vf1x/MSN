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
 *  This file is based on work by Espressif Systems (Shanghai) CO LTD and is subject to the Espressif Systems License Agreement.
 *  https://github.com/espressif/esp-idf
 */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "mbedtls/aes.h"
#include "config.h"

#define GATTC_TAG "BLE_GATEWAY"
#define REMOTE_SERVICE_UUID 0x181A
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static const uint16_t GATTS_CHAR_UUID_TEMPERATUR = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_HUMIDITY = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_VOLTAGE = 0xFF03;
static const char remote_device_name[] = "ESP_GATTS_DEMO";
static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

void decrypt_bytes(const uint8_t *input, uint8_t *output, const uint8_t len, uint8_t *key, uint8_t *iv)
{
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, key, 256);
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, len, iv, (unsigned char *)input, output);
    //ESP_LOG_BUFFER_HEX("cbc_decrypt", output, len);
    mbedtls_aes_free(&aes);
}

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = REMOTE_SERVICE_UUID,
    },
};

static esp_bt_uuid_t remote_filter_char_a_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = GATTS_CHAR_UUID_TEMPERATUR,
    },
};

static esp_bt_uuid_t remote_filter_char_b_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = GATTS_CHAR_UUID_HUMIDITY,
    },
};

static esp_bt_uuid_t remote_filter_char_c_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = GATTS_CHAR_UUID_VOLTAGE,
    },
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

// One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret)
        {
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:
    {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        ESP_LOG_BUFFER_HEX(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        if (mtu_ret)
        {
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
    {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
        {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        }
        else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
        {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0)
            {
                ESP_LOGI(GATTC_TAG, "Attr count %d", count);
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }
                else
                {
                    status = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                                                          p_data->search_cmpl.conn_id,
                                                                                          gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                                          gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                                          remote_filter_char_a_uuid,
                                                                                          char_elem_result,
                                                                                          &count));
                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }
                    ESP_LOGI(GATTC_TAG, "Get char a: count = %d", count);
                    for (int i = 0; i < count; ++i)
                    {
                        if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                        {
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[i].char_handle;
                            esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[i].char_handle);
                            ESP_LOGI(GATTC_TAG, "UUID16: %x", char_elem_result[i].uuid.uuid.uuid16);
                        }
                    }

                    status = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                                                          p_data->search_cmpl.conn_id,
                                                                                          gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                                          gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                                          remote_filter_char_b_uuid,
                                                                                          char_elem_result,
                                                                                          &count));
                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }
                    ESP_LOGI(GATTC_TAG, "Get char b: count = %d", count);
                    for (int i = 0; i < count; ++i)
                    {
                        if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                        {
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[i].char_handle;
                            esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[i].char_handle);
                            ESP_LOGI(GATTC_TAG, "UUID16: %x", char_elem_result[i].uuid.uuid.uuid16);
                        }
                    }

                    status = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                                                          p_data->search_cmpl.conn_id,
                                                                                          gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                                          gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                                          remote_filter_char_c_uuid,
                                                                                          char_elem_result,
                                                                                          &count));
                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }
                    ESP_LOGI(GATTC_TAG, "Get char c: count = %d", count);
                    for (int i = 0; i < count; ++i)
                    {
                        if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                        {
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[i].char_handle;
                            esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[i].char_handle);
                            ESP_LOGI(GATTC_TAG, "UUID16: %x", char_elem_result[i].uuid.uuid.uuid16);
                        }
                    }
                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result'
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }*/
                }
                /* free char_elem_result */
                free(char_elem_result);
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                        &count);
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT, count = %d, start = %d, end = %d, handle = %d", count, gl_profile_tab[PROFILE_A_APP_ID].service_start_handle, gl_profile_tab[PROFILE_A_APP_ID].service_end_handle, gl_profile_tab[PROFILE_A_APP_ID].char_handle);
            if (ret_status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            if (count > 0)
            {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        free(descr_elem_result);
                        descr_elem_result = NULL;
                        break;
                    }
                    // Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result'
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    free(descr_elem_result);
                }
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "decsr not found %d", count);
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
        {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        if (p_data->notify.handle == 42)
        {
            int16_t temp_raw = *(int16_t *)p_data->notify.value;
            float temp = 1.0 * temp_raw / 100;
            ESP_LOGI(GATTC_TAG, "Temperature: %.2f°C", temp);
        }
        if (p_data->notify.handle == 45)
        {
            uint16_t humidity_raw = *(uint16_t *)p_data->notify.value;
            float humidity = 1.0 * humidity_raw / 100;
            ESP_LOGI(GATTC_TAG, "Humidity: %.2f%%", humidity);
        }
        if (p_data->notify.handle == 48)
        {
            int16_t voltage_raw = *(int16_t *)p_data->notify.value;
            float voltage = 1.0 * voltage_raw / 1000;
            ESP_LOGI(GATTC_TAG, "Voltage: %.2fV", voltage);
        }
        ESP_LOGI(GATTC_TAG, "receive data handle = %d", p_data->notify.handle);
        ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success ");
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        ESP_LOG_BUFFER_HEX(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        esp_restart();
        break;
    default:
        break;
    }
}

static uint8_t last_sequence_number = 0;
static uint8_t SERIVCE_UUID[4] = {0x03, 0x03, 0x1A, 0x18};
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        // the unit of the duration is second
        uint32_t duration = 3600;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            if (scan_result->scan_rst.adv_data_len > 0)
            {

                uint8_t *encrypted = &scan_result->scan_rst.ble_adv[0];
                if (memcmp(SERIVCE_UUID, encrypted + 6, 4) != 0)
                {
                    //ESP_LOGI(GATTC_TAG, "Ignore device");
                    break;
                }

                static uint8_t offset_adv_data = 12;
                static uint8_t offset_adv_payload = 19;
                if (scan_result->scan_rst.adv_data_len >= offset_adv_data + 16)
                {
                    uint8_t data[16] = {0};
                    uint8_t offset = offset_adv_payload - offset_adv_data;
                    uint8_t dec_iv[16] = AES_IV;
                    decrypt_bytes(encrypted + 12, data, 16, (uint8_t *)enc_key, (uint8_t *)dec_iv);

                    uint16_t package_count = (data[offset] << 8) | data[offset + 1];
                    if (last_sequence_number == package_count)
                    {
                        //ESP_LOGI(GATTC_TAG, "Duplicate package detected, skipping...");
                        break;
                    }
                    //ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);

                    last_sequence_number = package_count;
                    // Extract temperature (2 bytes)
                    uint16_t temperature = (data[offset + 2] << 8) | data[offset + 3];
                    // Extract humidity (2 bytes)
                    uint16_t humidity = (data[offset + 4] << 8) | data[offset + 5];
                    // Extract voltage (2 bytes)
                    uint16_t voltage = (data[offset + 6] << 8) | data[offset + 7];

                    // Convert the values to the correct units
                    float temperature_celsius = temperature / 100.0;
                    float humidity_percent = humidity / 100.0;

                    ESP_LOGI(GATTC_TAG, "Received Data: %d, Temperature: %.2f°C, Humidity: %.2f%%, Battery Voltage: %.2fV", package_count, temperature_celsius, humidity_percent, voltage / 1000.0);
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    // register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}