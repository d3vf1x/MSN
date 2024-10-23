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


#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Attributes State Machine
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,
    IDX_CHAR_CFG_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,
    IDX_CHAR_CFG_C,

    HRS_IDX_NB,
};

#define TAG "BLE_SENSOR"

// UUIDs for the service and characteristics (example UUIDs)
#define ENVIRONMENTAL_SERVICE_UUID 0x181A
#define TEMP_CHAR_UUID 0x2A1F
#define HUMIDITY_CHAR_UUID 0x2A1E
#define BATTERY_VOLTAGE_CHAR_UUID 0x2A19

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "BLE_SENSOR"
#define SVC_INST_ID 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

//AES Encryption Change the values!
uint8_t enc_key[32] = {0xd4, 0x9c, 0xf8, 0x57, 0xce, 0xed, 0xd9, 0xda, 0x9e, 0x23, 0x1f, 0xa3, 0xb4, 0x11, 0x26, 0x05, 0x10, 0xe9, 0xb1, 0x36, 0x61, 0x9e, 0xe2, 0x18, 0x10, 0x8d, 0x9b, 0xa1, 0x63, 0x76, 0x4a, 0x5e};
uint8_t enc_iv[16] = {0xb6, 0x35, 0x11, 0xcd, 0x6e, 0xca, 0x97, 0x49, 0x98, 0x3c, 0x3c, 0x9c, 0xbf, 0x95, 0x80, 0xc1};


static uint8_t offset_adv_data = 12;
static uint8_t offset_adv_payload = 19;
static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0x1A, 0x18,
    /* device name */
    0x07, ESP_BLE_AD_TYPE_NAME_CMPL,
    'S',   'E',  'N',  'S',  'O', 'R',
    0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //2 bytes package counter, 6 bytes for Data
    0x00//Padding for 16 byte 
};




static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x0020, // 20ms
    .adv_int_max = 0x0020, // 20ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst gatt_profile[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static const uint16_t GATTS_SERVICE_UUID_TEST = ENVIRONMENTAL_SERVICE_UUID;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;

static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
    {
        // Service Declaration
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},
};