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
 *  https://github.com/espressif/esp-zigbee-sdk
 */
#include "esp_check.h"
#include "esp_log.h"

#include "zigbee.h"
#include "board.h"

static const char *TAG = "ESP_ZB_MULTI_SENSOR";

volatile bool zibee_ready = false;
volatile uint8_t count = 0;

bool is_ready()
{
    return zibee_ready;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

esp_err_t send_update(int16_t *voltage, int16_t *temperature, uint16_t *humidity)
{

    if (!esp_zb_lock_acquire(portMAX_DELAY))
        return ESP_FAIL;
    SHUTDOWN_ON_FAIL(write_attribute(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, temperature),
                     "Failed to write temperature attribute");
    SHUTDOWN_ON_FAIL(report_attribute(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID),
                     "Failed to report temperature attribute");

    SHUTDOWN_ON_FAIL(write_attribute(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, voltage),
                     "Failed to write voltage attribute");
    SHUTDOWN_ON_FAIL(report_attribute(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID),
                     "Failed to report voltage attribute");

    SHUTDOWN_ON_FAIL(write_attribute(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, humidity),
                     "Failed to write humidity attribute");
    SHUTDOWN_ON_FAIL(report_attribute(HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID),
                     "Failed to report humidity attribute");
    esp_zb_lock_release();

    return ESP_OK;
}

esp_zb_cluster_list_t *init_cluster()
{
    // Create Basic Cluster
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0001;
    uint32_t HWVersion = 0x0001;
    uint8_t ManufacturerName[] = {4, 'H', 'T', 'W', 'K'}; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {12, 'M', 'U', 'L', 'T', 'I', '_', 'S', 'E', 'N', 'S', 'O', 'R'};
    uint8_t DateCode[] = {8, '2', '0', '2', '4', '0', '8', '0', '1'};
    uint32_t SWBuildID = 0x01;
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion),
                     "Failed to add ApplicationVersion attribute");
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion),
                     "Failed to add StackVersion attribute");
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion),
                     "Failed to add HWVersion attribute");
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName),
                     "Failed to add ManufacturerName attribute");
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier),
                     "Failed to add ModelIdentifier attribute");
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode),
                     "Failed to add DateCode attribute");
    SHUTDOWN_ON_FAIL(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, &SWBuildID),
                     "Failed to add SWBuildID attribute");

    // Cluster IDENTIFY
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // Cluster Temperature
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -10000, //-10.00 °C
        .max_value = 10000,  // 100.00 °C
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);

    // Cluster Humidity
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = 0,
        .max_value = 10000, // 100.00 %
    };
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg);

    // Electrical (Battery Voltage) Cluster
    esp_zb_electrical_meas_cluster_cfg_t power_config_meas_cfg = {
        .measured_type = ESP_ZB_ZCL_ELECTRICAL_MEASUREMENT_DC_MEASUREMENT,
    };
    int16_t batteryVoltage = 0;
    esp_zb_attribute_list_t *esp_zb_electrical_meas_cluster = esp_zb_electrical_meas_cluster_create(&power_config_meas_cfg);
    SHUTDOWN_ON_FAIL(esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &batteryVoltage),
                     "Failed to add batteryVoltage attribute");

    esp_zb_power_config_cluster_cfg_t power_cfg = {
        .main_alarm_mask = 0, // Initialize with default values
        .main_voltage = 30,
        .main_voltage_max = 33,
        .main_voltage_min = 15,
        .main_freq = 0,

    };
    esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_power_config_cluster_create(&power_cfg);
    SHUTDOWN_ON_FAIL(esp_zb_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &batteryVoltage),
                     "Failed to add batteryVoltage attribute");

    //
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    SHUTDOWN_ON_FAIL(esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE),
                     "Failed to add basic cluster");
    SHUTDOWN_ON_FAIL(esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE),
                     "Failed to add power config cluster");
    SHUTDOWN_ON_FAIL(esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE),
                     "Failed to add identify cluster");
    SHUTDOWN_ON_FAIL(esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE),
                     "Failed to add temperature cluster");
    SHUTDOWN_ON_FAIL(esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE),
                     "Failed to add humidity cluster");
    SHUTDOWN_ON_FAIL(esp_zb_cluster_list_add_electrical_meas_cluster(esp_zb_cluster_list, esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE),
                     "Failed to add electrical cluster");
    struct esp_zb_cluster_list_s *next = esp_zb_cluster_list->next;
    while(next != NULL){
        ESP_LOGI(TAG, "Cluster: 0x%x", next->cluster.cluster_id);
        next = next->next;
    }
    
    
    return esp_zb_cluster_list;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_SENSOR_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        count++;
        if (count > 2)
        {
            shutdown_board();
        }
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_cluster_list_t *cluster_list = init_cluster();
    if (cluster_list == NULL)
    {
        blinking();
        shutdown_board();
        return;
    }

    // Create endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, cluster_list, endpoint_config);

    // Register Device
    SHUTDOWN_ON_FAIL(esp_zb_device_register(esp_zb_ep_list), "Failed to register device");

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_rx_on_when_idle(false);

    SHUTDOWN_ON_FAIL(esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK), "Failed to set primary channel mask");
    SHUTDOWN_ON_FAIL(esp_zb_start(false), "Failed to start Zigbee stack");

    esp_zb_main_loop_iteration();
}

esp_err_t zigbee_factory_reset()
{
    ESP_LOGI(TAG, "Clearing zigbee storage partition...");
    const char *partition_label = "zb_storage";

    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, partition_label);

    if (partition == NULL)
    {
        ESP_LOGE(TAG, "Can't find partition %s", partition_label);
        return ESP_FAIL;
    }

    esp_err_t erase_result = esp_partition_erase_range(partition, 0, partition->size);

    if (erase_result == ESP_OK)
    {
        ESP_LOGI(TAG, "Partition %s erased", partition_label);
    }
    else
    {
        ESP_LOGE(TAG, "Can't erase partition %s, error %d", partition_label, erase_result);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zigbee factory reset done");
    return ESP_OK;
}

esp_err_t write_attribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value)
{
    esp_zb_zcl_status_t status;
    status = esp_zb_zcl_set_attribute_val(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID, value, false);

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Setting attribute %04x:%04x failed(0x%02x)!", clusterID, attributeID, status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t report_attribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID)
{
    esp_zb_zcl_status_t status;
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    status = esp_zb_zcl_report_attr_cmd_req(&cmd);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Updating attribute %04x:%04x failed(0x%02x)!", clusterID, attributeID, status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void zigbee_device_start(esp_err_t err_status)
{
    if (err_status == ESP_OK)
    {
        ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new())
        {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            ESP_LOGI(TAG, "Device rebooted");
            zibee_ready = true;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        ESP_LOGI(TAG, "Device started up for the first time");
        zigbee_device_start(err_status);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        zigbee_device_start(err_status);
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zibee_ready = true;
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

esp_err_t init_zigbee()
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    esp_err_t err;
    err = nvs_flash_init();
    if (err != ESP_OK)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = esp_zb_platform_config(&config);
    if (err != ESP_OK)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
    return ESP_OK;
}