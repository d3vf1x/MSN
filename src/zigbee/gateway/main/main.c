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

#include "zigbee.h"

#include "string.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_power_config.h"



static const char *TAG = "ESP_ZB_THERMOSTAT";

static node_device_params_t node_device;

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        if (bind_req->req_dst_addr == esp_zb_get_short_address()) {
            ESP_LOGI(TAG, "Successfully bind the sensor from address(0x%x) on endpoint(%d)",
                     node_device.short_addr, node_device.endpoint);
        }
    }
}
static void bind_cb_final(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        if (bind_req->req_dst_addr == esp_zb_get_short_address()) {
            ESP_LOGI(TAG, "Successfully bind the sensor from address(0x%x) on endpoint(%d)",
                     node_device.short_addr, node_device.endpoint);

            // Read peer Manufacture Name & Model Identifier 
            esp_zb_zcl_read_attr_cmd_t read_req;
            read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            read_req.zcl_basic_cmd.src_endpoint = HA_THERMOSTAT_ENDPOINT;
            read_req.zcl_basic_cmd.dst_endpoint = node_device.endpoint;
            read_req.zcl_basic_cmd.dst_addr_u.addr_short = node_device.short_addr;
            read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

            uint16_t attributes[] = {
                ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
            };
            read_req.attr_number = ARRAY_LENTH(attributes);
            read_req.attr_field = attributes;

            esp_zb_zcl_read_attr_cmd_req(&read_req);
        }
        if (bind_req->req_dst_addr == node_device.short_addr) {
            ESP_LOGI(TAG, "The temperature sensor from address(0x%x) on endpoint(%d) successfully binds us",
                     node_device.short_addr, node_device.endpoint);
        }
        free(bind_req);
    }
    else {
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t peer_addr, uint8_t peer_endpoint, void *user_ctx)
{

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        
        ESP_LOGI(TAG, "Found temperature sensor");
        // Store the information of the remote device 
        node_device_params_t *sensor = (node_device_params_t *)user_ctx;
        sensor->endpoint = peer_endpoint;
        sensor->short_addr = peer_addr;
        esp_zb_ieee_address_by_short(sensor->short_addr, sensor->ieee_addr);

        //1. Send binding request to the sensor
        esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
        bind_req->req_dst_addr = peer_addr;

        // populate the src information of the binding
        memcpy(bind_req->src_address, sensor->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req->src_endp = peer_endpoint;
       
        //populate the dst information of the binding
        bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req->dst_address_u.addr_long);
        bind_req->dst_endp = HA_THERMOSTAT_ENDPOINT;


        ESP_LOGI(TAG, "Request temperature sensor to bind us");
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);

        ESP_LOGI(TAG, "Request humidity sensor to bind us");
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);

        ESP_LOGI(TAG, "Request bat voltage to bind us");
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT;
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);


        // 2. Send binding request to self 
        bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
        bind_req->req_dst_addr = esp_zb_get_short_address();

        // populate the src information of the binding 
        esp_zb_get_long_address(bind_req->src_address);
        bind_req->src_endp = HA_THERMOSTAT_ENDPOINT;
      
        // populate the dst information of the binding 
        bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req->dst_address_u.addr_long, sensor->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req->dst_endp = peer_endpoint;

       
        ESP_LOGI(TAG, "Bind bat voltage sensor");
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT;
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);

        ESP_LOGI(TAG, "Bind temperature sensor");
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);

        ESP_LOGI(TAG, "Bind humidity sensor");
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        esp_zb_zdo_device_bind_req(bind_req, bind_cb_final, bind_req);
    }
}
static void find_temperature_sensor(esp_zb_zdo_match_desc_req_param_t *param, esp_zb_zdo_match_desc_callback_t user_cb, void *user_ctx)
{
    uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT};
    param->profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    param->num_in_clusters = 3;
    param->num_out_clusters = 0;
    param->cluster_list = cluster_list;
    esp_zb_zdo_match_cluster(param, user_cb, (void *)&node_device);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                esp_zb_bdb_open_network(200);
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
       
        esp_zb_zdo_match_desc_req_param_t  cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        find_temperature_sensor(&cmd_req, user_find_cb, NULL);
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t* attribute)
{
    
      ESP_LOGI(TAG, "Cluster ID: 0x%x, Attribute ID: 0x%x, Type: 0x%x", cluster_id, attribute->id, attribute->data.type);

    /* Basic cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char*)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Manufacturer is \"%s\"", string);
            free(string);
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char*)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Model is \"%s\"", string);
            free(string);
        }
    }
    // Temperature Measurement cluster attributes
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
            uint16_t value = attribute->data.value ? *(uint16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Received value is %f%%", 1.0 * value / 100);
        }
    }

    // Electrical Measurement (Battery Voltage) cluster attributes
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID) {
            uint16_t value = attribute->data.value ? *(uint16_t *)attribute->data.value / 1000 : 0;
            ESP_LOGI(TAG, "Received value is %dV and type: %d", value, attribute->data.type);
        }
    }

    // Temperature Measurement cluster attributes
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Received value is %.2f°C", 1.0 * value / 100);
        }
    }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    esp_app_zb_attribute_handler(message->cluster, &message->attribute);
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                    variable->status, message->info.cluster,
                    variable->attribute.id, variable->attribute.data.type,
                    variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }

        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
                 variable->status, message->info.cluster, variable->direction, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static esp_zb_cluster_list_t *custom_thermostat_clusters_create(esp_zb_thermostat_cfg_t *thermostat)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(thermostat->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(thermostat->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_thermostat_cluster(cluster_list, esp_zb_thermostat_cluster_create(&(thermostat->thermostat_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_electrical_meas_cluster(cluster_list, esp_zb_electrical_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    
    return cluster_list;
}

static esp_zb_ep_list_t *custom_thermostat_ep_create(uint8_t endpoint_id, esp_zb_thermostat_cfg_t *thermostat)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_thermostat_clusters_create(thermostat), endpoint_config);
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Create customized thermostat endpoint 
    esp_zb_thermostat_cfg_t thermostat_cfg = ESP_ZB_DEFAULT_THERMOSTAT_CONFIG();
    esp_zb_ep_list_t *esp_zb_thermostat_ep = custom_thermostat_ep_create(HA_THERMOSTAT_ENDPOINT, &thermostat_cfg);

    // Register the device 
    esp_zb_device_register(esp_zb_thermostat_ep);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
