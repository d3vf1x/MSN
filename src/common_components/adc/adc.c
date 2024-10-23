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

#include "adc.h"

static const char *TAG = "ADC";


static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali;

#define NUM_SAMPLES 10
esp_err_t read_battery_voltage(int *voltage) {
    int raw_sum = 0;
    int raw_reading = 0;
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &raw_reading);
        if (ret != ESP_OK) {
            return ret;
        }
        raw_sum += raw_reading;
    }

    int raw_avg = raw_sum / NUM_SAMPLES;
    esp_err_t ret = adc_cali_raw_to_voltage(adc1_cali, raw_avg, voltage);
    return ret;
}


esp_err_t init_adc()
{
    esp_err_t err;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    err = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if(err != ESP_OK)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,

    };
    err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config);
    if(err != ESP_OK)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    if (!adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTEN_DB_12, &adc1_cali))
    {
        ESP_LOGE(TAG, "ADC Calibration Init Failed");
        return ESP_FAIL;
    }

	return ESP_OK;
}

esp_err_t deinit_adc(){
    esp_err_t err = ESP_OK;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Curve Fitting");
    err = adc_cali_delete_scheme_curve_fitting(adc1_cali);
    if(err != ESP_OK){
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Line Fitting");
    err = adc_cali_delete_scheme_line_fitting(adc1_cali);
    if(err != ESP_OK){
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }
#endif

    err = adc_oneshot_del_unit(adc1_handle);
    if(err != ESP_OK){
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    ESP_LOGD(TAG, "ADC deinitialized");
    return ESP_OK;
}