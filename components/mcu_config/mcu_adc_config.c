///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 28.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
#include "mcu_adc_config.h"
#include "esp_log.h"

#define TAG "MCU_ADC"

mcu_adc_config_t mcu_adc_config = {                                                        
  .adc_cal = {1.0f, 5.742f, 5.180f},                     
  .adc_chan = {ISENSE_CHANNEL, IGNITER_1_CHANNEL, IGNITER_2_CHANNEL}, 
  .adc_chan_num = MAX_CHANNEL_INDEX,
  .oneshot_unit_init_cfg = {                                  
    .unit_id = ADC_UNIT_1,                               
  },        
  .oneshot_chan_cfg = {                                                  
    .bitwidth = ADC_BITWIDTH_12,                   
    .atten = ADC_ATTEN_DB_12,                      
  },         
  .oneshot_unit_handle = NULL,                                     
};

esp_err_t mcu_adc_init() {
  if (mcu_adc_config.adc_chan_num > MAX_ADC_CHANNELS) {
    ESP_LOGE(TAG, "Too many ADC channels to configure!");
    return ESP_FAIL;
  }
  esp_err_t res = ESP_OK;

  res |= adc_oneshot_new_unit(&(mcu_adc_config.oneshot_unit_init_cfg),
                              &(mcu_adc_config.oneshot_unit_handle));

  for (uint8_t i = 0; i < mcu_adc_config.adc_chan_num; i++) {
    res |= adc_oneshot_config_channel(mcu_adc_config.oneshot_unit_handle,
                                      mcu_adc_config.adc_chan[i],
                                      &mcu_adc_config.oneshot_chan_cfg);
    if (res != ESP_OK) {
      ESP_LOGE(TAG, "ADC channel %d configuration failed!", i);
    }
  }

  return res;
}

bool _mcu_adc_read_raw(uint8_t channel, uint16_t* adc_raw) {
  int vRaw;
  if (adc_oneshot_read(mcu_adc_config.oneshot_unit_handle, mcu_adc_config.adc_chan[channel], &vRaw) != ESP_OK) {
    return false;
  }
  *adc_raw = (uint16_t)vRaw;
  return true;
}

bool _mcu_adc_read_voltage(uint8_t channel, float* adc_voltage) {
  uint16_t vRaw;
  if (!_mcu_adc_read_raw(channel, &vRaw)) {
    return false;
  }
  *adc_voltage = mcu_adc_config.adc_cal[channel] * (float)vRaw / 4095.0f * 1.0f;
  return true;
}
