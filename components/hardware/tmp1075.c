///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 13.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "tmp1075.h"
#include "esp_log.h"

#define TAG "TMP1075"

float convert_to_celsius(const int16_t raw) {
  return (float)raw / 256;
}

static tmp1075_status_t write_reg_8b(tmp1075_struct_t *tmp1075, const uint8_t reg, uint8_t val) {
    bool ret = true;
    ret = tmp1075->_i2c_write(tmp1075->i2c_address, reg, &val, 1);
    return ret ? TMP1075_OK : TMP1075_WRITE_ERR;
}

static tmp1075_status_t write_reg_16b(tmp1075_struct_t *tmp1075, const uint8_t reg, uint16_t val) {
    bool ret = true;
    uint8_t data[2] = {val >> 8, val & 0xFF};
    ret = tmp1075->_i2c_write(tmp1075->i2c_address, reg, data, 2);
    return ret ? TMP1075_OK : TMP1075_WRITE_ERR;
}

static tmp1075_status_t read_reg_16b(tmp1075_struct_t *tmp1075, const uint8_t reg, uint16_t *val) {
    bool ret = true;
    uint8_t data[2];
    ret = tmp1075->_i2c_read(tmp1075->i2c_address, reg, data, 2);
    *val = (data[0] << 8) | data[1];
    return ret ? TMP1075_OK : TMP1075_READ_ERR;
}

tmp1075_status_t tmp1075_init(tmp1075_struct_t *tmp1075) {
    tmp1075_status_t ret = TMP1075_OK;
    uint16_t reg_val = 0;

    ret = read_reg_16b(tmp1075, TMP1075_REG_CONFIG, &reg_val);
    if (ret != TMP1075_OK) {
        ESP_LOGE(TAG, "TMP1075 initialization failed");
        return ret;
    }
    tmp1075->config_register = (uint8_t)(reg_val >> 8);

    return ret;
}

tmp1075_status_t tmp1075_get_temp_raw(tmp1075_struct_t *tmp1075, int16_t *raw) {
    tmp1075_status_t ret = TMP1075_OK;
    uint16_t raw_temp = 0;

    ret = read_reg_16b(tmp1075, TMP1075_REG_TEMP, &raw_temp);
    if (ret != TMP1075_OK) {
        return ret;
    }
    *raw = (int16_t)raw_temp;
    
    return TMP1075_OK;
}

tmp1075_status_t tmp1075_get_temp_celsius(tmp1075_struct_t *tmp1075, float *celsius) {
    tmp1075_status_t ret = TMP1075_OK;
    int16_t raw_temp = 0;

    ret = tmp1075_get_temp_raw(tmp1075, &raw_temp);
    if (ret != TMP1075_OK) {
        return ret;
    }
    *celsius = convert_to_celsius(raw_temp);

    return TMP1075_OK;
}

tmp1075_status_t tmp1075_start_conv(tmp1075_struct_t *tmp1075) {
    tmp1075_status_t ret = TMP1075_OK;
    uint8_t reg_val = 0;

    reg_val = (uint8_t)(tmp1075->config_register | (1 << TMP1075_OFFSET_OS));
    ret = write_reg_8b(tmp1075, TMP1075_REG_CONFIG, reg_val);
    if (ret != TMP1075_OK) {
        return ret;
    }

    return TMP1075_OK;
}

tmp1075_status_t tmp1075_get_conv_mode(tmp1075_struct_t *tmp1075, bool *is_single_shot) {
    *is_single_shot = (bool)((tmp1075->config_register >> TMP1075_OFFSET_SD) & 0b1);
    return TMP1075_OK;
}

tmp1075_status_t tmp1075_set_conv_mode(tmp1075_struct_t *tmp1075, const bool is_single_shot) {
    tmp1075_status_t ret = TMP1075_OK;
    uint8_t reg_val = 0;

    reg_val = (uint8_t)(tmp1075->config_register & ~(1 << TMP1075_OFFSET_SD)) | ((uint8_t)is_single_shot << TMP1075_OFFSET_SD);
    ret = write_reg_8b(tmp1075, TMP1075_REG_CONFIG, reg_val);
    if (ret != TMP1075_OK) {
        return ret;
    }

    tmp1075->config_register = reg_val;
    return TMP1075_OK;
}

tmp1075_status_t tmp1075_get_conv_time(tmp1075_struct_t *tmp1075, tmp1075_conversion_time_t *conv_time) {
    *conv_time = (tmp1075_conversion_time_t)((tmp1075->config_register >> TMP1075_OFFSET_R) & 0b11);
    return TMP1075_OK;
}

tmp1075_status_t tmp1075_set_conv_time(tmp1075_struct_t *tmp1075, const tmp1075_conversion_time_t conv_time) {
    tmp1075_status_t ret = TMP1075_OK;
    uint8_t reg_val = 0;

    if (((tmp1075_conversion_time_t)(tmp1075->config_register >> TMP1075_OFFSET_R) & 0b11) != conv_time) {
        return TMP1075_OK;
    }

    reg_val = (uint8_t)(tmp1075->config_register & ~(0b11 << TMP1075_OFFSET_R)) | ((uint8_t)conv_time << TMP1075_OFFSET_R);
    ret = write_reg_8b(tmp1075, TMP1075_REG_CONFIG, reg_val);
    if (ret != TMP1075_OK) {
        return ret;
    }

    tmp1075->config_register = reg_val;
    return TMP1075_OK;
}