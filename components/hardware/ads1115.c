///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 14.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "ads1115.h"
#include "esp_log.h"

#define TAG "ADS1115"

#define REG_CONVERSION 0
#define REG_CONFIG     1
#define REG_THRESH_L   2
#define REG_THRESH_H   3

#define COMP_QUE_OFFSET  1
#define COMP_QUE_MASK    0x03
#define COMP_LAT_OFFSET  2
#define COMP_LAT_MASK    0x01
#define COMP_POL_OFFSET  3
#define COMP_POL_MASK    0x01
#define COMP_MODE_OFFSET 4
#define COMP_MODE_MASK   0x01
#define DR_OFFSET        5
#define DR_MASK          0x07
#define MODE_OFFSET      8
#define MODE_MASK        0x01
#define PGA_OFFSET       9
#define PGA_MASK         0x07
#define MUX_OFFSET       12
#define MUX_MASK         0x07
#define OS_OFFSET        15
#define OS_MASK          0x01

///===-----------------------------------------------------------------------------------------===//

const float ads1115_gain_values[] = {
    [ADS1115_GAIN_6V144]   = 6.144,
    [ADS1115_GAIN_4V096]   = 4.096,
    [ADS1115_GAIN_2V048]   = 2.048,
    [ADS1115_GAIN_1V024]   = 1.024,
    [ADS1115_GAIN_0V512]   = 0.512,
    [ADS1115_GAIN_0V256]   = 0.256,
    [ADS1115_GAIN_0V256_2] = 0.256,
    [ADS1115_GAIN_0V256_3] = 0.256
};

///===-----------------------------------------------------------------------------------------===//

static ads1115_status_t read_reg_2b(ads1115_struct_t *ads1115, uint8_t reg, uint16_t *val) {
    bool ret = true;
    uint8_t buf[2];
    ret = ads1115->_i2c_read(ads1115->i2c_address, reg, buf, 2);
    if (ret != true)
    {
        ESP_LOGE(TAG, "Could not read from register 0x%02x", reg);
        return ADS1115_READ_ERR;
    }
    *val = (buf[0] << 8) | buf[1];
    return ADS1115_OK;
}

static ads1115_status_t write_reg_2b(ads1115_struct_t *ads1115, uint8_t reg, uint16_t val) {
    bool ret = true;
    uint8_t buf[2] = { val >> 8, val };
    ret = ads1115->_i2c_write(ads1115->i2c_address, reg, buf, 2);
    if (ret != true)
    {
        ESP_LOGE(TAG, "Could not write 0x%04x to register 0x%02x", val, reg);
        return ADS1115_WRITE_ERR;
    }
    return ADS1115_OK;
}

static ads1115_status_t read_conf_bits(ads1115_struct_t *ads1115, uint8_t offs, uint16_t mask,
        uint16_t *bits) {
    ads1115_status_t ret = ADS1115_OK;
    uint16_t val;

    ret = read_reg_2b(ads1115, REG_CONFIG, &val);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Could not read config register");
        return ret;
    }

    ESP_LOGD(TAG, "Got config value: 0x%04x", val);

    *bits = (val >> offs) & mask;

    return ADS1115_OK;
}

static ads1115_status_t write_conf_bits(ads1115_struct_t *ads1115, uint16_t val, uint8_t offs,
        uint16_t mask) {
    ads1115_status_t ret = ADS1115_OK;
    uint16_t old;

    ret = read_reg_2b(ads1115, REG_CONFIG, &old);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Could not read config register");
        return ret;
    }

    ret = write_reg_2b(ads1115, REG_CONFIG, (old & ~(mask << offs)) | (val << offs));
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Could not write config register");
        return ret;
    }

    return ADS1115_OK;
}

///===-----------------------------------------------------------------------------------------===//

ads1115_status_t ads1115_is_busy(ads1115_struct_t *ads1115, bool *busy)
{
    if (ads1115 == NULL || busy == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t r;
    
    ret = read_conf_bits(ads1115, OS_OFFSET, OS_MASK, &r);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read busy bit failed");
        return ret;
    }

    *busy = !r;
    return ADS1115_OK;
}

ads1115_status_t ads1115_start_conversion(ads1115_struct_t *ads1115) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, 1, OS_OFFSET, OS_MASK);
}

ads1115_status_t ads1115_get_value(ads1115_struct_t *ads1115, int16_t *value) {
    if (ads1115 == NULL || value == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return read_reg_2b(ads1115, REG_CONVERSION, (uint16_t *)value);
}

ads1115_status_t ads1115_get_gain(ads1115_struct_t *ads1115, ads1115_gain_t *gain) {
    if (ads1115 == NULL || gain == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;                        

    ret = read_conf_bits(ads1115, PGA_OFFSET, PGA_MASK, &bits);  
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read gain failed");
        return ret;
    }

    *gain = bits;                    
    return ADS1115_OK;                  
}

ads1115_status_t ads1115_set_gain(ads1115_struct_t *ads1115, ads1115_gain_t gain) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, gain, PGA_OFFSET, PGA_MASK);
}

ads1115_status_t ads1115_get_input_mux(ads1115_struct_t *ads1115, ads1115_mux_t *mux) {
    if (ads1115 == NULL || mux == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, MUX_OFFSET, MUX_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read input mux failed");
        return ret;
    }

    *mux = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_input_mux(ads1115_struct_t *ads1115, ads1115_mux_t mux) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, mux, MUX_OFFSET, MUX_MASK);
}

ads1115_status_t ads1115_get_mode(ads1115_struct_t *ads1115, ads1115_mode_t *mode) {
    if (ads1115 == NULL || mode == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, MODE_OFFSET, MODE_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read mode failed");
        return ret;
    }

    *mode = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_mode(ads1115_struct_t *ads1115, ads1115_mode_t mode) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, mode, MODE_OFFSET, MODE_MASK);
}

ads1115_status_t ads1115_get_data_rate(ads1115_struct_t *ads1115, ads1115_data_rate_t *rate) {
    if (ads1115 == NULL || rate == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, DR_OFFSET, DR_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read data rate failed");
        return ret;
    }

    *rate = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_data_rate(ads1115_struct_t *ads1115, ads1115_data_rate_t rate) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, rate, DR_OFFSET, DR_MASK);
}

ads1115_status_t ads1115_get_comp_mode(ads1115_struct_t *ads1115, ads1115_comp_mode_t *mode) {   
    if (ads1115 == NULL || mode == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, COMP_MODE_OFFSET, COMP_MODE_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read comparator mode failed");
        return ret;
    }

    *mode = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_comp_mode(ads1115_struct_t *ads1115, ads1115_comp_mode_t mode) {   
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, mode, COMP_MODE_OFFSET, COMP_MODE_MASK);
}

ads1115_status_t ads1115_get_comp_polarity(ads1115_struct_t *ads1115, ads1115_comp_polarity_t *polarity) {
    if (ads1115 == NULL || polarity == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, COMP_POL_OFFSET, COMP_POL_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read comparator polarity failed");
        return ret;
    }

    *polarity = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_comp_polarity(ads1115_struct_t *ads1115, ads1115_comp_polarity_t polarity) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, polarity, COMP_POL_OFFSET, COMP_POL_MASK);
}

ads1115_status_t ads1115_get_comp_latch(ads1115_struct_t *ads1115, ads1115_comp_latch_t *latch) {
    if (ads1115 == NULL || latch == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, COMP_LAT_OFFSET, COMP_LAT_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read comparator latch failed");
        return ret;
    }

    *latch = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_comp_latch(ads1115_struct_t *ads1115, ads1115_comp_latch_t latch) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, latch, COMP_LAT_OFFSET, COMP_LAT_MASK);
}

ads1115_status_t ads1115_get_comp_queue(ads1115_struct_t *ads1115, ads1115_comp_queue_t *queue) {
    if (ads1115 == NULL || queue == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t bits;

    ret = read_conf_bits(ads1115, COMP_QUE_OFFSET, COMP_QUE_MASK, &bits);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read comparator queue failed");
        return ret;
    }

    *queue = bits;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_comp_queue(ads1115_struct_t *ads1115, ads1115_comp_queue_t queue) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_conf_bits(ads1115, queue, COMP_QUE_OFFSET, COMP_QUE_MASK);
}

ads1115_status_t ads1115_get_comp_low_thresh(ads1115_struct_t *ads1115, int16_t *th) {
    if (ads1115 == NULL || th == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t val;

    ret = read_reg_2b(ads1115, REG_THRESH_L, &val);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read comparator low threshold failed");
        return ret;
    }

    *th = val;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_comp_low_thresh(ads1115_struct_t *ads1115, int16_t th) {
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_reg_2b(ads1115, REG_THRESH_L, th);
}

ads1115_status_t ads1115_get_comp_high_thresh(ads1115_struct_t *ads1115, int16_t *th) {
    if (ads1115 == NULL || th == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }

    ads1115_status_t ret = ADS1115_OK;
    uint16_t val;

    ret = read_reg_2b(ads1115, REG_THRESH_H, &val);
    if (ret != ADS1115_OK) {
        ESP_LOGE(TAG, "Read comparator high threshold failed");
        return ret;
    }

    *th = val;
    return ADS1115_OK;
}

ads1115_status_t ads1115_set_comp_high_thresh(ads1115_struct_t *ads1115, int16_t th)
{
    if (ads1115 == NULL)
    {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return ADS1115_NULL_ARG;
    }
    return write_reg_2b(ads1115, REG_THRESH_H, th);
}