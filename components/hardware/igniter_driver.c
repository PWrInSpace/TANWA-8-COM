///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 28.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
#include "igniter_driver.h"
#include "esp_log.h"

#define TAG "IGNITER"

igniter_status_t igniter_check_continuity(igniter_struct_t* igniter, igniter_continuity_t* continuity) {
    if (igniter == NULL || continuity == NULL) {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return IGNITER_NULL_ARG;
    }
    uint16_t value;
    if (!igniter->_adc_analog_read_raw(igniter->adc_channel_continuity, &value)) {
        ESP_LOGE(TAG, "Failed to read analog value from ADC for continuity check");
        return IGNITER_ADC_ERR;
    }
    ESP_LOGD(TAG, "Continuity check value: %d", value);
    if (value > IGNITER_CONTINUITY_THRESHOLD) {
        *continuity = IGNITER_CONTINUITY_OK;
    } else {
        *continuity = IGNITER_CONTINUITY_FAIL;
    }
    return IGNITER_OK;
}

igniter_status_t igniter_arm(igniter_struct_t* igniter) {
    if (igniter == NULL) {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return IGNITER_NULL_ARG;
    }
    if (igniter->state == IGNITER_STATE_ARMED) {
        ESP_LOGD(TAG, "Already armed");
        return IGNITER_OK;
    }
    if (igniter->state == IGNITER_STATE_FIRED) {
        ESP_LOGE(TAG, "Already fired");
        return IGNITER_OK;
    }
    if(!igniter->_gpio_set_level(igniter->gpio_num_arm, IGNITER_PIN_STATE_HIGH)) {
        ESP_LOGE(TAG, "Failed to set GPIO level for arming");
        return IGNITER_GPIO_ERR;
    }
    igniter->state = IGNITER_STATE_ARMED;
    return IGNITER_OK;
}

igniter_status_t igniter_disarm(igniter_struct_t* igniter) {
    if (igniter == NULL) {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return IGNITER_NULL_ARG;
    }
    if (igniter->state == IGNITER_STATE_WAITING) {
        ESP_LOGD(TAG, "Already disarmed");
        return IGNITER_OK;
    }
    if(!igniter->_gpio_set_level(igniter->gpio_num_arm, IGNITER_PIN_STATE_LOW)) {
        ESP_LOGE(TAG, "Failed to set GPIO level for disarming");
        return IGNITER_GPIO_ERR;
    }
    igniter->state = IGNITER_STATE_WAITING;
    return IGNITER_OK;
}

igniter_status_t igniter_fire(igniter_struct_t* igniter) {
    if (igniter == NULL) {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return IGNITER_NULL_ARG;
    }
    if (igniter->state == IGNITER_STATE_WAITING) {
        ESP_LOGE(TAG, "Not armed");
        return IGNITER_FAIL;
    }
    if (igniter->state == IGNITER_STATE_FIRED) {
        ESP_LOGE(TAG, "Already fired");
        return IGNITER_OK;
    }
    if(!igniter->_gpio_set_level(igniter->gpio_num_fire, IGNITER_PIN_STATE_HIGH)) {
        ESP_LOGE(TAG, "Failed to set GPIO level for firing");
        return IGNITER_GPIO_ERR;
    }
    igniter->state = IGNITER_STATE_FIRED;
    return IGNITER_OK;
}

igniter_status_t igniter_reset(igniter_struct_t* igniter) {
    if (igniter == NULL) {
        ESP_LOGE(TAG, "Invalid argument - NULL check failed");
        return IGNITER_NULL_ARG;
    }
    if (igniter->state == IGNITER_STATE_WAITING) {
        ESP_LOGD(TAG, "Already reset");
        return IGNITER_OK;
    }
    if(!igniter->_gpio_set_level(igniter->gpio_num_fire, IGNITER_PIN_STATE_LOW)) {
        ESP_LOGE(TAG, "Failed to set GPIO level for reset");
        return IGNITER_GPIO_ERR;
    }
    igniter->state = IGNITER_STATE_WAITING;
    return IGNITER_OK;
}