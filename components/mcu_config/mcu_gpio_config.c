///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 17.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
#include "mcu_gpio_config.h"

#include "esp_log.h"

#define TAG "MCU_GPIO"

static mcu_gpio_config_t mcu_gpio_config = {
    .pins = {LED_GPIO, LORA_CS_GPIO, LORA_D0_GPIO, ABORT_GPIO, ARM1_GPIO, ARM2_GPIO, FIRE_GPIO},
    .num_pins = MAX_GPIO_INDEX,
    .configs = {
        {
            .pin_bit_mask = (1ULL << LED_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        },
        {
            .pin_bit_mask = (1ULL << LORA_CS_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        },
        {
            .pin_bit_mask = (1ULL << LORA_D0_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_POSEDGE,
        },
        {
            .pin_bit_mask = (1ULL << ABORT_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        },
        {
            .pin_bit_mask = (1ULL << ARM1_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        },
        {
            .pin_bit_mask = (1ULL << ARM2_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        },
        {
            .pin_bit_mask = (1ULL << FIRE_GPIO),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        }
    },
};

esp_err_t mcu_gpio_init() {
    esp_err_t res = ESP_OK;

    for (uint8_t i = 0; i < mcu_gpio_config.num_pins; i++) {
        res |= gpio_config(&mcu_gpio_config.configs[i]);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "GPIO pin %d configuration failed!", i);
        }
    }

    for (uint8_t i = 0; i < mcu_gpio_config.num_pins; i++) {
        if (mcu_gpio_config.configs[i].mode == GPIO_MODE_INPUT) {
            continue;
        }
        if (!_mcu_gpio_set_level(mcu_gpio_config.pins[i], 0)) {
            ESP_LOGE(TAG, "GPIO pin %d level set failed!", i);
        }
    }

    return res;
}

bool _mcu_gpio_set_level(uint8_t gpio, uint8_t level) {
    // ESP_LOGI(TAG, "Setting GPIO pin %d to %d", mcu_gpio_config.pins[gpio], level);
    return gpio_set_level(mcu_gpio_config.pins[gpio], level) == ESP_OK ? true : false;
}

bool _mcu_gpio_get_level(uint8_t gpio, uint8_t* level) {
    *level = (uint8_t) gpio_get_level(mcu_gpio_config.pins[gpio]);
    return true;
}

bool _abort_gpio_attach_isr(gpio_isr_t interrupt_cb) {
    esp_err_t res = ESP_OK;
    res = gpio_isr_handler_add(ABORT_GPIO, interrupt_cb, NULL);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR handler add failed!");
        return false;
    }
    return true;
}

bool _lora_gpio_set_level(uint8_t gpio, uint8_t level) {
    return gpio_set_level(gpio, level) == ESP_OK ? true : false;
}

bool _lora_gpio_attach_d0_isr(gpio_isr_t interrupt_cb) {
    esp_err_t res = ESP_OK;
    res = gpio_install_isr_service(0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR service installation failed!");
        return false;
    }
    res = gpio_isr_handler_add(LORA_D0_GPIO, interrupt_cb, NULL); 
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR handler add failed!");
        return false;
    }
    return true;
}
