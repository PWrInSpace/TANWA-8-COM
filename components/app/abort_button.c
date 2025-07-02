///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 19.03.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "abort_button.h"

#include "mcu_gpio_config.h"
#include "mcu_misc_config.h"

#include "timers_config.h"

#include "esp_log.h"

#define TAG "ABORT_BUTTON"

static abort_button_struct_t abort_button = {
    ._gpio_get_level = _mcu_gpio_get_level,
    ._delay = _abort_button_delay_ms,
    .gpio_idx = ABORT_GPIO_INDEX,
    .polarity = ABORT_BUTTTON_POLARITY_NEGATIVE,
};

abort_button_status_t abort_button_init() {
    // Attach interrupt to the GPIO pin
    if (!_abort_gpio_attach_isr(abort_button_callback)) {
        ESP_LOGE(TAG, "Failed to attach interrupt to the abort button");
        return ABORT_BUTTON_ERROR;
    }
    return ABORT_BUTTON_OK;
}

abort_button_status_t abort_button_get_level(uint8_t *level) {
    if (level == NULL) {
        return ABORT_BUTTON_ERROR;
    }
    if(!abort_button._gpio_get_level(abort_button.gpio_idx, level)) {
        ESP_LOGE(TAG, "Failed to read the level of the abort button");
        return ABORT_BUTTON_READ_ERROR;
    }
    return ABORT_BUTTON_OK;
}

void abort_button_callback(void* arg) {
    // Disable the interrupt to prevent re-triggering during debounce period
    gpio_intr_disable(ABORT_GPIO);
    // Start the debounce timer with a delay of 50 ms
    abort_button_timer_start_once(500);
}