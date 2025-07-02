///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 29.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "mcu_misc_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"

#include "esp_log.h"

#define TAG "MCU_MISC"

void _led_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void _igniter_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void _lora_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void _buzzer_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void _abort_button_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t _get_uptime_ms(void) {
    return esp_timer_get_time() / 1000.0;
}

void _lora_log(const char* info) { 
    ESP_LOGD(TAG, "%s", info); 
}