// Copyright 2022 PWrInSpace, Kuba
#include "timers_config.h"

#include "board_config.h"
#include "board_data.h"

#include "mcu_gpio_config.h"
#include "state_machine_config.h"

#include "sd_task.h"

#include "esp_log.h"

#define TAG "TIMERS"

void on_sd_timer(void *arg){
    tanwa_data_t tanwa_data = tanwa_data_read();
    if (SDT_send_data(&tanwa_data, sizeof(tanwa_data)) == false) {
        ESP_LOGE(TAG, "Error while sending data to sd card");
    }
}

void on_abort_button_timer(void *arg){
    // Check the state of the button pin
    uint8_t level;
    _mcu_gpio_get_level(ABORT_GPIO_INDEX, &level);
    if (level == 0) {
        ESP_LOGW(TAG, "ABORT BUTTON PRESSED!");
        // Handle the button press event (e.g., send a message to a task)
        state_machine_force_change_state(ABORT);
    }
    // Re-enable the interrupt
    gpio_intr_enable(ABORT_GPIO);
}

void on_ignition_timer(void *arg){
    // Handle the ignition event
    ESP_LOGW(TAG, "IGNITION EVENT");

    if(tanwa_hardware.igniter[0].state != IGNITER_STATE_ARMED || tanwa_hardware.igniter[1].state != IGNITER_STATE_ARMED){
        ESP_LOGE(TAG, "Igniters not armed");
        return;
    }

    if(igniter_fire(&tanwa_hardware.igniter[0]) == IGNITER_OK){
        ESP_LOGI(TAG, "Igniter 1 fired");
    } else {
        ESP_LOGE(TAG, "Failed to fire igniter 1");
    }

    if(igniter_fire(&tanwa_hardware.igniter[1]) == IGNITER_OK){
        ESP_LOGI(TAG, "Igniter 2 fired");
    } else {
        ESP_LOGE(TAG, "Failed to fire igniter 2");
    }

    sys_timer_start(TIMER_IGNITION_OFF, IGNITION_OFF_TIMER, TIMER_TYPE_ONE_SHOT);

    return;

}

static void on_ignition_off(void *arg){
    if(igniter_reset(&tanwa_hardware.igniter[0]) == IGNITER_OK){
        ESP_LOGI(TAG, "Igniter 1 reset");
    } else {
        ESP_LOGE(TAG, "Failed to fire igniter 1");
    }

    if(igniter_reset(&tanwa_hardware.igniter[1]) == IGNITER_OK){
        ESP_LOGI(TAG, "Igniter 2 reset");
    } else {
        ESP_LOGE(TAG, "Failed to fire igniter 2");
    }
}

bool initialize_timers(void) {
    sys_timer_t timers[] = {
    {.timer_id = TIMER_SD_DATA, .timer_callback_fnc = on_sd_timer, .timer_arg = NULL},
    //{.timer_id = TIMER_BUZZER, .timer_callback_fnc = on_buzzer_timer, .timer_arg = NULL},
    {.timer_id = TIMER_ABORT_BUTTON, .timer_callback_fnc = on_abort_button_timer, .timer_arg = NULL},
    //{.timer_id = TIMER_DISCONNECT, .timer_callback_fnc = on_disconnect_timer, .timer_arg = NULL},
    //{.timer_id = TIMER_IGNITION, .timer_callback_fnc = on_ignition_timer, .timer_arg = NULL},
    //{.timer_id = TIMER_BURN, .timer_callback_fnc = on_burn_timer, .timer_arg = NULL},
    //{.timer_id = TIMER_AFTER_BURNOUT, .timer_callback_fnc = on_after_burnout_timer, .timer_arg = NULL},
    //{.timer_id = TIMER_FUEL_INITIAL, .timer_callback_fnc = on_fuel_initial, .timer_arg = NULL},
    //{.timer_id = TIMER_OXIDIZER_FULL, .timer_callback_fnc = on_oxidizer_full, .timer_arg = NULL},
    //{.timer_id = TIMER_FUEL_FULL, .timer_callback_fnc = on_fuel_full, .timer_arg = NULL},
    {.timer_id = TIMER_IGNITION_OFF, .timer_callback_fnc = on_ignition_off, .timer_arg = NULL}
    };
    return sys_timer_init(timers, sizeof(timers) / sizeof(timers[0]));
}

bool buzzer_timer_start(uint32_t period_ms) {
    return sys_timer_start(TIMER_BUZZER, period_ms, TIMER_TYPE_PERIODIC);
}

bool buzzer_timer_change_period(uint32_t period_ms) {
    if (!sys_timer_stop(TIMER_BUZZER)) {
        ESP_LOGE(TAG, "Failed to stop buzzer timer");
        return false;
    }
    return sys_timer_start(TIMER_BUZZER, period_ms, TIMER_TYPE_PERIODIC);
}

bool abort_button_timer_start_once(uint32_t period_ms) {
    return sys_timer_start(TIMER_ABORT_BUTTON, period_ms, TIMER_TYPE_ONE_SHOT);
}

bool sd_timer_change_period(uint32_t period_ms) {
    if (!sys_timer_stop(TIMER_SD_DATA)) {
        ESP_LOGE(TAG, "Failed to stop SD timer");
        return false;
    }
    return sys_timer_start(TIMER_SD_DATA, period_ms, TIMER_TYPE_PERIODIC);
}