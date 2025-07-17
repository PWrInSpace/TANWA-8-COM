// Copyright 2022 PWrInSpace, Kuba
#include "timers_config.h"

#include "board_config.h"
#include "board_data.h"
#include "can_commands.h"
#include "can_api.h"

#include "mcu_gpio_config.h"
#include "state_machine_config.h"
#include "settings_mem.h"

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

void on_disconnect_timer(void* args){
    // Handle the disconnect event
    ESP_LOGW(TAG, "DISCONNECT EVENT");
    // Handle the disconnect event
    state_machine_force_change_state(ABORT);
}

void on_after_burnout_timer(void *arg){
    // Handle the after burnout event
    ESP_LOGW(TAG, "AFTER BURNOUT EVENT");
    // Handle the after burnout event
    state_machine_force_change_state(AFTER_BURNOUT);
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

void on_burn_timer(void *arg){
    // Handle the burn event
    ESP_LOGW(TAG, "BURN EVENT");

    sd_timer_change_period(20);

    Settings settings = settings_get_all();

    // valve_move_angle_servo(&(TANWA_utility.servo_valve[1]), settings.oxidizer_valve_initial_angle);

    uint8_t data[8] = {1, (uint8_t)(settings.oxidizer_valve_initial_angle), 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_ANGLE_ID, data, 2);

    if(settings.fuel_open_time_ms == 0){
        // valve_move_angle_servo(&(TANWA_utility.servo_valve[0]), settings.fuel_valve_initial_angle);
        uint8_t data_fuel[8] = {0, (uint8_t)(settings.fuel_valve_initial_angle), 0, 0, 0, 0, 0, 0};
        can_send_message(CAN_SOL_SERVO_ANGLE_ID, data_fuel, 2);

    }
    else{
        if(!sys_timer_start(TIMER_FUEL_INITIAL, settings.fuel_open_time_ms, TIMER_TYPE_ONE_SHOT)){
            ESP_LOGE(TAG, "Error starting timer");
        }
    }
    if(settings.oxidizer_full_open_time_ms + settings.fuel_open_time_ms == 0){
        // valve_open_servo(&(TANWA_utility.servo_valve[1]));

        //ESP_LOGI(TAG, "OXI OPEN FULL");
        uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
        can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 1);
    }
    else{
        if(!sys_timer_start(TIMER_OXIDIZER_FULL, settings.oxidizer_full_open_time_ms + settings.fuel_open_time_ms, TIMER_TYPE_ONE_SHOT)){
            ESP_LOGE(TAG, "Error starting timer");
        }
    }
    if(settings.fuel_full_open_time_ms + settings.oxidizer_full_open_time_ms + settings.fuel_open_time_ms == 0){
        // valve_open_servo(&(TANWA_utility.servo_valve[0]));
        uint8_t data_fuel[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        can_send_message(CAN_SOL_SERVO_OPEN_ID, data_fuel, 1);
    }
    else{
        if(!sys_timer_start(TIMER_FUEL_FULL, settings.fuel_full_open_time_ms + settings.oxidizer_full_open_time_ms + settings.fuel_open_time_ms, TIMER_TYPE_ONE_SHOT)){
            ESP_LOGE(TAG, "Error starting timer");
        }
    }

    // Handle the burn event
    state_machine_force_change_state(FIRE);
}

static void on_fuel_initial(void *arg) {
    ESP_LOGI(TAG, "Fuel initial valve open");

    Settings settings = settings_get_all();

    //valve_move_angle_servo(&(TANWA_utility.servo_valve[0]), settings.fuel_valve_initial_angle);

    uint8_t data[8] = {0, (uint8_t)(settings.fuel_valve_initial_angle), 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_ANGLE_ID, data, 2);
}

static void on_oxidizer_full(void *arg) {
    ESP_LOGI(TAG, "Oxidizer full valve open");

    // valve_open_servo(&(TANWA_utility.servo_valve[1]));
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 1);
}

static void on_fuel_full(void *arg) {
    ESP_LOGI(TAG, "Fuel full valve open");

    //valve_open_servo(&(TANWA_utility.servo_valve[0]));
    uint8_t data[8] = {0}; 
    can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 1);
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
    {.timer_id = TIMER_DISCONNECT, .timer_callback_fnc = on_disconnect_timer, .timer_arg = NULL},
    {.timer_id = TIMER_IGNITION, .timer_callback_fnc = on_ignition_timer, .timer_arg = NULL},
    {.timer_id = TIMER_BURN, .timer_callback_fnc = on_burn_timer, .timer_arg = NULL},
    {.timer_id = TIMER_AFTER_BURNOUT, .timer_callback_fnc = on_after_burnout_timer, .timer_arg = NULL},
    {.timer_id = TIMER_FUEL_INITIAL, .timer_callback_fnc = on_fuel_initial, .timer_arg = NULL},
    {.timer_id = TIMER_OXIDIZER_FULL, .timer_callback_fnc = on_oxidizer_full, .timer_arg = NULL},
    {.timer_id = TIMER_FUEL_FULL, .timer_callback_fnc = on_fuel_full, .timer_arg = NULL},
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