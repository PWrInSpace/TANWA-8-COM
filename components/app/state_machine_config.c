///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 28.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "state_machine_config.h"

#include <stddef.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include "board_config.h"

#include "timers_config.h"
#include "can_commands.h"
#include "mission_timer_config.h"
#include "settings_mem.h"

#include "board_data.h"

#include "can_api.h"

#define TAG "SMC"

extern led_state_display_struct_t led_state_display;

static void on_init(void *arg) {
    ESP_LOGI(TAG, "ON INIT");
}

static void on_idle(void *arg) {

    // valve_close_servo(&TANWA_utility.servo_valve[0]);
    // valve_close_servo(&TANWA_utility.servo_valve[1]);

    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_CLOSE_ID, data, 1);
    uint8_t data_fuel[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_CLOSE_ID, data_fuel, 1);
    can_send_message(CAN_UTIL_SET_BUZZER_ID, data, 1);

    igniter_disarm(&tanwa_hardware.igniter[0]);
    igniter_disarm(&tanwa_hardware.igniter[1]);

    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_IDLE);
    ESP_LOGI(TAG, "ON IDLE");
}

static void on_recovery_arm(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_ARMED);
    ESP_LOGI(TAG, "ON ARM");
}

static void on_fueling(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_FUELING);
    buzzer_timer_start(5000);
    ESP_LOGI(TAG, "ON FUELING");
}

static void on_armed_to_launch(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_ARMED_TO_LAUNCH);
    buzzer_timer_start(3000);
    //buzzer_timer_change_period(3000);
    ESP_LOGI(TAG, "ON ARMED TO LAUNCH");
}

static void on_ready_to_lauch(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_RDY_TO_LAUNCH);
    buzzer_timer_change_period(2000);
    ESP_LOGI(TAG, "ON READY_TO_LAUNCH");
}

static void on_countdown(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_COUTDOWN);
    buzzer_timer_change_period(500);
    if (sys_timer_stop(TIMER_DISCONNECT) == false) {
        ESP_LOGE(TAG, "Unable to stop disconnect timer");
        goto abort_countdown;
    }
    ESP_LOGI(TAG, "ON COUNTDOWN");

    Settings settings = settings_get_all();
    if (liquid_ignition_test_timer_start(settings.countdownTime, settings.ignitTime) == false) {
        ESP_LOGE(TAG, "Mission timer error");
        goto abort_countdown;
    }

    return;

abort_countdown:
    state_machine_change_to_previous_state(true);
    sys_timer_start(TIMER_DISCONNECT, TIMER_DISCONNECT_PERIOD_MS, TIMER_TYPE_ONE_SHOT);
}

static void on_fire(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_FLIGHT);
    sys_timer_stop(TIMER_BUZZER);
    ESP_LOGI(TAG, "----> ON FLIGHT <----");

    if(sys_timer_start(TIMER_AFTER_BURNOUT, ENGINE_BURN_TIME_MS, TIMER_TYPE_ONE_SHOT) == false){
        ESP_LOGE(TAG, "Failed to start after burnout timer");
    }
}

static void on_first_stage_recovery(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_FIRST_STAGE);
    ESP_LOGI(TAG, "ON FIRST STAGE RECOVERY");
}

static void on_second_stage_recovery(void *arg) {
    led_state_display_state_update(&led_state_display, LED_STATE_DISPLAY_STATE_SECOND_STAGE);
    ESP_LOGI(TAG, "ON FIRST STAGE RECOVERY");
}

static void on_ground(void *arg) {
    led_state_display_state_update(&led_state_display, LED_STATE_DISPLAY_STATE_ON_GROUND);
    ESP_LOGI(TAG, "ON GROUND");
}

static void on_after_burnout(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_AFTER_BURNOUT);
    buzzer_timer_change_period(1000);
    ESP_LOGI(TAG, "ON AFTER BURNOUT");
    igniter_disarm(&tanwa_hardware.igniter[0]);
    igniter_disarm(&tanwa_hardware.igniter[1]);

    // valve_close_servo(&TANWA_utility.servo_valve[1]);

    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_CLOSE_ID, data, 1);

    ESP_LOGI(TAG, "OXI CLOSE");
    vTaskDelay(80 / portTICK_PERIOD_MS);
    // valve_close_servo(&TANWA_utility.servo_valve[0]);
    uint8_t data_fuel[8] = {0};
    can_send_message(CAN_SOL_SERVO_CLOSE_ID, data_fuel, 1);

    sd_timer_change_period(TIMER_SD_DATA_PERIOD_MS);

    liquid_ignition_test_timer_stop();
}

static void on_hold(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_HOLD);
    ESP_LOGI(TAG, "ON HOLD");
}

static void on_abort(void *arg) {
    //led_state_display_state_update(&TANWA_utility.led_state_display, LED_STATE_DISPLAY_STATE_ABORT);
    buzzer_timer_change_period(1000);
    ESP_LOGI(TAG, "ON ABORT");
    igniter_disarm(&tanwa_hardware.igniter[0]);
    igniter_disarm(&tanwa_hardware.igniter[1]);
    // solenoid_driver_valve_close(&TANWA_utility.solenoid_driver, SOLENOID_DRIVER_VALVE_FILL);
    // solenoid_driver_valve_open(&TANWA_utility.solenoid_driver, SOLENOID_DRIVER_VALVE_DEPR);
    // valve_close_servo(&TANWA_utility.servo_valve[1]);
    // valve_close_servo(&TANWA_utility.servo_valve[0]);
    // twai_message_t servo_mess = {
    //     .identifier = CAN_FAC_SERVO_CLOSE,
    //     .data_length_code = 1,                  
    //     .data = {2, 0, 0, 0, 0, 0, 0, 0}
    // };
    // twai_transmit(&servo_mess, pdMS_TO_TICKS(100));
}

static state_config_t states_cfg[] = {
    {INIT, on_init, NULL},
    {IDLE, on_idle, NULL},
    {ARMED_TO_LAUNCH, on_armed_to_launch, NULL},
    {RDY_TO_LAUNCH, on_ready_to_lauch, NULL},
    {COUNTDOWN, on_countdown, NULL},
    {FIRE, on_fire, NULL},
    {AFTER_BURNOUT, on_after_burnout, NULL},
    {HOLD, on_hold, NULL},
    {ABORT, on_abort, NULL},
};

bool initialize_state_machine(void) {
    ESP_LOGI(TAG, "Initializing state machine");
    state_machine_task_cfg_t task_cfg = {
        .stack_depth = CONFIG_SM_TASK_STACK_DEPTH,
        .core_id = CONFIG_SM_TASK_CORE_ID,
        .priority = CONFIG_SM_TASK_PRIORITY,
    };

    state_machine_status_t status = STATE_MACHINE_OK;
    state_machine_init();
    status |= state_machine_set_states(states_cfg, sizeof(states_cfg)/sizeof(states_cfg[0]));
    status |= state_machine_run(&task_cfg);
    return status == STATE_MACHINE_OK ? true : false;
}

void get_state_text(int32_t state, char *text) {
    switch (state) {
        case INIT:
            strcpy(text, "INIT");
            break;
        case IDLE:
            strcpy(text, "IDLE");
            break;
        // case RECOVERY_ARM:
        //     strcpy(text, "RECOVERY_ARM");
        //     break;
        // case FUELING:
        //     strcpy(text, "FUELING");
        //     break;
        case ARMED_TO_LAUNCH:
            strcpy(text, "ARMED_TO_LAUNCH");
            break;
        case RDY_TO_LAUNCH:
            strcpy(text, "RDY_TO_LAUNCH");
            break;
        case COUNTDOWN:
            strcpy(text, "COUNTDOWN");
            break;
        case FIRE:
            strcpy(text, "FIRE");
            break;
        // case FIRST_STAGE_RECOVERY:
        //     strcpy(text, "FIRST_STAGE_RECOVERY");
        //     break;
        // case SECOND_STAGE_RECOVERY:
        //     strcpy(text, "SECOND_STAGE_RECOVERY");
        //     break;
        case AFTER_BURNOUT:
            strcpy(text, "AFTER_BURNOUT");
            break;
        case HOLD:
            strcpy(text, "HOLD");
            break;
        case ABORT:
            strcpy(text, "ABORT");
            break;
        default:
            strcpy(text, "UNKNOWN");
            break;
    }
}