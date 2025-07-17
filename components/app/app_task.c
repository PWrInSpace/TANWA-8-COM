#include "app_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include "driver/twai.h"

#include "esp_log.h"

#include "can_api.h"
#include "can_commands.h"
#include "board_data.h"
#include "mcu_adc_config.h"
#include "igniter_driver.h"
#include "board_config.h"
#include "state_machine.h"
#include "abort_button.h"
#include "relay_driver.h"
#include "tmp1075.h"
#include "system_timer.h"
    #include "mission_timer.h"
#include "mission_timer_config.h"
#include "timers_config.h"

#define APP_TASK_STACK_SIZE CONFIG_APP_TASK_STACK_SIZE
#define APP_TASK_PRIORITY CONFIG_APP_TASK_PRIORITY
#define APP_TASK_CORE_ID CONFIG_APP_TASK_CORE_ID
#define APP_TASK_FREQUENCY CONFIG_APP_TASK_FREQUENCY

#define TAG "APP_TASK"

static TaskHandle_t app_task_handle = NULL;
static volatile TickType_t app_task_freq = APP_TASK_FREQUENCY;
static SemaphoreHandle_t app_task_freq_mutex = NULL;

esp_err_t app_task_init(void) {

    app_task_freq_mutex = xSemaphoreCreateMutex();
    
    if(xTaskCreatePinnedToCore(app_task, "app_task", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, &app_task_handle, APP_TASK_CORE_ID) == pdPASS) {
        ESP_LOGI("APP_TASK", "App task created successfully");
    } else {
        ESP_LOGE("APP_TASK", "Failed to create app task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t app_task_deinit(void) {
    if (app_task_handle != NULL) {
        vTaskDelete(app_task_handle);
        app_task_handle = NULL;
    }
    
    return ESP_OK;
}

void change_app_task_period(uint32_t period_ms) {
    if (xSemaphoreTake(app_task_freq_mutex, (TickType_t) 10) == pdTRUE) {
        app_task_freq = (TickType_t) period_ms;
        xSemaphoreGive(app_task_freq_mutex);
    }
}

void app_task(void *arg) {

    igniter_continuity_t ign_cnt_1, ign_cnt_2;
    float i_sense;

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t local_freq;

    while(1) {

        if (xSemaphoreTake(app_task_freq_mutex, (TickType_t) 10) == pdTRUE) {
            local_freq = app_task_freq;
            xSemaphoreGive(app_task_freq_mutex);
        } else {
            local_freq = APP_TASK_FREQUENCY;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(local_freq));

        tanwa_data_t tanwa_data = tanwa_data_read();

        uint64_t dc_timer_expire = 0;
        if (sys_timer_get_expiry_time(TIMER_DISCONNECT, &dc_timer_expire) == false) {
            tanwa_data.uptime = TIMER_DISCONNECT_PERIOD_MS;
        } else {
            tanwa_data.uptime = ((dc_timer_expire / 1000) - esp_timer_get_time() / 1000.0) / 1000.0;
        }

        tanwa_data.engine_work_time = liquid_ignition_test_timer_get_time();

        // Update tanwa data
        tanwa_data_update(&tanwa_data);
    
        uint8_t data[8] = {0};
        can_send_message(CAN_SOL_GET_DATA_ID, data, 0);
        can_send_message(CAN_SOL_GET_STATUS_ID, data, 0);
        can_send_message(CAN_POWER_GET_DATA_ID, data, 0);
        can_send_message(CAN_POWER_GET_STATUS_ID, data, 0);
        can_send_message(CAN_SENSOR_GET_DATA_ID, data, 0);
        can_send_message(CAN_SENSOR_GET_STATUS_ID, data, 0);
        can_send_message(CAN_SENSOR_GET_TEMPERATURE_ID, data, 0);
        can_send_message(CAN_UTIL_GET_DATA_ID, data, 0);
        can_send_message(CAN_UTIL_GET_STATUS_ID, data, 0);
        uint8_t data_weights[8] = {1, 2, 3, 4, 5, 6, 7, 8}; // Example data, adjust as needed
        can_send_message(CAN_WEIGHTS_GET_ADS_CH_WEIGHT_ID, data, 2);
        can_send_message(CAN_WEIGHTS_GET_STATUS_ID, data, 0);

        //tanwa_read_i_sense(&i_sense);

        // Check igniter continuity
        igniter_check_continuity(&(tanwa_hardware.igniter[0]), &ign_cnt_1);
        igniter_check_continuity(&(tanwa_hardware.igniter[1]), &ign_cnt_2);

        uint8_t abort_button_state;
        bool abort_button = false;
        abort_button_get_level(&abort_button_state);
        if (abort_button_state == 0) {
            abort_button = true;
        } else {
            abort_button = false;
        }

        tanwa_data_update_state((uint8_t) state_machine_get_current_state());

        com_data_t com_data = tanwa_data_read_com_data();
        //com_data.i_sense = i_sense;
        com_data.abort_button = abort_button;
        com_data.igniter_cont_1 = (ign_cnt_1 == IGNITER_CONTINUITY_OK) ? true : false;
        com_data.igniter_cont_2 = (ign_cnt_2 == IGNITER_CONTINUITY_OK) ? true : false;
        relay_driver_state_t relay1_state, relay2_state, relay3_state, relay4_state;
        relay_get_state(&tanwa_hardware.relay[0], &relay1_state);
        relay_get_state(&tanwa_hardware.relay[1], &relay2_state);
        relay_get_state(&tanwa_hardware.relay[2], &relay3_state);
        relay_get_state(&tanwa_hardware.relay[3], &relay4_state);
        com_data.relay_state1 = (relay1_state == RELAY_ON) ? true : false;
        com_data.relay_state2 = (relay2_state == RELAY_ON) ? true : false;
        com_data.relay_state3 = (relay3_state == RELAY_ON) ? true : false;
        com_data.relay_state4 = (relay4_state == RELAY_ON) ? true : false;
        float temperature_1, temperature_2;
        tmp1075_get_temp_celsius(&(tanwa_hardware.tmp1075[0]), &temperature_1);
        tmp1075_get_temp_celsius(&(tanwa_hardware.tmp1075[1]), &temperature_2);
        com_data.temperature_1 = temperature_1;
        com_data.temperature_2 = temperature_2;
        tanwa_data_update_com_data(&com_data);

    }
}