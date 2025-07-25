///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 02.06.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "cmd_commands.h"

#include "esp_log.h"

#include "state_machine_config.h"
#include "board_config.h"

#include "lora_task.h"

#include "can_commands.h"
#include "can_api.h"

#include "timers_config.h"
#include "system_timer.h"
#include "settings_mem.h"
#include "mission_timer_config.h"
#include "relay_driver.h"

#include "board_data.h"

#define TAG "CMD_COMMANDS"

extern led_state_display_struct_t led_state_display;

void tanwa_state_change(int32_t state) {
    if (state_machine_get_current_state() == ABORT) {
        ESP_LOGW(TAG, "SM | System in ABORT state");
        return;
    }
    state_machine_status_t sm_status = state_machine_change_state(state);
    if (sm_status != STATE_MACHINE_OK) {
        ESP_LOGE(TAG, "SM | State change error | %d", (uint8_t)sm_status);
    }
}

void tanwa_abort(void) {
    state_t curr_state = state_machine_get_current_state();
    state_t prev_state = state_machine_get_previous_state();
    if (curr_state == ABORT) {
        ESP_LOGW(TAG, "SM | Already in ABORT state");
        return;
    }
    if (curr_state > COUNTDOWN && curr_state < HOLD) {
        ESP_LOGW(TAG, "SM | Abort not possible in FLIGHT");
        return;
    }
    state_machine_status_t sm_status = state_machine_force_change_state(ABORT);
    if (sm_status != STATE_MACHINE_OK) {
        ESP_LOGE(TAG, "SM | State force ABORT error | %d", (uint8_t)sm_status);
    }
}

void tanwa_hold_in(void) {
    state_t curr_state = state_machine_get_current_state();
    state_t prev_state = state_machine_get_previous_state();
    if (curr_state == HOLD) {
        ESP_LOGW(TAG, "SM | Already in HOLD state");
        return;
    }
    if (curr_state == ABORT) {
        ESP_LOGW(TAG, "SM | System in ABORT state");
        return;
    }
    if (curr_state > COUNTDOWN && curr_state < HOLD) {
        ESP_LOGW(TAG, "SM | Hold not possible in FLIGHT");
        return;
    }
    state_machine_status_t sm_status = state_machine_force_change_state(HOLD);
    if (sm_status != STATE_MACHINE_OK) {
        ESP_LOGE(TAG, "SM | State force HOLD error | %d", (uint8_t)sm_status);
    }
}

void tanwa_hold_out(void) {
    state_t curr_state = state_machine_get_current_state();
    state_t prev_state = state_machine_get_previous_state();
    if (curr_state != HOLD) {
        ESP_LOGW(TAG, "SM | Not in HOLD state");
        return;
    }
    if (prev_state == COUNTDOWN) {
        state_machine_status_t sm_status = state_machine_force_change_state(RDY_TO_LAUNCH);
        if (sm_status != STATE_MACHINE_OK) {
            ESP_LOGE(TAG, "SM | State force RDY_TO_LAUNCH error | %d", sm_status);
        }
    } else {
        state_machine_status_t sm_status = state_machine_change_to_previous_state(true);
        if (sm_status != STATE_MACHINE_OK) {
            ESP_LOGE(TAG, "SM | State change error | %d",(uint8_t) sm_status);
        }
    }
}

void tanwa_soft_restart_rck(void) {
    can_send_message(CAN_WEIGHTS_RESET_ID, NULL, 0);
}

void tanwa_tare_weight(void){
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_WEIGHTS_ADS_TARE_ID, data, 1);
    data[0] = 1; // Tare for ADS1
    can_send_message(CAN_WEIGHTS_ADS_TARE_ID, data, 1);
}

void tanwa_set_offset_weight(float offset) {
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    memcpy(&data[1], &offset, sizeof(float));
    can_send_message(CAN_WEIGHTS_SET_ADS_OFFSET_ID, data, 3);
    data[0] = 1; // Set offset for ADS1
    can_send_message(CAN_WEIGHTS_SET_ADS_OFFSET_ID, data, 3);
}

void tanwa_soft_arm(void) {
    com_data_t data = tanwa_data_read_com_data();
    igniter_status_t ign_status = IGNITER_OK;
    ign_status = igniter_arm(&(tanwa_hardware.igniter[0]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter arm error | %d", (uint8_t)ign_status);
    }
    ign_status = igniter_arm(&(tanwa_hardware.igniter[1]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter arm error | %d", (uint8_t)ign_status);
    }

    if(ign_status == IGNITER_OK){
        data.arm_state = true;
        ESP_LOGI(TAG, "IGN | Igniter armed successfully");
    }
    else{
        data.arm_state = false;
        ESP_LOGE(TAG, "IGN | Igniter arm failed");
    }

    tanwa_data_update_com_data(&data);
}

void tanwa_soft_disarm(void) {

    com_data_t data = tanwa_data_read_com_data();
    igniter_status_t ign_status = IGNITER_OK;
    ign_status = igniter_disarm(&(tanwa_hardware.igniter[0]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter disarm error | %d", (uint8_t)ign_status);
    }
    ign_status = igniter_disarm(&(tanwa_hardware.igniter[1]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter disarm error | %d", (uint8_t)ign_status);
    }

    data.arm_state = IGNITER_OK == ign_status ? false : true;

    tanwa_data_update_com_data(&data);
}

void tanwa_fire(void) {
    igniter_status_t ign_status = IGNITER_OK;
    ign_status = igniter_fire(&(tanwa_hardware.igniter[0]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter fire error | %d", ign_status);
    }
    ign_status = igniter_fire(&(tanwa_hardware.igniter[1]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter fire error | %d", ign_status);
    }

    if(ign_status == IGNITER_OK){
        com_data_t data = tanwa_data_read_com_data();
        data.arm_state = false;
        tanwa_data_update_com_data(&data);
    }
}

void tanwa_soft_restart_esp(void) {
    esp_restart();
}

void tanwa_fill(uint8_t valve_state) {
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (valve_state == CMD_VALVE_OPEN) {
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    } else if (valve_state == CMD_VALVE_CLOSE) {
        can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
    } else {
        ESP_LOGE(TAG, "Invalid fill valve state");
    }
}

void tanwa_fill_time(uint32_t open_time) {
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t open_time_scaled = (uint16_t)open_time;
    memcpy(&data[1], &open_time_scaled, sizeof(uint16_t));
    can_send_message(CAN_SOL_OPEN_SOL_ID, data, 3);
}

void tanwa_fill_n2(uint8_t valve_state) {
    uint8_t data[8] = {2, 0, 0, 0, 0, 0, 0, 0};
    if (valve_state == CMD_VALVE_OPEN) {
        can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 1);
    } else if (valve_state == CMD_VALVE_CLOSE) {
        can_send_message(CAN_SOL_SERVO_CLOSE_ID, data, 1);
    } else {
        ESP_LOGE(TAG, "Invalid fill valve state");
    }
}

void tanwa_fill_n2_time(uint32_t open_time) {
    uint8_t data[8] = {2, 0, 0, 0, 0, 0, 0, 0};
    uint16_t open_time_scaled = (uint16_t)open_time;
    memcpy(&data[1], &open_time_scaled, sizeof(uint16_t));
    can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 3);
}

void tanwa_depr(uint8_t valve_state) {
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    if (valve_state == CMD_VALVE_OPEN) {
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    } else if (valve_state == CMD_VALVE_CLOSE) {
        can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
    } else {
        ESP_LOGE(TAG, "Invalid depr valve state");
    }
}

void tanwa_depr_time(uint32_t open_time) {
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    uint16_t open_time_scaled = (uint16_t)open_time;
    memcpy(&data[1], &open_time_scaled, sizeof(uint16_t));
    can_send_message(CAN_SOL_OPEN_SOL_ID, data, 3);
}

void tanwa_depr_n2(uint8_t valve_state) {
    uint8_t data[8] = {3, 0, 0, 0, 0, 0, 0, 0};
    if (valve_state == CMD_VALVE_OPEN) {
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    } else if (valve_state == CMD_VALVE_CLOSE) {
        can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
    } else {
        ESP_LOGE(TAG, "Invalid depr valve state");
    }
}

void tanwa_depr_n2_time(uint32_t open_time) {
    uint8_t data[8] = {3, 0, 0, 0, 0, 0, 0, 0};
    uint16_t open_time_scaled = (uint16_t)open_time;
    memcpy(&data[1], &open_time_scaled, sizeof(uint16_t));
    can_send_message(CAN_SOL_OPEN_SOL_ID, data, 3);
}

void tanwa_qd_n2o(uint8_t qd_cmd) {
    uint8_t data[8] = {4, 0, 0, 0, 0, 0, 0, 0};
    if(qd_cmd == CMD_QD_UNPLUG) {
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    } else if(qd_cmd == CMD_QD_STOP) {
        can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
    } else {
        ESP_LOGE(TAG, "Invalid QD N2O command");
    }
}

void tanwa_qd_n2(uint8_t qd_cmd) {
    uint8_t data[8] = {5, 0, 0, 0, 0, 0, 0, 0};
    if(qd_cmd == CMD_QD_UNPLUG) {
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    } else if(qd_cmd == CMD_QD_STOP) {
        can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
    } else {
        ESP_LOGE(TAG, "Invalid QD O2 command");
    }
}

void tanwa_heating_tank_start(void) {
    relay_driver_err_t err = relay_open(&(tanwa_hardware.relay[0]));
    if (err != RELAY_DRIVER_OK) {
        ESP_LOGE(TAG, "Relay open error | %d", (uint8_t)err);
    } else {
        ESP_LOGI(TAG, "Heating tank started");
    }
}

void tanwa_heating_tank_stop(void) {
    relay_driver_err_t err = relay_close(&(tanwa_hardware.relay[0]));
    if (err != RELAY_DRIVER_OK) {
        ESP_LOGE(TAG, "Relay close error | %d", (uint8_t)err);
    }
}

void tanwa_heating_valve_start(void) {
    relay_driver_err_t err = relay_open(&(tanwa_hardware.relay[1]));
    if (err != RELAY_DRIVER_OK) {
        ESP_LOGE(TAG, "Relay open error | %d", (uint8_t)err);
    } else {
        ESP_LOGI(TAG, "Heating valve started");
    }
}
void tanwa_heating_valve_stop(void) {
    relay_driver_err_t err = relay_close(&(tanwa_hardware.relay[1]));
    if (err != RELAY_DRIVER_OK) {
        ESP_LOGE(TAG, "Relay close error | %d", (uint8_t)err);
    } else {
        ESP_LOGI(TAG, "Heating valve stopped");
    }
}

bool lora_command_parsing(uint32_t lora_id, uint32_t command, int32_t payload) {

    ESP_LOGI(TAG, "LORA | Command parsing | ID: %d, CMD: %d, PAYLOAD: %d", lora_id, command, payload);
    if (lora_id == LORA_DEV_ID_ALL || lora_id == LORA_DEV_ID_ALL_SUDO || 
        lora_id == LORA_DEV_ID_TANWA || lora_id == LORA_DEV_ID_TANWA_SUDO) { 

        settings_read_all();
        Settings settings = settings_get_all();

        // Check if the command is for this device
        ESP_LOGI(TAG, "LORA | Command for TANWA");
        switch (command) {
            case CMD_STATE_CHANGE: {
                ESP_LOGI(TAG, "LORA | State change | %d", payload);
                state_t state = (state_t) payload;
                char state_text[20];
                get_state_text(state, state_text);
                ESP_LOGI(TAG, "LORA | State change | %s", state_text);
                tanwa_state_change(payload);
                break;
            }
            case CMD_ABORT: {
                ESP_LOGI(TAG, "LORA | Abort");
                tanwa_abort();
                break;
            }
            case CMD_HOLD_IN: {
                ESP_LOGI(TAG, "LORA | Hold in");
                tanwa_hold_in();
                break;
            }
            case CMD_HOLD_OUT: {
                ESP_LOGI(TAG, "LORA | Hold out");
                tanwa_hold_out();
                break;
            }
            case CMD_LORA_TRANSMIT_F: {
                ESP_LOGI(TAG, "LORA | Transmit F");
                lora_change_frequency(payload);
                break;
            }
            case CMD_LORA_TRANSMIT_T: {
                ESP_LOGI(TAG, "LORA | Transmit P");
                //lora_change_period(payload);
                break;
            }
            case CMD_COUNTDOWN: {
                ESP_LOGI(TAG, "LORA | Countdown");
                settings.countdownTime = payload;
                settings_save(SETTINGS_COUNTDOWN_TIME, settings.countdownTime);
                liquid_ignition_test_timer_set_disable_val(settings.countdownTime);
                break;
            }
            case CMD_SEND_SETTINGS: {
                ESP_LOGI(TAG, "LORA | Send settings");
                break;
            }
            case CMD_RESET_ERRORS: {
                ESP_LOGI(TAG, "LORA | Reset errors");
                break;
            }
            case CMD_FLASH_FORMAT: {
                ESP_LOGI(TAG, "LORA | Flash format");
                break;
            }
            case CMD_RESET: {
                ESP_LOGI(TAG, "LORA | Reset");
                esp_restart();
                break;
            }
            case CMD_DISCONNECT_TIMER: {
                ESP_LOGI(TAG, "LORA | Disconnect timer");
                if(!sys_timer_restart(TIMER_DISCONNECT, TIMER_DISCONNECT_PERIOD_MS)) {
                    ESP_LOGE(TAG, "LORA | Unable to restart timer");
                }
                break;
            }
            case CMD_SOFT_ARM: {
                ESP_LOGI(TAG, "LORA | Soft arm");
                tanwa_soft_arm();
                break;
            }
            case CMD_SOFT_DISARM: {
                ESP_LOGI(TAG, "LORA | Soft disarm");
                tanwa_soft_disarm();
                break;
            }
            case CMD_RESTART_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Restart RCK");
                tanwa_soft_restart_rck();
                break;
            }
            case CMD_CALIBRATE_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Calibrate RCK");
                //tanwa_calibrate_weight((float)payload);
                break;
            }
            case CMD_TARE_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Tare RCK");
                tanwa_tare_weight();
                break;
            }
            case CMD_SET_CAL_FACTOR_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Set cal factor RCK");
                //tanwa_set_cal_factor_weight((float)payload);
                break;
            }
            case CMD_SET_OFFSET_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Set offset RCK");
                tanwa_set_offset_weight((float)payload);
                break;
            }
            case CMD_N2O_FILL_OPEN: {
                ESP_LOGI(TAG, "LORA | Fill open");
                tanwa_fill(CMD_VALVE_OPEN);
                break;
            }
            case CMD_N2O_FILL_CLOSE: {
                ESP_LOGI(TAG, "LORA | Fill close");
                tanwa_fill(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_N2O_FILL_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Fill open time");
                tanwa_fill_time((uint32_t)payload);
                break;
            }
            case CMD_N2_FILL_OPEN: {
                ESP_LOGI(TAG, "LORA | Fill open");
                tanwa_fill_n2(CMD_VALVE_OPEN);
                break;
            }
            case CMD_N2_FILL_CLOSE: {
                ESP_LOGI(TAG, "LORA | Fill close");
                tanwa_fill_n2(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_N2_FILL_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Fill open time");
                tanwa_fill_n2_time((uint32_t)payload);
                break;
            }
            case CMD_N2O_DEPR_OPEN: {
                ESP_LOGI(TAG, "LORA | Depr open");
                tanwa_depr(CMD_VALVE_OPEN);
                break;
            }
            case CMD_N2O_DEPR_CLOSE: {
                ESP_LOGI(TAG, "LORA | Depr close");
                tanwa_depr(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_N2O_DEPR_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Depr open time");
                tanwa_depr_time((uint32_t)payload);
                break;
            }
            case CMD_N2_DEPR_OPEN: {
                ESP_LOGI(TAG, "LORA | Depr open");
                tanwa_depr_n2(CMD_VALVE_OPEN);
                break;
            }
            case CMD_N2_DEPR_CLOSE: {
                ESP_LOGI(TAG, "LORA | Depr close");
                tanwa_depr_n2(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_N2_DEPR_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Depr open time");
                tanwa_depr_n2_time((uint32_t)payload);
                break;
            }
            case CMD_QD_N2O_UNPLUG: {
                ESP_LOGI(TAG, "LORA | QD N2O unplug");
                tanwa_qd_n2o(CMD_QD_UNPLUG);
                break;
            }
            case CMD_QD_N2O_STOP: {
                ESP_LOGI(TAG, "LORA | QD N2O stop");
                tanwa_qd_n2o(CMD_QD_STOP);
                break;
            }
            case CMD_QD_N2_UNPLUG: {
                ESP_LOGI(TAG, "LORA | QD N2 unplug");
                tanwa_qd_n2(CMD_QD_UNPLUG);
                break;
            }
            case CMD_QD_N2_STOP: {
                ESP_LOGI(TAG, "LORA | QD N2 stop");
                tanwa_qd_n2(CMD_QD_STOP);
                break;
            }
            case CMD_HEATING_TANK_START: {
                ESP_LOGI(TAG, "LORA | Heating tank start");
                tanwa_heating_tank_start();
                break;
            }
            case CMD_HEATING_TANK_STOP: {
                ESP_LOGI(TAG, "LORA | Heating tank stop");
                tanwa_heating_tank_stop();
                break;
            }
            case CMD_HEATING_VALVE_START: {
                ESP_LOGI(TAG, "LORA | Heating valve start");
                tanwa_heating_valve_start();
                break;
            }
            case CMD_HEATING_VALVE_STOP: {
                ESP_LOGI(TAG, "LORA | Heating valve stop");
                tanwa_heating_valve_stop();
                break;
            }
            case CMD_VENT_OPEN: {
                ESP_LOGI(TAG, "LORA | Vent open");
                tanwa_fill_n2(CMD_VALVE_OPEN);
                break;
            }
            case CMD_VENT_CLOSE: {
                ESP_LOGI(TAG, "LORA | Vent close");
                tanwa_fill_n2(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_VENT_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Vent open time");
                tanwa_fill_n2_time((uint32_t)payload);
                break;
            }
            default: {
                ESP_LOGI(TAG, "LORA command: %d", command);
                ESP_LOGW(TAG, "LORA | Unknown command");
                return false;
                break;
            }
        }
        return true;
    } else {
        ESP_LOGW(TAG, "LORA | Command for other device");
        return false;
    }
}