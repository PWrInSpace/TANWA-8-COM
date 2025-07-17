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

void tanwa_fill(uint8_t valve_cmd) {
    return;
}

void tanwa_fill_time(uint32_t open_time) {
    // solenoid_driver_status_t sol_status = SOLENOID_DRIVER_OK;
    // sol_status = solenoid_driver_valve_open(&(TANWA_utility.solenoid_driver), SOLENOID_DRIVER_VALVE_FILL);
    // if (sol_status != SOLENOID_DRIVER_OK) {
    //     ESP_LOGE(TAG, "SOL | Solenoid driver fill error | %d", sol_status);
    // }
    // vTaskDelay(pdMS_TO_TICKS(open_time));
    // sol_status = solenoid_driver_valve_close(&(TANWA_utility.solenoid_driver), SOLENOID_DRIVER_VALVE_FILL);
    // if (sol_status != SOLENOID_DRIVER_OK) {
    //     ESP_LOGE(TAG, "SOL | Solenoid driver fill error | %d", (uint8_t)sol_status);
    // }
}

void tanwa_depr(uint8_t valve_cmd) {
    // solenoid_driver_status_t sol_status = SOLENOID_DRIVER_OK;
    // if (valve_cmd == CMD_VALVE_OPEN) {
    //     sol_status = solenoid_driver_valve_open(&(TANWA_utility.solenoid_driver), SOLENOID_DRIVER_VALVE_DEPR);
    // } else if (valve_cmd == CMD_VALVE_CLOSE){
    //     sol_status = solenoid_driver_valve_close(&(TANWA_utility.solenoid_driver), SOLENOID_DRIVER_VALVE_DEPR);
    // } else {
    //     ESP_LOGE(TAG, "SOL | Invalid depr valve command | %d", valve_cmd);
    // }
    // if (sol_status != SOLENOID_DRIVER_OK) {
    //     ESP_LOGE(TAG, "SOL | Solenoid driver depr error | %d", (uint8_t)sol_status);
    // }
}

void tanwa_depr_time(uint32_t open_time) {
    // solenoid_driver_status_t sol_status = SOLENOID_DRIVER_OK;
    // sol_status = solenoid_driver_valve_open(&(TANWA_utility.solenoid_driver), SOLENOID_DRIVER_VALVE_DEPR);
    // if (sol_status != SOLENOID_DRIVER_OK) {
    //     ESP_LOGE(TAG, "SOL | Solenoid driver depr error | %d", sol_status);
    // }
    // vTaskDelay(pdMS_TO_TICKS(open_time));
    // sol_status = solenoid_driver_valve_close(&(TANWA_utility.solenoid_driver), SOLENOID_DRIVER_VALVE_DEPR);
    // if (sol_status != SOLENOID_DRIVER_OK) {
    //     ESP_LOGE(TAG, "SOL | Solenoid driver depr error | %d", (uint8_t)sol_status);
    // }
}

void tanwa_qd_1(uint8_t qd_cmd) {
    // if (qd_cmd == CMD_QD_PUSH) {
    //     twai_message_t fac_mess = CAN_FAC_QD_PUSH();
    //     can_task_add_message(&fac_mess);
    // } else if (qd_cmd == CMD_QD_STOP) {
    //     twai_message_t fac_mess = CAN_FAC_QD_STOP();
    //     can_task_add_message(&fac_mess);
    // } else if (qd_cmd == CMD_QD_PULL) {
    //     twai_message_t fac_mess = CAN_FAC_QD_PULL();
    //     can_task_add_message(&fac_mess);
    // } else {
    //     ESP_LOGE(TAG, "QD | Invalid command | %d", qd_cmd);
    // }
}

void tanwa_qd_2(uint8_t qd_cmd) {
    // // ToDo: send command to FAC vid CAN to push/pull quick disconnect
    // if (qd_cmd == CMD_QD_PUSH) {
    //     // ToDo: send push command to FAC
    // } else if (qd_cmd == CMD_QD_STOP) {
    //     // ToDo: send stop command to FAC
    // } else if (qd_cmd == CMD_QD_PULL) {
    //     // ToDo: send pull command to FAC
    // } else {
    //     ESP_LOGE(TAG, "QD 2 | Invalid command | %d", qd_cmd);
    // }
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

bool lora_command_parsing(uint32_t lora_id, uint32_t command, int32_t payload) {

    ESP_LOGI(TAG, "LORA | Command parsing | ID: %d, CMD: %d, PAYLOAD: %d", lora_id, command, payload);
    if (lora_id == LORA_DEV_ID_ALL || lora_id == LORA_DEV_ID_ALL_SUDO || 
        lora_id == LORA_DEV_ID_TANWA || lora_id == LORA_DEV_ID_TANWA_SUDO) { 

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
            // case CMD_COUNTDOWN: {
            //     ESP_LOGI(TAG, "LORA | Countdown");
            //     settings.countdownTime = payload;
            //     //settings_save(SETTINGS_COUNTDOWN_TIME, settings.countdownTime);
            //     //liquid_ignition_test_timer_set_disable_val(settings.countdownTime);
            //     break;
            // }
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
                //tanwa_soft_restart_rck();
                break;
            }
            case CMD_CALIBRATE_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Calibrate RCK");
                //tanwa_calibrate_weight((float)payload);
                break;
            }
            case CMD_TARE_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Tare RCK");
                //tanwa_tare_weight();
                break;
            }
            case CMD_SET_CAL_FACTOR_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Set cal factor RCK");
                //tanwa_set_cal_factor_weight((float)payload);
                break;
            }
            case CMD_SET_OFFSET_WEIGHT: {
                ESP_LOGI(TAG, "LORA | Set offset RCK");
                //tanwa_set_offset_weight((float)payload);
                break;
            }
            case CMD_FILL_OPEN: {
                ESP_LOGI(TAG, "LORA | Fill open");
                tanwa_fill(CMD_VALVE_OPEN);
                break;
            }
            case CMD_FILL_CLOSE: {
                ESP_LOGI(TAG, "LORA | Fill close");
                tanwa_fill(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_FILL_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Fill open time");
                tanwa_fill_time((uint32_t)payload);
                break;
            }
            case CMD_DEPR_OPEN: {
                ESP_LOGI(TAG, "LORA | Depr open");
                tanwa_depr(CMD_VALVE_OPEN);
                break;
            }
            case CMD_DEPR_CLOSE: {
                ESP_LOGI(TAG, "LORA | Depr close");
                tanwa_depr(CMD_VALVE_CLOSE);
                break;
            }
            case CMD_DEPR_OPEN_TIME: {
                ESP_LOGI(TAG, "LORA | Depr open time");
                tanwa_depr_time((uint32_t)payload);
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