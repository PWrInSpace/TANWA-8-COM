///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains implementation of the system console configuration, including initialization
/// and available commands for debugging/testing purposes.
///===-----------------------------------------------------------------------------------------===//

#include "esp_log.h"
#include "esp_system.h"

#include "console.h"
#include "console_config.h"
#include "board_config.h"
#include "board_data.h"
#include "igniter_driver.h"

#define TAG "CONSOLE_CONFIG"

extern tanwa_hardware_dev_t tanwa_hardware;

// example function to reset the device
int reset_device(int argc, char **argv) {
    ESP_LOGI(TAG, "Resetting device...");
    esp_restart();
    return 0;
}

int arm_igniters(int argc, char **argv) {
    // This function can be used to arm the igniter
    // Implementation depends on the specific hardware and requirements
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
    return 0;
}

int disarm_igniters(int argc, char **argv) {
    // This function can be used to disarm the igniter
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
    return 0;
}

 // Place for the console configuration

 static esp_console_cmd_t cmd [] = {
 // example command:
 // cmd     help description   hint  function      args
    {"reset", "Reset the device", NULL, reset_device, NULL, NULL, NULL},
    {"arm_igniters", "Arm the igniters", NULL, arm_igniters, NULL, NULL, NULL},
    {"disarm-igniters", "Disarm the igniters", NULL, disarm_igniters, NULL, NULL, NULL},
 };

esp_err_t console_config_init() {
    esp_err_t ret;
    ret = console_init();
    ret = console_register_commands(cmd, sizeof(cmd) / sizeof(cmd[0]));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(ret));
        return ret;
    }
    return ret;
}