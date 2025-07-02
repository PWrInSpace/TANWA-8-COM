///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 14.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "commands.h"

#include <string.h>

#include "esp_log.h"

#define TAG "CMD"

static bool init(cmd_t *cmd) {
    assert(cmd->devices != NULL);
    assert(cmd->number_of_devices > 0);

    if (cmd->devices == NULL || cmd->number_of_devices == 0) {
        return false;
    }

    return true;
}

bool cmd_init_standard_mode(cmd_t *cmd) {
    cmd->lora_dev_id = CMD_BROADCAST_DEV_ID;  // set unused id

    return init(cmd);
}

bool cmd_init_lora_mode(cmd_t *cmd) {
    assert(cmd->lora_dev_id != CMD_BROADCAST_DEV_ID);
    assert(~(cmd->lora_dev_id & 0x01));
    if (cmd->lora_dev_id == CMD_BROADCAST_DEV_ID) {
        ESP_LOGE(TAG, "Invalid dev id");
        return false;
    }

    if (cmd->lora_dev_id & 0x01) {  // check if id include privilage mask
        ESP_LOGE(TAG, "Lora Dev ID include privilage mask");
        return false;
    }

    return init(cmd);
}

static bool get_device_index(cmd_t *cmd, cmd_sys_dev_id dev_id, size_t *dev_index) {
    for (size_t i = 0; i < cmd->number_of_devices; ++i) {
        if (cmd->devices[i].dev_id == dev_id) {
            *dev_index = i;
            return true;
        }
    }
    return false;
}

static bool find_command_and_execute(
                                    cmd_device_t *dev_cmd,
                                    cmd_message_t *received_command,
                                    bool privilage) {
    for (size_t i = 0; i < dev_cmd->number_of_cmd; ++i) {
        if (dev_cmd->cmd[i].command_id == received_command->cmd.command) {
            if (dev_cmd->cmd[i].on_command_receive_fnc != NULL) {
                dev_cmd->cmd[i].on_command_receive_fnc(
                    received_command->cmd.command, received_command->cmd.payload, privilage);
            }
            return true;
        }
    }

    return false;
}

static bool process_command(cmd_t *cmd,
                            cmd_sys_dev_id dev_id,
                            cmd_message_t *message,
                            bool privilage) {
    size_t index = 0;
    if (get_device_index(cmd, dev_id, &index) == false) {
        return false;
    }

    return find_command_and_execute(&cmd->devices[index], message, privilage);
}

bool cmd_process_command(cmd_t *cmd, cmd_sys_dev_id dev_id, cmd_message_t *message) {
    return process_command(cmd, dev_id, message, false);
}

static bool check_lora_dev_id(cmd_t *cmd, cmd_lora_dev_id dev_id) {
    if (dev_id == 0x00 || dev_id == 0x01) {
        return true;
    }
    // remove privilage mask and check id
    return (dev_id >> 1) == (cmd->lora_dev_id >> 1) ? true : false;
}

static bool check_dev_id_privilage_mode(cmd_lora_dev_id dev_id) {
    return (dev_id & CMD_PRIVILAGE_MASK) > 0 ? true : false;
}

bool cmd_process_lora_command(
                                cmd_t *cmd,
                                cmd_lora_dev_id lora_dev_id,
                                cmd_sys_dev_id dev_id,
                                cmd_message_t *message){
    if (cmd->lora_dev_id == CMD_BROADCAST_DEV_ID) {
        ESP_LOGE(TAG, "Library hasn't been initialized in LoRa mode");
        return false;
    }

    if (check_lora_dev_id(cmd, lora_dev_id) == false) {
        ESP_LOGE(TAG, "Incorrect lora dev id");
        return false;
    }

    if (check_dev_id_privilage_mode(lora_dev_id) == true) {
        return process_command(cmd, dev_id, message, true);
    }

    return process_command(cmd, dev_id, message, false);
}

cmd_message_t cmd_create_message(uint32_t command, int32_t payload) {
    cmd_message_t message = {.cmd.command = command, .cmd.payload = payload};

    return message;
}