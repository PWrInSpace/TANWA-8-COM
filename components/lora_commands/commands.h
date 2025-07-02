///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the commands for the system console.
///===-----------------------------------------------------------------------------------------===//
#ifndef PWRINSPACE_COMMANDS_H_
#define PWRINSPACE_COMMANDS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_BROADCAST_DEV_ID 0x00
#define CMD_PRIVILAGE_MASK 0x01

typedef uint32_t cmd_lora_dev_id;
typedef uint32_t cmd_sys_dev_id;
typedef uint32_t cmd_command_id;
typedef void (*cmd_on_command_receive)(uint32_t command, int32_t payload, bool privilage);

typedef struct {
    cmd_command_id command_id;
    cmd_on_command_receive on_command_receive_fnc;
} cmd_command_t;

typedef struct {
    cmd_sys_dev_id dev_id;
    cmd_command_t *cmd;
    size_t number_of_cmd;
} cmd_device_t;

typedef union {
    struct command {
        uint32_t command;
        int32_t payload;
    } cmd;

    uint8_t raw[sizeof(struct command)];
} cmd_message_t;

typedef struct {
    cmd_lora_dev_id lora_dev_id;
    cmd_device_t *devices;
    size_t number_of_devices;
} cmd_t;

/**
 * @brief Initialize cmd api without task

 *
 * @param cfg pointer to config
 * @return true :)
 * @return false :C
 */
bool cmd_init_standard_mode(cmd_t *cmd);

/**
 * @brief Initialize cmd api in lora mode
 *
 * @param cfg pointer to config
 * @return true :D
 * @return false :C
 */
bool cmd_init_lora_mode(cmd_t *cmd);

/**
 * @brief Process command in standard mode
 *
 * @param dev_id system dev id
 * @param message message to process
 * @return true :D
 * @return false :C
 */
bool cmd_process_command(
    cmd_t *cmd,
    cmd_sys_dev_id dev_id,
    cmd_message_t *message);

/**
 * @brief Process command in lora mode
 *
 * @param lora_dev_id lora dev id
 * @param dev_id system device id
 * @param message message to process
 * @return true :D
 * @return false :C
 */
bool cmd_process_lora_command(
    cmd_t *cmd,
    cmd_lora_dev_id lora_dev_id,
    cmd_sys_dev_id dev_id,
    cmd_message_t *message);

/**
 * @brief Create message
 *
 * @param cmd cmd
 * @param payload payload
 * @return cmd_message_t created message
 */
cmd_message_t cmd_create_message(uint32_t command, int32_t payload);

#ifdef __cplusplus
}
#endif

#endif /* PWRINSPACE_COMMANDS_H_ */