///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the system console configuration, including initialization
/// and available commands for debugging/testing purposes.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_BOARD_CONFIG_H
#define PWRINSPACE_BOARD_CONFIG_H

#include "led_driver.h"
#include "igniter_driver.h"
#include "led_state_display.h"
#include "mcu_gpio_config.h"
#include "mcp23018.h"
#include "tmp1075.h"
#include "esp_err.h"
#include "ens_config.h"
#include "relay_driver.h"

typedef struct {
    char board_name[32];
    led_struct_t status_led;
} board_config_t;

typedef struct {
    tmp1075_struct_t tmp1075[2];
    mcp23018_struct_t mcp23018;
    igniter_struct_t igniter[2];
    relay_driver_t relay[4];
} tanwa_hardware_dev_t;

extern board_config_t config;
extern tanwa_hardware_dev_t tanwa_hardware;
extern led_state_display_struct_t led_state_display;
extern uint16_t ens_periods[ENS_ENUM_MAX];


esp_err_t board_config_init(void);
esp_err_t tanwa_read_i_sense(float *i_sense);

#endif /* PWRINSPACE_BOARD_CONFIG_H */