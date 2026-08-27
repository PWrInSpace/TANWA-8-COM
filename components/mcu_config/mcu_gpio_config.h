///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 22.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains the configuration of the GPIO pins of the MCU.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_MCU_GPIO_CONFIG_H_
#define PWRINSPACE_MCU_GPIO_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "rom/gpio.h"
#include "soc/gpio_struct.h"

//ADD CONFIGURED GPIO PINS HERE
typedef enum {
    LED_GPIO = CONFIG_GPIO_LED,
    LORA_CS_GPIO = CONFIG_LORA_CS,
    LORA_RS_GPIO = CONFIG_LORA_RS,
    LORA_D0_GPIO = CONFIG_LORA_D0,
    ABORT_GPIO = CONFIG_GPIO_ABORT,
    ARM1_GPIO = CONFIG_GPIO_ARM1,
    ARM2_GPIO = CONFIG_GPIO_ARM2,
    FIRE_GPIO = CONFIG_GPIO_FIRE,
    RELAY_1_GPIO = CONFIG_RELAY_1_GPIO,
    RELAY_2_GPIO = CONFIG_RELAY_2_GPIO,
    RELAY_3_GPIO = CONFIG_RELAY_3_GPIO,
    RELAY_4_GPIO = CONFIG_RELAY_4_GPIO,
    CAN_STB_GPIO = CONFIG_CAN_STB,
} mcu_gpio_cfg_t;

// ADD GPIO PINS INDICES HERE
typedef enum {
    LED_GPIO_INDEX = 0,
    LORA_CS_GPIO_INDEX,
    LORA_RS_GPIO_INDEX,
    LORA_D0_GPIO_INDEX,
    ABORT_GPIO_INDEX,
    ARM1_GPIO_INDEX,
    ARM2_GPIO_INDEX,
    FIRE_GPIO_INDEX,
    RELAY_1_GPIO_INDEX,
    RELAY_2_GPIO_INDEX,
    RELAY_3_GPIO_INDEX,
    RELAY_4_GPIO_INDEX,
    CAN_STB_GPIO_INDEX,
    MAX_GPIO_INDEX,
} mcu_gpio_index_cfg_t;

typedef struct {
    uint8_t pins[MAX_GPIO_INDEX];
    uint8_t num_pins;
    gpio_config_t configs[MAX_GPIO_INDEX];
} mcu_gpio_config_t;

esp_err_t mcu_gpio_init(void);

bool _mcu_gpio_set_level(uint8_t gpio, uint8_t level);

bool _mcu_gpio_get_level(uint8_t gpio, uint8_t* level);

bool _abort_gpio_attach_isr(gpio_isr_t interrupt_cb);

bool _lora_gpio_attach_d0_isr(gpio_isr_t interrupt_cb);

bool _lora_gpio_set_level(uint8_t gpio, uint8_t level);

bool _relay_gpio_set_level(uint8_t gpio, bool level);

#endif /* PWRINSPACE_MCU_GPIO_CONFIG_H_ */