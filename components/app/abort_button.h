///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 19.03.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the system ABORT button, which is connected to the GPIO pin
/// of the MCU (ESP32) and is used to trigger the emergency stop of the system.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_ABORT_BUTTON_H_
#define PWRINSPACE_ABORT_BUTTON_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

typedef enum {
    ABORT_BUTTTON_POLARITY_POSITIVE = 0,
    ABORT_BUTTTON_POLARITY_NEGATIVE = 1
} abort_button_polarity_t;

typedef enum {
    ABORT_BUTTON_OK = 0,
    ABORT_BUTTON_ERROR = 1,
    ABORT_BUTTON_READ_ERROR = 2
} abort_button_status_t;

typedef bool (*abort_button_GPIO_get_level)(uint8_t _gpio_num, uint8_t *_level);
typedef void (*abort_button_delay)(uint32_t _ms);

typedef struct {
    abort_button_GPIO_get_level _gpio_get_level;
    abort_button_delay _delay;
    uint8_t gpio_idx;
    abort_button_polarity_t polarity;
} abort_button_struct_t;

abort_button_status_t abort_button_init();

abort_button_status_t abort_button_get_level(uint8_t *level);

void abort_button_callback(void* arg);

#endif /* PWRINSPACE_ABORT_BUTTON_H_ */