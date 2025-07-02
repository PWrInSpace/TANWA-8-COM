///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 12.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the LED state display utility. It is based on the LEDs 
/// connected to the MCP23018 I2C GPIO expander. The LEDs are used to display the state of the
/// system - from 1 to 10.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_LED_STATE_DISPLAY_H_
#define PWRINSPACE_LED_STATE_DISPLAY_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "mcp23018.h"

#define LED_STATE_DISPLAY_TANWA_CONFIG()        \
    {                                           \
        .mcp23018 = NULL,                       \
        .state = LED_STATE_DISPLAY_STATE_NONE,  \
    }

typedef enum {
    LED_STATE_DISPLAY_STATE_NONE = 0,
    LED_STATE_DISPLAY_STATE_IDLE = 1,
    LED_STATE_DISPLAY_STATE_ARMED = 2,
    LED_STATE_DISPLAY_STATE_FUELING = 3,
    LED_STATE_DISPLAY_STATE_ARMED_TO_LAUNCH = 4,
    LED_STATE_DISPLAY_STATE_RDY_TO_LAUNCH = 5,
    LED_STATE_DISPLAY_STATE_COUTDOWN = 6,
    LED_STATE_DISPLAY_STATE_FLIGHT = 7,
    LED_STATE_DISPLAY_STATE_FIRST_STAGE = 8,
    LED_STATE_DISPLAY_STATE_SECOND_STAGE = 9,
    LED_STATE_DISPLAY_STATE_ON_GROUND = 10,
    LED_STATE_DISPLAY_STATE_HOLD = 11,
    LED_STATE_DISPLAY_STATE_ABORT = 12
} led_state_display_state_t;

typedef enum {
    LED_STATE_DISPLAY_PORT_LED_1 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_2 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_3 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_4 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_5 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_6 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_7 = MCP23018_PORT_B,
    LED_STATE_DISPLAY_PORT_LED_8 = MCP23018_PORT_A,
    LED_STATE_DISPLAY_PORT_LED_9 = MCP23018_PORT_A,
    LED_STATE_DISPLAY_PORT_LED_10 = MCP23018_PORT_A,
    LED_STATE_DISPLAY_PORT_LED_11 = MCP23018_PORT_A,
    LED_STATE_DISPLAY_PORT_LED_12 = MCP23018_PORT_A,
    LED_STATE_DISPLAY_PORT_LED_13 = MCP23018_PORT_A
} led_state_display_led_port_t;

typedef enum {
    LED_STATE_DISPLAY_PIN_LED_1 = MCP23018_PIN_7,
    LED_STATE_DISPLAY_PIN_LED_2 = MCP23018_PIN_6,
    LED_STATE_DISPLAY_PIN_LED_3 = MCP23018_PIN_5,
    LED_STATE_DISPLAY_PIN_LED_4 = MCP23018_PIN_4,
    LED_STATE_DISPLAY_PIN_LED_5 = MCP23018_PIN_3,
    LED_STATE_DISPLAY_PIN_LED_6 = MCP23018_PIN_2,
    LED_STATE_DISPLAY_PIN_LED_7 = MCP23018_PIN_1,
    LED_STATE_DISPLAY_PIN_LED_8 = MCP23018_PIN_1,
    LED_STATE_DISPLAY_PIN_LED_9 = MCP23018_PIN_2,
    LED_STATE_DISPLAY_PIN_LED_10 = MCP23018_PIN_3,
    LED_STATE_DISPLAY_PIN_LED_11 = MCP23018_PIN_4,
    LED_STATE_DISPLAY_PIN_LED_12 = MCP23018_PIN_5,
    LED_STATE_DISPLAY_PIN_LED_13 = MCP23018_PIN_6
} led_state_display_led_pin_t;

typedef enum {
    LED_STATE_DISPLAY_OK = 0,
    LED_STATE_DISPLAY_FAIL = 1,
    LED_STATE_DISPLAY_WRITE_ERR = 3,
} led_state_display_status_t;

typedef struct {
    mcp23018_struct_t *mcp23018;
    led_state_display_state_t state;
} led_state_display_struct_t;

led_state_display_status_t led_state_display_clear(led_state_display_struct_t *led_state_display);

led_state_display_status_t led_state_display_state_update(led_state_display_struct_t *led_state_display, led_state_display_state_t state);

#endif  /* PWRINSPACE_LED_STATE_DISPLAY_H_ */