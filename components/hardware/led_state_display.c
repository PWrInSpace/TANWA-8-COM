///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 12.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "led_state_display.h"

led_state_display_status_t led_state_display_clear(led_state_display_struct_t *led_state_display) {
    if (led_state_display == NULL) {
        return LED_STATE_DISPLAY_FAIL;
    }

    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_1, LED_STATE_DISPLAY_PIN_LED_1, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_2, LED_STATE_DISPLAY_PIN_LED_2, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_3, LED_STATE_DISPLAY_PIN_LED_3, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_4, LED_STATE_DISPLAY_PIN_LED_4, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_5, LED_STATE_DISPLAY_PIN_LED_5, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_6, LED_STATE_DISPLAY_PIN_LED_6, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_7, LED_STATE_DISPLAY_PIN_LED_7, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_8, LED_STATE_DISPLAY_PIN_LED_8, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_9, LED_STATE_DISPLAY_PIN_LED_9, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_10, LED_STATE_DISPLAY_PIN_LED_10, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_11, LED_STATE_DISPLAY_PIN_LED_11, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_12, LED_STATE_DISPLAY_PIN_LED_12, MCP23018_HIGH);
    mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_13, LED_STATE_DISPLAY_PIN_LED_13, MCP23018_HIGH);

    return LED_STATE_DISPLAY_OK;
}

led_state_display_status_t led_state_display_state_update(led_state_display_struct_t *led_state_display, led_state_display_state_t state) {
    if (led_state_display == NULL) {
        return LED_STATE_DISPLAY_FAIL;
    }

    if (led_state_display->state == state) {
        return LED_STATE_DISPLAY_OK;
    }

    led_state_display_status_t status = LED_STATE_DISPLAY_OK;
    mcp23018_status_t mcp_status = MCP23018_OK;

    switch (state) {
        case LED_STATE_DISPLAY_STATE_IDLE:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_1, LED_STATE_DISPLAY_PIN_LED_1, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_ARMED:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_2, LED_STATE_DISPLAY_PIN_LED_2, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_FUELING:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_3, LED_STATE_DISPLAY_PIN_LED_3, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_ARMED_TO_LAUNCH:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_4, LED_STATE_DISPLAY_PIN_LED_4, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_RDY_TO_LAUNCH:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_5, LED_STATE_DISPLAY_PIN_LED_5, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_COUTDOWN:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_6, LED_STATE_DISPLAY_PIN_LED_6, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_FLIGHT:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_7, LED_STATE_DISPLAY_PIN_LED_7, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_FIRST_STAGE:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_8, LED_STATE_DISPLAY_PIN_LED_8, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_SECOND_STAGE:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_9, LED_STATE_DISPLAY_PIN_LED_9, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_ON_GROUND:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_10, LED_STATE_DISPLAY_PIN_LED_10, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_HOLD:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_11, LED_STATE_DISPLAY_PIN_LED_11, MCP23018_LOW);
            break;
        case LED_STATE_DISPLAY_STATE_ABORT:
            status = led_state_display_clear(led_state_display);
            mcp_status = mcp23018_digital_write(led_state_display->mcp23018, LED_STATE_DISPLAY_PORT_LED_12, LED_STATE_DISPLAY_PIN_LED_12, MCP23018_LOW);
            break;
        default:
            break;
    }

    if (status != LED_STATE_DISPLAY_OK) {
        return status;
    }

    if (mcp_status != MCP23018_OK) {
        return LED_STATE_DISPLAY_WRITE_ERR;
    }

    led_state_display->state = state;

    return LED_STATE_DISPLAY_OK;
}