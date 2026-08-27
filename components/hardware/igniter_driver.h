///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the simple electric igniter driver. It is used to check 
/// igniter continuity, software arming of the igniter and firing the igniter.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_IGNITER_DRIVER_H_
#define PWRINSPACE_IGNITER_DRIVER_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define IGNITER_CONTINUITY_THRESHOLD 2000

typedef enum {
    IGNITER_STATE_WAITING = 0,
    IGNITER_STATE_ARMED,
    IGNITER_STATE_FIRED,
} igniter_state_t;

typedef enum {
    IGNITER_PIN_STATE_LOW = 0,
    IGNITER_PIN_STATE_HIGH,
} igniter_pin_state_t;

typedef enum {
    IGNITER_CONTINUITY_FAIL = 0,
    IGNITER_CONTINUITY_OK,
} igniter_continuity_t;

typedef enum {
    IGNITER_DRIVE_POSITIVE = 0,
    IGNITER_DRIVE_OPEN_DRAIN,
} igniter_drive_t;

typedef enum {
    IGNITER_OK = 0,
    IGNITER_FAIL,
    IGNITER_ADC_ERR,
    IGNITER_GPIO_ERR,
    IGNITER_INVALID_ARG,
    IGNITER_NULL_ARG,
} igniter_status_t;

typedef bool (*igniter_ADC_analog_read_raw)(uint8_t _adc_channel, uint16_t* _value);
typedef bool (*igniter_GPIO_set_level)(uint8_t _gpio_num, uint8_t _level);
typedef void (*igniter_delay)(uint32_t _ms);

typedef struct {
    igniter_ADC_analog_read_raw _adc_analog_read_raw;
    igniter_GPIO_set_level _gpio_set_level;
    uint8_t adc_channel_continuity;
    uint8_t gpio_num_arm;
    uint8_t gpio_num_fire;
    igniter_drive_t drive;
    igniter_state_t state;
} igniter_struct_t;

igniter_status_t igniter_check_continuity(igniter_struct_t* igniter, igniter_continuity_t* continuity);

igniter_status_t igniter_arm(igniter_struct_t* igniter);

igniter_status_t igniter_disarm(igniter_struct_t* igniter);

igniter_status_t igniter_fire(igniter_struct_t* igniter);

igniter_status_t igniter_reset(igniter_struct_t* igniter);


#endif /* PWRINSPACE_IGNITER_DRIVER_H_ */