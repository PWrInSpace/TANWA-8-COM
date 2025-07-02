///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 29.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains wrappers for miscelanous functions and RTOS functions.
///===-----------------------------------------------------------------------------------------===//
#ifndef PWRINSPACE_MCU_MISC_CONFIG_H_
#define PWRINSPACE_MCU_MISC_CONFIG_H_

#include <stdint.h>

void _led_delay_ms(uint32_t ms);

void _igniter_delay_ms(uint32_t ms);

void _lora_delay_ms(uint32_t ms);

void _buzzer_delay_ms(uint32_t ms);

void _abort_button_delay_ms(uint32_t ms);

uint32_t _get_uptime_ms(void);

void _lora_log(const char* info);

#endif /* PWRINSPACE_MCU_MISC_CONFIG_H_ */