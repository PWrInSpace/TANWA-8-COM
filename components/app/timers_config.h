// Copyright 2022 PWrInSpace, Kuba
#ifndef TIMER_CFG_H
#define TIMER_CFG_H

#include "system_timer.h"
#include "stdbool.h"

#define TIMER_SD_DATA_PERIOD_MS 100
#define TIMER_DISCONNECT_PERIOD_MS 10 * 60 * 1000
#define ENGINE_BURN_TIME_MS 2500
#define IGNITION_OFF_TIMER 55

typedef enum {
    TIMER_SD_DATA = 0,
    TIMER_BUZZER = 1,
    TIMER_ABORT_BUTTON = 2,
    TIMER_DISCONNECT = 3,
    TIMER_IGNITION = 4,
    TIMER_BURN = 5,
    TIMER_AFTER_BURNOUT = 6,
    TIMER_FUEL_INITIAL = 7,
    TIMER_OXIDIZER_FULL = 8,
    TIMER_FUEL_FULL = 9,
    TIMER_IGNITION_OFF = 10,
} timers_id_def;

/**
 * @brief Initialize timers
 * 
 * @return true :D
 * @return false :C
 */
bool initialize_timers(void);

bool buzzer_timer_start(uint32_t period_ms);

bool buzzer_timer_change_period(uint32_t period_ms);

bool abort_button_timer_start_once(uint32_t period_ms);

bool sd_timer_change_period(uint32_t period_ms);

#endif