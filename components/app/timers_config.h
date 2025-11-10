// Copyright 2022 PWrInSpace, Kuba
#ifndef TIMER_CFG_H
#define TIMER_CFG_H

#include "system_timer.h"
#include "stdbool.h"

#define TIMER_SD_DATA_PERIOD_MS 100
#define TIMER_DISCONNECT_PERIOD_MS 10 * 60 * 1000
#define ENGINE_BURN_TIME_MS 15000
#define IGNITION_OFF_TIMER 55

typedef enum {
    TIMER_SD_DATA = 0,
    TIMER_ABORT_BUTTON = 2,
    TIMER_IGNITION_OFF = 10,
} timers_id_def;

/**
 * @brief Initialize timers
 * 
 * @return true :D
 * @return false :C
 */
bool initialize_timers(void);

bool abort_button_timer_start_once(uint32_t period_ms);

bool sd_timer_change_period(uint32_t period_ms);

#endif