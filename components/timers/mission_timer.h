// Copyright 2022 PWrInSpace, Kuba
#ifndef MISSION_TIMER_H
#define MISSION_TIMER_H

#include <stdint.h>
#include <stdbool.h>


bool mission_timer_init(int64_t timer_disable_value);

/**
 * @brief Start mission timer
 *
 * @param countdown_begin_time countdown time, negativ value
 * @return true :D
 * @return false :C
 */
bool mission_timer_start(int64_t countdown_begin_time_ms);
int mission_timer_get_time();
bool mission_timer_is_enable();
bool mission_timer_stop();
bool mission_timer_set_disable_value(int64_t _disableValue);


#endif