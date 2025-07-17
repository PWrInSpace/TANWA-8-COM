#ifndef MISSION_TIMER_CONFIG_H
#define MISSION_TIMER_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initalize liquid engine ignition test timer
 * 
 * @return true :D
 * @return false :C
 */
bool liquid_ignition_test_timer_init();

/**
 * @brief Start liquid engine ignition test timer
 * 
 * @param countdown_time coutndown time, negative value -> T:-x
 * @param ignition_time ignitonion time, negative value -> T:-x
 * @return true :D
 * @return false :C
 */
bool liquid_ignition_test_timer_start(int32_t countdown_time, int32_t ignition_time);

/**
 * @brief Stop mission timer
 * 
 * @return true :D
 * @return false :C
 */
bool liquid_ignition_test_timer_stop();

/**
 * @brief Get liquid engine igniton test time
 *
 * @return int time in microseconds
 */
int liquid_ignition_test_timer_get_time();


void liquid_ignition_test_timer_set_disable_val(int64_t time);

#endif