///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 14.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the general purpouse state machine with callbacks for sounding
/// rockets and its configuration. States are defined by user.
///===-----------------------------------------------------------------------------------------===//
#ifndef PWRINSPACE_STATE_MACHINE_H_
#define PWRINSPACE_STATE_MACHINE_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    STATE_MACHINE_OK,
    STATE_MACHINE_STATES_ERROR,
    STATE_MACHINE_STATES_NUMBER_ERROR,
    STATE_MACHINE_STATE_CHANGE_ERROR,
    STATE_MACHINE_NO_MORE_STATES,
    STATE_MACHINE_INVALID_STATE,
    STATE_MACHINE_RUN_ERROR,
    STATE_MACHINE_NULL_FUNCTION,
} state_machine_status_t;

typedef void (*state_change_callback)(void *arg);
typedef void (*end_looped_function)(void);
typedef uint8_t state_id;

typedef struct {
    state_id id;
    state_change_callback callback;
    void *arg;
} state_config_t;

typedef struct {
    uint32_t stack_depth;
    BaseType_t core_id;
    UBaseType_t priority;
} state_machine_task_cfg_t;

void state_machine_init(void);
state_machine_status_t state_machine_set_states(state_config_t *states, uint8_t states_number);
state_machine_status_t state_machine_set_end_function(end_looped_function fct, uint32_t freq_ms);
state_id state_machine_get_current_state(void);
state_id state_machine_get_previous_state(void);
state_machine_status_t state_machine_change_state(state_id new_state);
state_machine_status_t state_machine_force_change_state(state_id new_state);
state_machine_status_t state_machine_change_to_previous_state(bool run_callback);
state_machine_status_t state_machine_change_state_ISR(state_id new_state);
state_machine_status_t state_machine_run(state_machine_task_cfg_t *cfg);
state_machine_status_t state_machine_destroy(void);

#endif /* PWRINSPACE_STATE_MACHINE_H_ */