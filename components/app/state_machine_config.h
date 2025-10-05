///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 28.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
#ifndef PWRINSPACE_STATE_MACHINE_CONFIG_H_
#define PWRINSPACE_STATE_MACHINE_CONFIG_H_

#include "state_machine.h"

#include "stdbool.h"

typedef enum {
    INIT = 0,
    IDLE,
    RECOVERY_ARM,
    FUELING,
    PRESSURIZING,
    ARMED_TO_LAUNCH,
    RDY_TO_LAUNCH,
    COUNTDOWN,
    LIFT_OFF,
    BURN,
    FLIGHT,
    FIRST_STAGE_RECOVERY,
    SECOND_STAGE_RECOVERY,
    ON_GROUND,
    HOLD,
    ABORT,
} state_t;

/**
 * @brief Initialize state machine task
 * 
 * @return true :D
 * @return false :C
 */
bool initialize_state_machine(void);

/**
 * @brief Provide state text from number
 * @param state state number
 * @param text pointer to char array
 */
void get_state_text(int32_t state, char *text);

#endif /* PWRINSPACE_STATE_MACHINE_CONFIG_H_*/