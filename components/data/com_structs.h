///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 30.05.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the structures of data for COM board of TAMWA.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_TANWA_COM_STRUCTS_H_
#define PWRINSPACE_TANWA_COM_STRUCTS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float vbat;
    bool abort_button;
    bool arm_state;
    bool solenoid_state_fill: 1;
    bool solenoid_state_depr: 1;
    float pressure_1;
    float pressure_2;
    float pressure_3;
    float pressure_4;
    float temperature_1;
    float temperature_2;
    bool igniter_cont_1;
    bool igniter_cont_2;
} com_data_t;

#endif // PWRINSPACE_TANWA_COM_STRUCTS_H_