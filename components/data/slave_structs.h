///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 30.05.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the structures of data for the slave TANWA submodules.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_TANWA_SLAVE_STRUCTS_H_
#define PWRINSPACE_TANWA_SLAVE_STRUCTS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool hx_rocket: 1;
    bool hx_oxidizer: 1;
    bool fac: 1;
    bool flc: 1;
    bool termo: 1;
} can_connected_slaves_t;

typedef struct {
    bool state;
} can_weight_status_t;

typedef struct {
    float weight;
} can_weight_data_t;

typedef struct {
    bool state;
} can_solenoid_status_t;

typedef struct {
    bool state;
} can_sensor_status_t;

typedef struct {
    float temperature;
} can_sensor_temp_data_t;

typedef struct {
    float pressure;
} can_sensor_pressure_data_t;

typedef struct {
    bool state;
} can_utility_status_t;

typedef struct {
    bool state;
} can_power_status_t;

#endif // PWRINSPACE_SLAVE_STRUCTS_H_