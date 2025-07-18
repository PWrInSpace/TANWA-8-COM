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
    bool weights: 1;
    bool solenoid: 1;
    bool sensor : 1;
    bool power: 1;
    bool utility: 1;
} can_connected_slaves_t;

typedef struct {
    uint8_t i_sense;
    uint8_t temperature_1;
    uint8_t temperature_2;
} can_weight_status_t;

typedef struct {
    float ads1_weight1;
    float ads1_weight2;
    float ads1_weight3;
    float ads1_weight4;
    float ads2_weight1;
    float ads2_weight2;
    float ads2_weight3;
    float ads2_weight4;
    float rocket_weight;
    float tank_weight;
} can_weight_data_t;

typedef struct {
    uint8_t i_sense;
    uint8_t temperature1;
    uint8_t temperature2;
} can_solenoid_status_t;

typedef struct {
    bool state_sol1: 1;
    bool state_sol2: 1;
    bool state_sol3: 1;
    bool state_sol4: 1;
    bool state_sol5: 1;
    bool state_sol6: 1;
    bool servo_state1: 1;
    bool servo_state2: 1;
    bool servo_state3: 1;
    bool servo_state4: 1;
    uint8_t servo_angle1;
    uint8_t servo_angle2;
    uint8_t servo_angle3;
    uint8_t servo_angle4;
    uint8_t motor_state1;
    uint8_t motor_state2;
} can_solenoid_data_t;


typedef struct {
    uint8_t i_sense;
    uint8_t temperature1;
    uint8_t temperature2;
} can_sensor_status_t;

typedef struct {
    float temperature1;
    float temperature2;
    float temperature3;
    float temperature1_pt100;
    float temperature2_pt100;
} can_sensor_temp_data_t;

typedef struct {
    float pressure1;
    float pressure2;
    float pressure3;
    float pressure4;
    float pressure5;
    float pressure6;
    float pressure7;
    float pressure8;
} can_sensor_pressure_data_t;

typedef struct {
    uint8_t i_sense;
    uint8_t temperature1;
    uint8_t temperature2;
    bool switch_state1: 1;
    bool switch_state2: 1;
    bool switch_state3: 1;
    bool switch_state4: 1;
    bool switch_state5: 1;
    bool switch_state6: 1;
    bool switch_state7: 1;
    bool switch_state8: 1;
} can_utility_status_t;

typedef struct {
    uint8_t i_sense;
    uint8_t temperature1;
    uint8_t temperature2;
} can_power_status_t;

typedef struct {
    uint16_t voltage_12V;
    uint16_t voltage_24V;
    uint16_t current_12V;
    uint16_t current_24V;
} can_power_data_t;

#endif // PWRINSPACE_SLAVE_STRUCTS_H_