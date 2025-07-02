///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 30.05.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the structures of data for the TANWA.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_TANWA_DATA_H_
#define PWRINSPACE_TANWA_DATA_H_

#include <stdbool.h>
#include <stdint.h>

#include "com_structs.h"
#include "slave_structs.h"


///===-----------------------------------------------------------------------------------------===//
/// TANWA data
///===-----------------------------------------------------------------------------------------===//

typedef struct {
    uint8_t state;
    // COM
    com_data_t com_data;
    can_connected_slaves_t can_connected_slaves;
    
    can_weight_status_t can_weight_status; 
    can_weight_data_t can_weight_data;
    
    can_solenoid_status_t can_solenoid_status;
    
    can_sensor_status_t can_sensor_status;
    can_sensor_temp_data_t can_sensor_data;
    can_sensor_pressure_data_t can_sensor_pressure_data;
    
    can_utility_status_t can_utility_status;

    can_power_status_t can_power_status;
} tanwa_data_t;

bool tanwa_data_init(void);
tanwa_data_t tanwa_data_read(void);
com_data_t tanwa_data_read_com_data(void);


void tanwa_data_update_com_data(com_data_t *data);

#endif // PWRINSPACE_TANWA_DATA_H_