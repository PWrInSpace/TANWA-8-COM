///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 13.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of TMP1075 class which is a driver for TMP1075 temperature sensor.
/// The alert functionalisty is not implemented.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_TMP1075_H_
#define PWRINSPACE_TMP1075_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* TMP1075 REGISTERS */
#define TMP1075_REG_TEMP 0x00
#define TMP1075_REG_CONFIG 0x01

typedef enum {
    TMP1075_FAULT_1 = 0b00,
    TMP1075_FAULT_2 = 0b01,
    TMP1075_FAULT_3 = 0b10,
    TMP1075_FAULT_4 = 0b11,
} tmp1075_consecutive_faults_t;

typedef enum {
    TMP1075_CONV_TIME_27_5MS = 0b00,
    TMP1075_CONV_TIME_55MS = 0b01,
    TMP1075_CONV_TIME_110MS = 0b10,
    TMP1075_CONV_TIME_220MS = 0b11,
} tmp1075_conversion_time_t;

typedef enum {
    TMP1075_OFFSET_OS = 7,
    TMP1075_OFFSET_R  = 5,
    TMP1075_OFFSET_F  = 3,
    TMP1075_OFFSET_POL = 2,
    TMP1075_OFFSET_TM = 1,
    TMP1075_OFFSET_SD = 0,
} tmp1075_offsets_t;

typedef enum {
    TMP1075_OK = 0,
    TMP1075_FAIL,
    TMP1075_INIT_ERR,
    TMP1075_READ_ERR,
    TMP1075_WRITE_ERR,
} tmp1075_status_t;

// I2C access functions
typedef bool (*tmp1075_I2C_write)(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);
typedef bool (*tmp1075_I2C_read)(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);

typedef struct {
    tmp1075_I2C_write _i2c_write;
    tmp1075_I2C_read _i2c_read;
    uint8_t i2c_address;
    uint8_t config_register;
} tmp1075_struct_t;

tmp1075_status_t tmp1075_init(tmp1075_struct_t *tmp1075);

tmp1075_status_t tmp1075_get_temp_raw(tmp1075_struct_t *tmp1075, int16_t *raw);

tmp1075_status_t tmp1075_get_temp_celsius(tmp1075_struct_t *tmp1075, float *temp);

tmp1075_status_t tmp1075_start_conv(tmp1075_struct_t *tmp1075);

tmp1075_status_t tmp1075_get_conv_mode(tmp1075_struct_t *tmp1075, bool *is_single_shot);

tmp1075_status_t tmp1075_set_conv_mode(tmp1075_struct_t *tmp1075, const bool is_single_shot);

tmp1075_status_t tmp1075_get_conv_time(tmp1075_struct_t *tmp1075, tmp1075_conversion_time_t *conv_time);

tmp1075_status_t tmp1075_set_conv_time(tmp1075_struct_t *tmp1075, const tmp1075_conversion_time_t conv_time);

#endif /* PWRINSPACE_TMP1075_H_ */
