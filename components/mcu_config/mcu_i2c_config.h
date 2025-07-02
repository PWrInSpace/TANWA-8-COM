///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 22.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains the configuration of the I2C peripheral for the MCU.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_MCU_I2C_CONFIG_H_
#define PWRINSPACE_MCU_I2C_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define MCU_I2C_DEFAULT_CONFIG()                                                      \
  {                                                                                   \
    .port = CONFIG_I2C_MASTER_PORT_NUM, .sda = CONFIG_I2C_SDA, .scl = CONFIG_I2C_SCL, \
    .clk_speed = CONFIG_I2C_MASTER_FREQUENCY, .i2c_init_flag = false,                 \
  }

typedef struct {
  i2c_port_t port;
  i2c_cmd_handle_t *cmd;
  gpio_num_t sda;
  gpio_num_t scl;
  uint32_t clk_speed;
  uint32_t timeout;
  bool i2c_init_flag;
} mcu_i2c_config_t;

/*!
 * \brief Initiates the I2C bus
 * \param i2c I2C configuration
 * \return ESP_OK on success, ESP_FAIL otherwise
 * \note This function will initiate I2C peripheral.
 */
esp_err_t mcu_i2c_init();

bool _mcu_i2c_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);

bool _mcu_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);

#endif // PWRINSPACE_MCU_I2C_CONFIG_H_