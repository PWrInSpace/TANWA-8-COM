///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Szymon Rzewuski
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of the system console configuration, including initialization
/// and available commands for debugging/testing purposes.
///===-----------------------------------------------------------------------------------------===//

#include "board_config.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "mcu_gpio_config.h"
#include "mcu_twai_config.h"
#include "mcu_adc_config.h"
#include "mcu_i2c_config.h"
#include "mcu_spi_config.h"
#include "can_config.h"
#include "console_config.h"
#include "tmp1075.h"
#include "mcp23018.h"
#include "led_state_display.h"
#include "ens_task.h"
#include "sd_task.h"
#include "state_machine_config.h"
#include "lora_task.h"
#include "board_data.h"
#include "system_timer.h"
#include "timers_config.h"

#define TAG "BOARD_CONFIG"

#define IOEXP_MODE (IOCON_INTCC | IOCON_INTPOL | IOCON_ODR | IOCON_MIRROR)

void _led_delay(uint32_t _ms) {
    vTaskDelay(_ms / portTICK_PERIOD_MS);
}

board_config_t config = {
    .board_name = "TANWA_BOARD", //CHANGE TO REAL BOARD NAME
    .status_led = {
        ._gpio_set_level = _mcu_gpio_set_level,
        ._delay = _led_delay,
        .gpio_num = CONFIG_GPIO_LED,
        .drive = LED_DRIVE_POSITIVE,
        .state = LED_STATE_OFF, 
    },
};

tanwa_hardware_dev_t tanwa_hardware = {
    .tmp1075 = {
        {
            ._i2c_write = _mcu_i2c_write,
            ._i2c_read = _mcu_i2c_read,
            .i2c_address = CONFIG_I2C_TMP1075_TS1_ADDR,
            .config_register = 0,
        },
        {
            ._i2c_write = _mcu_i2c_write,
            ._i2c_read = _mcu_i2c_read,
            .i2c_address = CONFIG_I2C_TMP1075_TS2_ADDR,
            .config_register = 0,
        },
    },
    .mcp23018 = {
        ._i2c_write = _mcu_i2c_write,
        ._i2c_read = _mcu_i2c_read,
        .i2c_address = CONFIG_I2C_MCP23018_ADDR,
        .iocon = 0,
        .dirRegisters = {0, 0},
        .polRegisters = {0, 0},
        .pullupRegisters = {0, 0},
        .ports = {0, 0},
    },
    .igniter = {
        {
            ._adc_analog_read_raw = _mcu_adc_read_raw,
            ._gpio_set_level = _mcu_gpio_set_level,
            .adc_channel_continuity = IGNITER_1_CHANNEL_INDEX,
            .gpio_num_arm = ARM1_GPIO_INDEX,
            .gpio_num_fire = FIRE_GPIO_INDEX,
            .drive = IGNITER_DRIVE_POSITIVE,
            .state = IGNITER_STATE_WAITING,
        },
        {
            ._adc_analog_read_raw = _mcu_adc_read_raw,
            ._gpio_set_level = _mcu_gpio_set_level,
            .adc_channel_continuity = IGNITER_2_CHANNEL_INDEX,
            .gpio_num_arm = ARM2_GPIO_INDEX,
            .gpio_num_fire = FIRE_GPIO_INDEX,
            .drive = IGNITER_DRIVE_POSITIVE,
            .state = IGNITER_STATE_WAITING,
        },
    },
};

led_state_display_struct_t led_state_display = {
    .mcp23018 = &tanwa_hardware.mcp23018,
    .state = LED_STATE_DISPLAY_STATE_NONE,
};

void _data_to_transmit(uint8_t *buffer, size_t buffer_size, size_t *tx_data_size){

    tanwa_data_t tanwa_data = tanwa_data_read();

    if (buffer == NULL || tx_data_size == NULL) {
        ESP_LOGE(TAG, "Buffer or tx_data_size is NULL");
        return;
    }

    memcpy(buffer, &tanwa_data, sizeof(tanwa_data_t));
    *tx_data_size = sizeof(tanwa_data_t);

}

ens_init_struct_t ens_init_struct = {
    ._data_to_transmit = _data_to_transmit, // Set to NULL for now, will be set later
    ._on_data_rx = NULL, // Set to NULL for now, will be set later
    .disable_sleep = false,
    .tx_nack_timeout_ms = 1000, // Default timeout for NACK
    .dev_mac_address = {0x80, 0x08, 0x50, 0x80, 0x08, 0x50}// Default MAC
};

uint16_t ens_periods[ENS_ENUM_MAX] = {
    [INIT_MS] = 200, // 1 second
    [IDLE_MS] = 200, // 1 second
    [ARMED_MS] = 150, // 1 second
    [FILLING_MS] = 150, // 1 second
    [ARMED_TO_LAUNCH_MS] = 150, // 1 second
    [RDY_TO_LAUNCH_MS] = 100, // 1 second
    [COUNTDOWN_MS] = 75, // 500 ms
    [FLIGHT_MS] = 75, // 1 second
    [FIRST_STAGE_MS] = 100, // 1 second
    [SECOND_STAGE_MS] = 150, // 1 second
    [ON_GROUND_MS] = 1000, // 1 second
    [HOLD_MS] = 500, // 1 second
    [ABORT_MS] = 200, // 1 second
    [SLEEP_MS] = 15000, // 5 seconds
};

esp_err_t board_config_init(void) {

    esp_err_t err;

    err = mcu_spi_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed");
    }
    
    err = mcu_gpio_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO initialization failed");
        return err;
    }

    err = mcu_twai_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI initialization failed");
        return err;
    }

    err = mcu_adc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC initialization failed");
        return err;
    }

    err = mcu_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return err;
    }

    err = can_config_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN initialization failed");
        return err;
    }

    uint8_t ret = 0;

    // ret = tmp1075_init(&(tanwa_hardware.tmp1075[0]));
    // if (ret != TMP1075_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize TMP1075 sensor 1");
    //     return ESP_FAIL;
    // } else {
    //     ESP_LOGI(TAG, "TMP1075 sensor 1 initialized");
    // }
    ret = tmp1075_init(&(tanwa_hardware.tmp1075[1]));
    if (ret != TMP1075_OK) {
        ESP_LOGE(TAG, "Failed to initialize TMP1075 sensor 2");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "TMP1075 sensor 2 initialized");
    }
    ret = mcp23018_init(&(tanwa_hardware.mcp23018), IOEXP_MODE);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP23018");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "MCP23018 initialized");
    }

    ret = led_state_display_state_update(&led_state_display, LED_STATE_DISPLAY_STATE_IDLE);
    if (ret != LED_STATE_DISPLAY_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED state display");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "LED state display initialized");
    }

    ESP_LOGI(TAG, "Initializing state machine...");

    if (!initialize_state_machine()) {
        ESP_LOGE(TAG, "State machine initialization failed");
    } else {
        ESP_LOGI(TAG, "### State machine initialization success ###");
        
    }

    if(ens_init(&ens_init_struct, ens_periods) != ENS_OK) {
        ESP_LOGE(TAG, "ENS initialization failed");
    } else {
        ESP_LOGI(TAG, "### ENS initialization success ###");
    }

    ESP_LOGI(TAG, "Initializing shared memory...");

    if (!tanwa_data_init()) {
        ESP_LOGE(TAG, "Shared memory initialization failed");
    } else {
        ESP_LOGI(TAG, "### Shared memory initialization success ###");
    }


    if (!initialize_timers()) {
        ESP_LOGE(TAG, "Timers initialization failed");
    } else {
        ESP_LOGI(TAG, "### Timers initialization success ###");
    }

    //SD CARD TIMER
    // if (!sys_timer_start(TIMER_SD_DATA, TIMER_SD_DATA_PERIOD_MS, TIMER_TYPE_PERIODIC)) {
    //     ESP_LOGE(TAG, "SD CARD | Timer start failed");
    // } else {
    //     ESP_LOGI(TAG, "SD CARD | Timer started");
    // }

    ESP_LOGI(TAG, "Initializing LoRa...");

    // if (!initialize_lora(LORA_TASK_FREQUENCY_KHZ, LORA_TASK_TRANSMIT_MS)) {
    //     ESP_LOGE(TAG, "LoRa initialization failed");
    // } else {
    //     ESP_LOGI(TAG, "### LoRa initialization success ###");
    // }

    state_machine_change_state(IDLE);
    //*********** ADD HARDWARE CONFIGURATION HERE ***********//

    err = console_config_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Console initialization failed");
        return err;
    }

    return ESP_OK;
    
}