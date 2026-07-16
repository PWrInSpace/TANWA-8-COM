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
#include "relay_driver.h"
#include "cmd_commands.h"
#include "ens_struct.h"

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
    .relay = {
        {
            .gpio_pin = CONFIG_RELAY_1_GPIO,
            .state = RELAY_OFF,
        },
        {
            .gpio_pin = CONFIG_RELAY_2_GPIO,
            .state = RELAY_OFF,
        },
        {
            .gpio_pin = CONFIG_RELAY_3_GPIO,
            .state = RELAY_OFF,
        },
        {
            .gpio_pin = CONFIG_RELAY_4_GPIO,
            .state = RELAY_OFF,
        },
    },
};

led_state_display_struct_t led_state_display = {
    .mcp23018 = &tanwa_hardware.mcp23018,
    .state = LED_STATE_DISPLAY_STATE_NONE,
};

void _data_to_transmit(uint8_t *buffer, size_t buffer_size, size_t *tx_data_size){

    tanwa_data_t tanwa_data = tanwa_data_read();

    data_to_obc_t data_to_obc;

    data_to_obc.tanWaState = tanwa_data.state;
    data_to_obc.vbat = tanwa_data.can_power_data.VOLTAGE_24V_SYS;
    data_to_obc.thrust_val = tanwa_data.can_weight_data.ads1_weight2;
    data_to_obc.tankWeight_val = tanwa_data.can_weight_data.ads1_weight3;
    data_to_obc.temperature_postFill = tanwa_data.can_sensor_temp_data.temperature1;
    data_to_obc.temperature_Wall = tanwa_data.can_sensor_temp_data.temperature2;
    data_to_obc.postFillN2_pres = tanwa_data.can_sensor_pressure_data.pressure1;
    data_to_obc.droidN2_press = tanwa_data.can_sensor_pressure_data.pressure2;
    data_to_obc.droidN2O_press = tanwa_data.can_sensor_pressure_data.pressure3;
    data_to_obc.combChamber_pres = tanwa_data.can_sensor_pressure_data.pressure4;
    data_to_obc.cutoffN2O_pres = tanwa_data.can_sensor_pressure_data.pressure5;
    data_to_obc.postRegulatorN2_pres = tanwa_data.can_sensor_pressure_data.pressure6;
    data_to_obc.preRegulatorN2_pres = tanwa_data.can_sensor_pressure_data.pressure7;
    data_to_obc.postFillN2O_pres = tanwa_data.can_sensor_pressure_data.pressure8;
        data_to_obc.soft_arm = tanwa_data.com_data.arm_state;
    data_to_obc.canWeighta_con = tanwa_data.can_connected_slaves.weights;
    data_to_obc.canSensor_con = tanwa_data.can_connected_slaves.sensor;
    data_to_obc.canSolenoid_con = tanwa_data.can_connected_slaves.solenoid;
    data_to_obc.canUtility_con = tanwa_data.can_connected_slaves.utility;
    data_to_obc.canPower_con = tanwa_data.can_connected_slaves.power;
    data_to_obc.igniterContinouity_1 = tanwa_data.com_data.igniter_cont_1;
    data_to_obc.igniterContinouity_2 = tanwa_data.com_data.igniter_cont_2;
    data_to_obc.fillN2OState = tanwa_data.can_solenoid_data.state_sol1;
    data_to_obc.deprN2OState = tanwa_data.can_solenoid_data.state_sol2;
    data_to_obc.fillN2State = tanwa_data.can_solenoid_data.state_sol3;
    data_to_obc.deprN2State = tanwa_data.can_solenoid_data.state_sol4;
    data_to_obc.droidN2OState = tanwa_data.can_solenoid_data.state_sol5;
    data_to_obc.droidN2State = tanwa_data.can_solenoid_data.state_sol6;
    relay_driver_state_t vent_state;
    relay_get_state(&(tanwa_hardware.relay[0]), &vent_state);
    data_to_obc.heatingTankState = vent_state == RELAY_OFF; //VENT
    data_to_obc.heatingValveState = tanwa_data.com_data.relay_state2;
    data_to_obc.abortButton = tanwa_data.com_data.abort_button;
    data_to_obc.TANWA_24V_SYS_VOLTAGE = tanwa_data.can_power_data.VOLTAGE_24V_SYS;
    data_to_obc.TANWA_24V_SYS_CURRENT = tanwa_data.can_power_data.current_24V_SYS;
    data_to_obc.TANWA_24V_SOL_VOLTAGE = tanwa_data.can_power_data.VOLTAGE_24V_SOL;
    data_to_obc.TANWA_24V_SOL_CURRENT = tanwa_data.can_power_data.current_24V_SOL;

    

    if (buffer == NULL || tx_data_size == NULL) {
        ESP_LOGE(TAG, "Buffer or tx_data_size is NULL");
        return;
    }

    if(buffer_size < sizeof(data_to_obc_t)) {
        ESP_LOGE(TAG, "Buffer size is too small. Required: %d, Provided: %d", sizeof(data_to_obc_t), buffer_size);
        return;
    }

    memcpy(buffer, &data_to_obc, sizeof(data_to_obc_t));
    *tx_data_size = sizeof(data_to_obc_t);
} 

// ens_init_struct_t ens_init_struct = {
//     ._data_to_transmit = _data_to_transmit, // Set to NULL for now, will be set later
//     ._on_data_rx = ens_command_parsing, // Set to NULL for now, will be set later
//     .disable_sleep = false,
//     .tx_nack_timeout_ms = 1000, // Default timeout for NACK
//     .dev_mac_address = {0x80, 0x08, 0x50, 0x80, 0x08, 0x50}// Default MAC
// };

// uint16_t ens_periods[ENS_ENUM_MAX] = {
//     [INIT_MS] = 1000, // 1 second
//     [IDLE_MS] = 500, // 1 second
//     [ARMED_MS] = 300, // 1 second
//     [FILLING_MS] = 300, // 1 second
//     [PRESSURIZING_MS] = 300, // 1 second
//     [ARMED_TO_LAUNCH_MS] = 200, // 1 second
//     [RDY_TO_LAUNCH_MS] = 200, // 1 second
//     [COUNTDOWN_MS] = 100, // 500 ms
//     [LIFT_OFF_MS] = 50, // 1 second
//     [BURN_MS] = 200, // 1 second
//     [FLIGHT_MS] = 500, // 1 second
//     [FIRST_STAGE_MS] = 500, // 1 second
//     [SECOND_STAGE_MS] = 500, // 1 second
//     [ON_GROUND_MS] = 1000, // 1 second
//     [HOLD_MS] = 500, // 1 second
//     [ABORT_MS] = 500, // 1 second
//     [SLEEP_MS] = 15000, // 15 seconds
// };

uint16_t lora_periods[] = {
    [INIT] = 1000,
    [IDLE] = 500,
    [RECOVERY_ARM] = 300,
    [FILLING] = 300,
    [PRESSURIZING] = 300,
    [ARMED_TO_LAUNCH] = 200,
    [RDY_TO_LAUNCH] = 200,
    [COUNTDOWN] = 100,
    [LIFT_OFF] = 100,
    [BURN] = 200,
    [FLIGHT] = 500,
    [FIRST_STAGE_RECOVERY] = 500,
    [SECOND_STAGE_RECOVERY] = 500,
    [ON_GROUND] = 1000,
    [HOLD] = 500,
    [ABORT] = 500,
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

    //vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for LoRa to initialize

    err = mcu_twai_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI initialization failed");
        return err;
    }

    err = can_config_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN initialization failed");
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

    if(relay_driver_init(_relay_gpio_set_level) != RELAY_DRIVER_OK) {
        ESP_LOGE(TAG, "Relay driver initialization failed");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Relay driver initialized");
    }

    relay_open(&(tanwa_hardware.relay[0])); //VENT

    uint8_t ret = 0;

    // ret = tmp1075_init(&(tanwa_hardware.tmp1075[0]));
    // if (ret != TMP1075_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize TMP1075 sensor 1");
    //     return ESP_FAIL;
    // } else {
    //     ESP_LOGI(TAG, "TMP1075 sensor 1 initialized");
    // }
    // ret = tmp1075_init(&(tanwa_hardware.tmp1075[1]));
    // if (ret != TMP1075_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize TMP1075 sensor 2");
    //     return ESP_FAIL;
    // } else {
    //     ESP_LOGI(TAG, "TMP1075 sensor 2 initialized");
    // }
    // ret = mcp23018_init(&(tanwa_hardware.mcp23018), IOEXP_MODE);
    // if (ret != MCP23018_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize MCP23018");
    //     return ESP_FAIL;
    // } else {
    //     ESP_LOGI(TAG, "MCP23018 initialized");
    // }

    // ret = led_state_display_state_update(&led_state_display, LED_STATE_DISPLAY_STATE_IDLE);
    // if (ret != LED_STATE_DISPLAY_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize LED state display");
    //     return ESP_FAIL;
    // } else {
    //     ESP_LOGI(TAG, "LED state display initialized");
    // }

    ESP_LOGI(TAG, "Initializing state machine...");

    if (!initialize_state_machine()) {
        ESP_LOGE(TAG, "State machine initialization failed");
    } else {
        ESP_LOGI(TAG, "### State machine initialization success ###");
        
    }

    // if(ens_init(&ens_init_struct, ens_periods) != ENS_OK) {
    //     ESP_LOGE(TAG, "ENS initialization failed");
    // } else {
    //     ESP_LOGI(TAG, "### ENS initialization success ###");
    // }

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


     ESP_LOGI(TAG, "Initializing LoRa...");

     if (!initialize_lora(LORA_TASK_FREQUENCY_KHZ, LORA_TASK_TRANSMIT_MS)) {
        ESP_LOGE(TAG, "LoRa initialization failed");
    } else {
        ESP_LOGI(TAG, "### LoRa initialization success ###");
        lora_set_state_periods(lora_periods, sizeof(lora_periods) / sizeof(lora_periods[0]));
    }

    gpio_set_level(LORA_RS_GPIO, 1);
    

    //HAS TO BE AFTER LORA- DOESN'T WORK OTHERWISE

    ESP_LOGI(TAG, "Initializing SD Card...");

    if (!init_sd_card()) {
        ESP_LOGE(TAG, "SD Card initialization failed");
    } else {
        ESP_LOGI(TAG, "### SD Card initialization success ###");
    }

    //SD CARD TIMER
    if (!sys_timer_start(TIMER_SD_DATA, TIMER_SD_DATA_PERIOD_MS, TIMER_TYPE_PERIODIC)) {
        ESP_LOGE(TAG, "SD CARD | Timer start failed");
    } else {
        ESP_LOGI(TAG, "SD CARD | Timer started");
    }

    state_machine_change_state(IDLE);    

    err = console_config_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Console initialization failed");
        return err;
    }
    
    return ESP_OK;
    
}

esp_err_t tanwa_read_i_sense(float *i_sense) {
    if (i_sense == NULL) {
        ESP_LOGE(TAG, "i_sense pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!_mcu_adc_read_voltage(ISENSE_CHANNEL_INDEX, i_sense)) {
        ESP_LOGE(TAG, "Failed to read i_sense");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void buzzer_change_period(uint16_t period_ms){
    uint8_t data[8] = {0};
    data[0] = 1;

    if(period_ms == 0){
        return;
    }

    memcpy(&data[1], &period_ms, sizeof(uint16_t));

    can_send_message(CAN_UTIL_CHANGE_BUZZER_STATE_ID, data, 3);
}

void buzzer_stop(void){
    uint8_t data[8] = {0};
    data[0] = 0;
    can_send_message(CAN_UTIL_CHANGE_BUZZER_STATE_ID, data, 3);
}

void buzzer_toggle(bool state, uint8_t freq_s){
    uint8_t data[8] = {0};
    data[0] = state;
    data[1] = freq_s;
    ESP_LOGI(TAG, "Toggling buzzer -> state: %d, freq_s: %d", state, freq_s);
    can_send_message(CAN_UTIL_CHANGE_BUZZER_STATE_ID, data, 8);
}