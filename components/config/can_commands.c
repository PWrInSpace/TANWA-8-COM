#include <string.h>

#include "esp_log.h"

#include "can_commands.h"

#include "board_data.h"
#include "slave_structs.h"
#include "can_api.h"
#include "board_config.h"
#include "relay_driver.h"

#define TAG "CAN_COMMANDS"

esp_err_t switch_update(bool *switch_states, bool *prev_switch_states) {
    if (switch_states == NULL || prev_switch_states == NULL) {
        ESP_LOGE(TAG, "Switch states pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < 6; i++) {
        if (switch_states[i] != prev_switch_states[i]) { // tylko jeśli nastąpiła zmiana
            uint8_t data[8] = {i, 0, 0, 0, 0, 0, 0, 0};
            if (switch_states[i]) {
                can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
            } else {
                can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
            }// aktualizuj poprzedni stan
        }
    }

    // Obsługa przekaźnika tylko jeśli nastąpiła zmiana
    if (switch_states[6] != prev_switch_states[6]) {
        relay_driver_err_t err;
        if (switch_states[6]) {
            err = relay_open(&(tanwa_hardware.relay[0]));
            if (err != RELAY_DRIVER_OK) {
                ESP_LOGE(TAG, "Relay open error: %d", (uint8_t) err); 
            }
        } else {
            err = relay_close(&(tanwa_hardware.relay[0]));
            if (err != RELAY_DRIVER_OK) {
                ESP_LOGE(TAG, "Relay close error: %d", (uint8_t) err);
            }
        } // aktualizuj poprzedni stan
    }

    return ESP_OK;
}

esp_err_t example_command_handler(uint8_t *data, uint8_t length) {
    
    // Process the data received in the CAN message
    // For example, print the first byte of data
    ESP_LOGI("CAN_COMMANDS", "Example command handler");
    
    return ESP_OK;
}

esp_err_t parse_solenoid_status(uint8_t *data, uint8_t length) {

    if (length < 3) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_solenoid_status_t status;
    status.i_sense = data[2];
    status.temperature1 = data[0];
    status.temperature2 = data[1];

    tanwa_data_update_can_solenoid_status(&status);

    return ESP_OK;
}

esp_err_t parse_uint8_t_to_bool(uint8_t value, uint8_t num_states, bool *states)
{
    if (states == NULL || num_states > 8) {
        ESP_LOGE(TAG, "Invalid output or too many states for uint8_t");
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < num_states; i++) {
        states[i] = (value & (1 << i)) ? true : false;
    }
    return ESP_OK;
}

esp_err_t parse_solenoid_data(uint8_t *data, uint8_t length) {

    can_solenoid_data_t sol_data = tanwa_data_read_can_solenoid_data();

    if (length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    bool solenoid_states[6];
    bool servo_states[4];

    // Dekoduj stany elektrozaworów
    esp_err_t ret = parse_uint8_t_to_bool(data[0], 6, solenoid_states);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode solenoid states");
        return ret;
    }

    // Dekoduj stany serw
    ret = parse_uint8_t_to_bool(data[1], 4, servo_states);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode servo states");
        return ret;
    }

    sol_data.state_sol1 = solenoid_states[0];
    sol_data.state_sol2 = solenoid_states[1];
    sol_data.state_sol3 = solenoid_states[2];
    sol_data.state_sol4 = solenoid_states[3];
    sol_data.state_sol5 = solenoid_states[4];
    sol_data.state_sol6 = solenoid_states[5];
    sol_data.servo_state1 = servo_states[0];
    sol_data.servo_state2 = servo_states[1];
    sol_data.servo_state3 = servo_states[2];
    sol_data.servo_state4 = servo_states[3];
    sol_data.servo_angle1 = data[2];
    sol_data.servo_angle2 = data[3];
    sol_data.servo_angle3 = data[4];
    sol_data.servo_angle4 = data[5];
    sol_data.motor_state1 = data[6];
    sol_data.motor_state2 = data[7];

    tanwa_data_update_can_solenoid_data(&sol_data);

    return ESP_OK;
}

esp_err_t parse_power_status(uint8_t *data, uint8_t length) {
    
    if (length < 3) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }
    can_power_status_t status;
    status.i_sense = data[2];
    status.temperature1 = data[0];
    status.temperature2 = data[1];
    tanwa_data_update_can_power_status(&status);
    
    return ESP_OK;
}

esp_err_t parse_power_data(uint8_t *data, uint8_t length) {
    if (length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_power_data_t power_data;
    int16_t raw_voltage_24V, raw_current_24V, raw_voltage_12V, raw_current_12V;

    memcpy(&raw_voltage_24V, &data[0], sizeof(int16_t));
    memcpy(&raw_current_24V, &data[2], sizeof(int16_t));
    memcpy(&raw_voltage_12V, &data[4], sizeof(int16_t));
    memcpy(&raw_current_12V, &data[6], sizeof(int16_t));

    power_data.volatage_24V = ((float)raw_voltage_24V) / 100.0f;
    power_data.current_24V  = ((float)raw_current_24V)  / 100.0f;
    power_data.voltage_12V  = ((float)raw_voltage_12V)  / 100.0f;
    power_data.current_12V  = ((float)raw_current_12V)  / 100.0f;

    tanwa_data_update_can_power_data(&power_data);

    return ESP_OK;
}

esp_err_t parse_power_channel(uint8_t *data, uint8_t length) {

    ESP_LOGI(TAG, "POWER CHANNEL: %d", data[0]);
    
    return ESP_OK;
}

esp_err_t parse_sensor_status(uint8_t *data, uint8_t length) {
    
    if (length < 3) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_sensor_status_t status;
    status.i_sense = data[2];
    status.temperature1 = data[0];
    status.temperature2 = data[1];
    tanwa_data_update_can_sensor_status(&status);
    
    return ESP_OK;
}

esp_err_t parse_sensor_data(uint8_t *data, uint8_t length) {
    
    for(int i = 0; i < length; i++) {
        ESP_LOGI("CAN_COMMANDS", "Sensor data byte %d: %02X", i, data[i]);
    }
    
    return ESP_OK;
}

esp_err_t parse_sensor_pressure1(uint8_t *data, uint8_t length) {
    if (length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_sensor_pressure_data_t pressure_data = tanwa_data_read_can_sensor_pressure_data();

    // Odczytaj int16_t z dwóch bajtów (little-endian) i przelicz na float
    int16_t raw_pressure1, raw_pressure2, raw_pressure3, raw_pressure4;
    memcpy(&raw_pressure1, &data[0], sizeof(int16_t));
    memcpy(&raw_pressure2, &data[2], sizeof(int16_t));
    memcpy(&raw_pressure3, &data[4], sizeof(int16_t));
    memcpy(&raw_pressure4, &data[6], sizeof(int16_t));

    pressure_data.pressure1 = ((float)raw_pressure1) / 100.0f;
    pressure_data.pressure2 = ((float)raw_pressure2) / 100.0f;
    pressure_data.pressure3 = ((float)raw_pressure3) / 100.0f;
    pressure_data.pressure4 = ((float)raw_pressure4) / 100.0f;

    tanwa_data_update_can_sensor_pressure_data(&pressure_data);

    return ESP_OK;
}

esp_err_t parse_sensor_pressure2(uint8_t *data, uint8_t length) {
    if (length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_sensor_pressure_data_t pressure_data = tanwa_data_read_can_sensor_pressure_data();

    int16_t raw_pressure5, raw_pressure6, raw_pressure7, raw_pressure8;
    memcpy(&raw_pressure5, &data[0], sizeof(int16_t));
    memcpy(&raw_pressure6, &data[2], sizeof(int16_t));
    memcpy(&raw_pressure7, &data[4], sizeof(int16_t));
    memcpy(&raw_pressure8, &data[6], sizeof(int16_t));

    pressure_data.pressure5 = ((float)raw_pressure5) / 100.0f;
    pressure_data.pressure6 = ((float)raw_pressure6) / 100.0f;
    pressure_data.pressure7 = ((float)raw_pressure7) / 100.0f;
    pressure_data.pressure8 = ((float)raw_pressure8) / 100.0f;

    tanwa_data_update_can_sensor_pressure_data(&pressure_data);

    return ESP_OK;
}

esp_err_t parse_sensor_temperature(uint8_t *data, uint8_t length) {
    if (length < 6) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_sensor_temp_data_t temp_data;
    // Odczytaj int16_t z dwóch bajtów (little-endian)
    memcpy(&temp_data.temperature1, &data[0], sizeof(int16_t));
    memcpy(&temp_data.temperature2, &data[2], sizeof(int16_t));
    memcpy(&temp_data.temperature3, &data[4], sizeof(int16_t));

    // Możesz teraz używać temp_data.temperature1/2/3 jako int16_t
    // tanwa_data_update_can_sensor_temp_data(&temp_data); // jeśli masz taką funkcję

    return ESP_OK;
}

esp_err_t parse_sensor_pressure_info(uint8_t *data, uint8_t length) {
    
    ESP_LOGI(TAG, "ADS NUMBER: %d", data[0]);
    ESP_LOGI(TAG, "ADS GAIN: %d", data[1]);
    ESP_LOGI(TAG, "ADS DATA RATE: %d", data[2]);

    return ESP_OK;
}

esp_err_t parse_util_status(uint8_t *data, uint8_t length) {
    
    if (length < 3) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }
    can_utility_status_t status = tanwa_data_read_can_utility_status();
    status.i_sense = data[2];
    status.temperature1 = data[0];
    status.temperature2 = data[1];
    bool switch_states[8];
    esp_err_t ret = parse_uint8_t_to_bool(data[3], 8, switch_states);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode switch states");
        return ret;
    }

    bool prev_switch_states[8] = {
        status.switch_state1,
        status.switch_state2,
        status.switch_state3,
        status.switch_state4,
        status.switch_state5,
        status.switch_state6,
        status.switch_state7,
        status.switch_state8
    };

    switch_update(switch_states, prev_switch_states);

    status.switch_state1 = switch_states[0];
    status.switch_state2 = switch_states[1];
    status.switch_state3 = switch_states[2];
    status.switch_state4 = switch_states[3];
    status.switch_state5 = switch_states[4];
    status.switch_state6 = switch_states[5];
    status.switch_state7 = switch_states[6];
    status.switch_state8 = switch_states[7];

    tanwa_data_update_can_utility_status(&status);
    
    return ESP_OK;
}

esp_err_t parse_util_data(uint8_t *data, uint8_t length) {
    
    for(int i = 0; i < length; i++) {
        ESP_LOGI("CAN_COMMANDS", "Utility data byte %d: %02X", i, data[i]);
    }
    
    return ESP_OK;
}

esp_err_t parse_weights_status(uint8_t *data, uint8_t length) {
    
    if (length < 3) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_weight_status_t status;
    status.i_sense = data[2];
    status.temperature_1 = data[0];
    status.temperature_2 = data[1];
    tanwa_data_update_can_weight_status(&status);
    
    return ESP_OK;
}

esp_err_t parse_weights_board_data(uint8_t *data, uint8_t length) {
    
    for(int i = 0; i < length; i++) {
        ESP_LOGI("CAN_COMMANDS", "Weights board data byte %d: %02X", i, data[i]);
    }
    
    return ESP_OK;
}

esp_err_t parse_weights_ads1_all_ch_weight1(uint8_t *data, uint8_t length) {
    
    if( length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_weight_data_t weight_data = tanwa_data_read_can_weight_data();

    memcpy(&weight_data.ads1_weight1, &data[0], sizeof(float));
    memcpy(&weight_data.ads1_weight2, &data[4], sizeof(float));

    tanwa_data_update_can_weight_data(&weight_data);

    
    return ESP_OK;
}
esp_err_t parse_weights_ads1_all_ch_weight2(uint8_t *data, uint8_t length) {

    if( length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_weight_data_t weight_data = tanwa_data_read_can_weight_data();
    memcpy(&weight_data.ads1_weight3, &data[0], sizeof(float));
    memcpy(&weight_data.ads1_weight4, &data[4], sizeof(float));

    tanwa_data_update_can_weight_data(&weight_data);
    
    return ESP_OK;
}

esp_err_t parse_weights_ads2_all_ch_weight1(uint8_t *data, uint8_t length) {
    
    if( length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_weight_data_t weight_data = tanwa_data_read_can_weight_data();
    memcpy(&weight_data.ads2_weight1, &data[0], sizeof(float));
    memcpy(&weight_data.ads2_weight2, &data[4], sizeof(float));
    tanwa_data_update_can_weight_data(&weight_data);
    
    return ESP_OK;
}

esp_err_t parse_weights_ads2_all_ch_weight2(uint8_t *data, uint8_t length) {
    
    if( length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }
    can_weight_data_t weight_data = tanwa_data_read_can_weight_data();
    memcpy(&weight_data.ads2_weight3, &data[0], sizeof(float));
    memcpy(&weight_data.ads2_weight4, &data[4], sizeof(float));
    tanwa_data_update_can_weight_data(&weight_data);
    
    return ESP_OK;
}

esp_err_t parse_weights_ads_ch_weight(uint8_t *data, uint8_t length) {
    
    for(int i = 0; i < length; i++) {
        ESP_LOGI("CAN_COMMANDS", "Weights ADS channel weight byte %d: %02X", i, data[i]);
    }
    
    return ESP_OK;
}

esp_err_t parse_weights(uint8_t *data, uint8_t length) {
    
    if (length < 8) {
        ESP_LOGE(TAG, "Frame too short");
        return ESP_ERR_INVALID_ARG;
    }

    can_weight_data_t weights_data = tanwa_data_read_can_weight_data();

    // Odczytaj float z czterech bajtów (little-endian)
    memcpy(&weights_data.rocket_weight, &data[4], sizeof(float));
    memcpy(&weights_data.tank_weight, &data[0], sizeof(float));

    tanwa_data_update_can_weight_data(&weights_data);

    return ESP_OK;
}


