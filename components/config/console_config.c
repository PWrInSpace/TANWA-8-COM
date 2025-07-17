///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains implementation of the system console configuration, including initialization
/// and available commands for debugging/testing purposes.
///===-----------------------------------------------------------------------------------------===//

#include "esp_log.h"
#include "esp_system.h"

#include "console.h"
#include "console_config.h"
#include "board_config.h"
#include "board_data.h"
#include "igniter_driver.h"
#include "can_commands.h"
#include "can_api.h"
#include "state_machine_config.h"
#include "state_machine.h"

#define TAG "CONSOLE_CONFIG"

extern tanwa_hardware_dev_t tanwa_hardware;

// example function to reset the device
int reset_device(int argc, char **argv) {
    ESP_LOGI(TAG, "Resetting device...");
    esp_restart();
    return 0;
}

int arm_igniters(int argc, char **argv) {
    // This function can be used to arm the igniter
    // Implementation depends on the specific hardware and requirements
    com_data_t data = tanwa_data_read_com_data();
    igniter_status_t ign_status = IGNITER_OK;
    ign_status = igniter_arm(&(tanwa_hardware.igniter[0]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter arm error | %d", (uint8_t)ign_status);
    }
    ign_status = igniter_arm(&(tanwa_hardware.igniter[1]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter arm error | %d", (uint8_t)ign_status);
    }

    if(ign_status == IGNITER_OK){
        data.arm_state = true;
        ESP_LOGI(TAG, "IGN | Igniter armed successfully");
    }
    else{
        data.arm_state = false;
        ESP_LOGE(TAG, "IGN | Igniter arm failed");
    }

    tanwa_data_update_com_data(&data);
    return 0;
}

int disarm_igniters(int argc, char **argv) {
    // This function can be used to disarm the igniter
    com_data_t data = tanwa_data_read_com_data();
    igniter_status_t ign_status = IGNITER_OK;
    ign_status = igniter_disarm(&(tanwa_hardware.igniter[0]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter disarm error | %d", (uint8_t)ign_status);
    }
    ign_status = igniter_disarm(&(tanwa_hardware.igniter[1]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter disarm error | %d", (uint8_t)ign_status);
    }

    data.arm_state = IGNITER_OK == ign_status ? false : true;

    tanwa_data_update_com_data(&data);
    return 0;
}

int fire_igniters(int argc, char **argv) {
    // This function can be used to fire the igniter
    com_data_t data = tanwa_data_read_com_data();
    igniter_status_t ign_status = IGNITER_OK;
    ign_status = igniter_fire(&(tanwa_hardware.igniter[0]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter fire error | %d", (uint8_t)ign_status);
    }
    ign_status = igniter_fire(&(tanwa_hardware.igniter[1]));
    if (ign_status != IGNITER_OK) {
        ESP_LOGE(TAG, "IGN | Igniter fire error | %d", (uint8_t)ign_status);
    }

    data.arm_state = IGNITER_OK == ign_status ? false : true;

    tanwa_data_update_com_data(&data);
    return 0;
}

int read_com_data(int argc, char **argv) {
    // This function can be used to read COM data
    com_data_t data = tanwa_data_read_com_data();
    ESP_LOGI(TAG, "COM Data: I_Sense: %.2f, Abort Button: %d, Arm State: %d, Relay States: [%d, %d, %d, %d], "
                  "Temperature 1: %.2f, Temperature 2: %.2f, Igniter Cont 1: %d, Igniter Cont 2: %d",
             data.i_sense, data.abort_button, data.arm_state,
             data.relay_state1, data.relay_state2, data.relay_state3, data.relay_state4,
             data.temperature_1, data.temperature_2,
             data.igniter_cont_1, data.igniter_cont_2);
    return 0;
}

int read_weight_data(int argc, char **argv) {
    // This function can be used to read weight data
    can_weight_data_t weight_data = tanwa_data_read_can_weight_data();
    ESP_LOGI(TAG, "Weight Data: Weight 1: %.2f, Weight 2: %.2f, Weight 3: %.2f, Weight 4: %.2f",
             weight_data.ads1_weight1, weight_data.ads1_weight2, weight_data.ads1_weight3, weight_data.ads1_weight4);
    ESP_LOGI(TAG, "Weight Data: Weight 5: %.2f, Weight 6: %.2f, Weight 7: %.2f, Weight 8: %.2f",
             weight_data.ads2_weight1, weight_data.ads2_weight2, weight_data.ads2_weight3, weight_data.ads2_weight4);
    ESP_LOGI(TAG, "Rocket Weight: %.2f, Tank Weight: %.2f",
             weight_data.rocket_weight, weight_data.tank_weight);
    return 0;
}

int read_sensor_data(int argc, char **argv) {
    // This function can be used to read sensor data
    can_sensor_pressure_data_t sensor_data = tanwa_data_read_can_sensor_pressure_data();
    can_sensor_temp_data_t temp_data = tanwa_data_read_can_sensor_temp_data();
    ESP_LOGI(TAG, "Sensor Data: Temperature 1: %d, Temperature 2: %d, Temperature 3: %d",
             temp_data.temperature1, temp_data.temperature2, temp_data.temperature3);
    ESP_LOGI(TAG, "PRESSURE Data: Pressure 1: %.2f, Pressure 2: %.2f, Pressure 3: %.2f, Pressure 4: %.2f",
             sensor_data.pressure1, sensor_data.pressure2, sensor_data.pressure3, sensor_data.pressure4);
    ESP_LOGI(TAG, "PRESSURE Data: Pressure 5: %.2f, Pressure 6: %.2f, Pressure 7: %.2f, Pressure 8: %.2f",
             sensor_data.pressure5, sensor_data.pressure6, sensor_data.pressure7, sensor_data.pressure8);
    return 0;
}

int read_solenoid_data(int argc, char **argv) {
    // This function can be used to read solenoid data
    can_solenoid_data_t solenoid_data = tanwa_data_read_can_solenoid_data();
    ESP_LOGI(TAG, "Solenoid Data: Solenoid States: [%d, %d, %d, %d, %d, %d], Servo States: [%d, %d, %d, %d]",
             solenoid_data.state_sol1, solenoid_data.state_sol2, solenoid_data.state_sol3,
             solenoid_data.state_sol4, solenoid_data.state_sol5, solenoid_data.state_sol6,
             solenoid_data.servo_state1, solenoid_data.servo_state2,
             solenoid_data.servo_state3, solenoid_data.servo_state4);
    return 0;
}

int read_power_data(int argc, char **argv) {
    // This function can be used to read power data
    can_power_data_t data = tanwa_data_read_can_power_data();
    ESP_LOGI(TAG, "Power Data: 12V Voltage: %d, 24V Voltage: %d, 12V Current: %d, 24V Current: %d",
             data.voltage_12V, data.volatage_24V, data.current_12V, data.current_24V);
    return 0;
}

int read_utility_data(int argc, char **argv) {
    // This function can be used to read utility data
    can_utility_status_t utility_data = tanwa_data_read_can_utility_status();
    ESP_LOGI(TAG, "Utility Data: I_Sense: %d, Temperature 1: %d, Temperature 2: %d, Switch States: [%d, %d, %d, %d, %d, %d, %d, %d]",
             utility_data.i_sense, utility_data.temperature1, utility_data.temperature2,
             utility_data.switch_state1, utility_data.switch_state2,
             utility_data.switch_state3, utility_data.switch_state4,
             utility_data.switch_state5, utility_data.switch_state6,
             utility_data.switch_state7, utility_data.switch_state8);
    return 0;
}

int tanwa_data_print(int argc, char **argv) {
    // This function can be used to print all TANWA data
    com_data_t com_data = tanwa_data_read_com_data();
    can_weight_data_t weight_data = tanwa_data_read_can_weight_data();
    can_sensor_temp_data_t sensor_temp_data = tanwa_data_read_can_sensor_temp_data();
    can_sensor_pressure_data_t sensor_pressure_data = tanwa_data_read_can_sensor_pressure_data();
    can_solenoid_data_t solenoid_data = tanwa_data_read_can_solenoid_data();
    can_power_data_t power_data = tanwa_data_read_can_power_data();
    can_utility_status_t utility_data = tanwa_data_read_can_utility_status();\
    can_solenoid_status_t solenoid_status = tanwa_data_read_can_solenoid_status();

    ESP_LOGI(TAG, "TANWA Data:");
    ESP_LOGI(TAG, "COM Data: I_Sense: %.2f, Abort Button: %d, Arm State: %d", com_data.i_sense, com_data.abort_button, com_data.arm_state);
    ESP_LOGI(TAG, "Relay States: [%d, %d, %d, %d], Temperature 1: %.2f, Temperature 2: %.2f, Igniter Cont 1: %d, Igniter Cont 2: %d",
             com_data.relay_state1, com_data.relay_state2, com_data.relay_state3, com_data.relay_state4,
             com_data.temperature_1, com_data.temperature_2,
             com_data.igniter_cont_1, com_data.igniter_cont_2);
    ESP_LOGI(TAG, "Weight Data: Rocket Weight: %.2f, Tank Weight: %.2f", weight_data.rocket_weight, weight_data.tank_weight);
    ESP_LOGI(TAG, "Sensor Temp Data: Temperature 1: %.2f, Temperature 2: %.2f, Temperature 3: %.2f", sensor_temp_data.temperature1, sensor_temp_data.temperature2, sensor_temp_data.temperature3);
    ESP_LOGI(TAG, "Pressure Data: Pressure 1: %.2f, Pressure 2: %.2f, Pressure 3: %.2f, Pressure 4: %.2f",
             sensor_pressure_data.pressure1, sensor_pressure_data.pressure2, sensor_pressure_data.pressure3, sensor_pressure_data.pressure4);
    ESP_LOGI(TAG, "Pressure Data: Pressure 5: %.2f, Pressure 6: %.2f, Pressure 7: %.2f, Pressure 8: %.2f",
             sensor_pressure_data.pressure5, sensor_pressure_data.pressure6, sensor_pressure_data.pressure7, sensor_pressure_data.pressure8);
    ESP_LOGI(TAG, "Solenoid Data: Temperature 1: %d, Temperature 2: %d, i_sense: %d",
             solenoid_status.temperature1, solenoid_status.temperature2, solenoid_status.i_sense);
    ESP_LOGI(TAG, "Solenoid Data: Servo Angle 1: %d, Servo Angle 2: %d", solenoid_data.servo_angle1, solenoid_data.servo_angle2);
    ESP_LOGI(TAG, "Solenoid Data: Servo Angle 3: %d, Servo Angle 4: %d", solenoid_data.servo_angle3, solenoid_data.servo_angle4);
    ESP_LOGI(TAG, "Solenoid Data: Motor State 1: %d, Motor State 2: %d",
             solenoid_data.motor_state1, solenoid_data.motor_state2);
    ESP_LOGI(TAG, "Solenoid States: [%d, %d, %d, %d, %d, %d], Servo States: [%d, %d, %d, %d]",
             solenoid_data.state_sol1, solenoid_data.state_sol2, solenoid_data.state_sol3,
             solenoid_data.state_sol4, solenoid_data.state_sol5, solenoid_data.state_sol6,
             solenoid_data.servo_state1, solenoid_data.servo_state2,
             solenoid_data.servo_state3, solenoid_data.servo_state4);
    ESP_LOGI(TAG, "Power Data: 12V Voltage: %d, 24V Voltage: %d, 12V Current: %d, 24V Current: %d",
             power_data.voltage_12V, power_data.volatage_24V,
             power_data.current_12V, power_data.current_24V);
    ESP_LOGI(TAG, "Utility Data: I_Sense: %d, Temperature 1: %d, Temperature 2: %d",
             utility_data.i_sense, utility_data.temperature1, utility_data.temperature2);
    ESP_LOGI(TAG, "Utility Switch States: [%d, %d, %d, %d, %d, %d, %d, %d]",
             utility_data.switch_state1, utility_data.switch_state2,
             utility_data.switch_state3, utility_data.switch_state4,
             utility_data.switch_state5, utility_data.switch_state6,
             utility_data.switch_state7, utility_data.switch_state8);

    return 0;
}

int open_solenoid(int argc, char **argv) {
    
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: open-solenoid <solenoid_id>");
        return -1;
    }

    int solenoid_id = atoi(argv[1]);
    if(solenoid_id < 0 || solenoid_id > 5) {
        ESP_LOGE(TAG, "Invalid solenoid ID. Must be between 0 and 5.");
        return -1;
    }
    
    uint8_t data[8] = {(uint8_t)solenoid_id, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    ESP_LOGI(TAG, "Solenoid %d opened", solenoid_id);
    return 0;
}

int open_solenoid_time(int argc, char **argv) {
    
    if(argc < 3) {
        ESP_LOGE(TAG, "Usage: open-solenoid-time <solenoid_id> <open_time>");
        return -1;
    }

    int solenoid_id = atoi(argv[1]);
    uint16_t open_time = (uint16_t)atoi(argv[2]);
    
    if(solenoid_id < 0 || solenoid_id > 5) {
        ESP_LOGE(TAG, "Invalid solenoid ID. Must be between 0 and 5.");
        return -1;
    }
    
    uint8_t data[8] = {(uint8_t)solenoid_id, 0, 0, 0, 0, 0, 0, 0};
    memcpy(&data[1], &open_time, sizeof(uint16_t)); // Copy open_time to data[1] and data[2]
    can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
    ESP_LOGI(TAG, "Solenoid %d opened for %d ms", solenoid_id, open_time);
    return 0;
}

int close_solenoid(int argc, char **argv) {
    
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: close-solenoid <solenoid_id>");
        return -1;
    }

    int solenoid_id = atoi(argv[1]);
    if(solenoid_id < 0 || solenoid_id > 5) {
        ESP_LOGE(TAG, "Invalid solenoid ID. Must be between 0 and 5.");
        return -1;
    }
    
    uint8_t data[8] = {(uint8_t)solenoid_id, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
    ESP_LOGI(TAG, "Solenoid %d closed", solenoid_id);
    return 0;
}

int open_relay(int argc, char **argv) {
    // This function can be used to open a relay
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: open-relay <relay_id>");
        return -1;
    }

    int relay_id = atoi(argv[1]);
    if(relay_id < 0 || relay_id > 3) {
        ESP_LOGE(TAG, "Invalid relay ID. Must be between 0 and 3.");
        return -1;
    }

    relay_open(&(tanwa_hardware.relay[relay_id]));
    ESP_LOGI(TAG, "Relay %d opened", relay_id);
    return 0;
}

int open_relay_time(int argc, char **argv) {
    // This function can be used to open a relay for a specified time
    if(argc < 3) {
        ESP_LOGE(TAG, "Usage: open-relay-time <relay_id> <open_time>");
        return -1;
    }

    int relay_id = atoi(argv[1]);
    uint16_t open_time = (uint16_t)atoi(argv[2]);
    
    if(relay_id < 0 || relay_id > 3) {
        ESP_LOGE(TAG, "Invalid relay ID. Must be between 0 and 3.");
        return -1;
    }

    relay_time_open(&(tanwa_hardware.relay[relay_id]), open_time);
    ESP_LOGI(TAG, "Relay %d opened for %d ms", relay_id, open_time);
    // Here you would typically start a timer to close the relay after open_time
    return 0;
}

int close_relay(int argc, char **argv) {
    // This function can be used to close a relay
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: close-relay <relay_id>");
        return -1;
    }

    int relay_id = atoi(argv[1]);
    if(relay_id < 0 || relay_id > 3) {
        ESP_LOGE(TAG, "Invalid relay ID. Must be between 0 and 3.");
        return -1;
    }

    relay_driver_err_t err = relay_close(&(tanwa_hardware.relay[relay_id]));
    if (err != RELAY_DRIVER_OK) {
        ESP_LOGE(TAG, "Relay close error | %d", (uint8_t)err);
        return -1;
    }
    
    ESP_LOGI(TAG, "Relay %d closed", relay_id);
    return 0;
}

int open_servo(int argc, char **argv) {
    // This function can be used to open a servo
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: open-servo <servo_id>");
        return -1;
    }

    int servo_id = atoi(argv[1]);
    if(servo_id < 0 || servo_id > 3) {
        ESP_LOGE(TAG, "Invalid servo ID. Must be between 0 and 3.");
        return -1;
    }

    uint8_t data[8] = {(uint8_t)servo_id, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 1);
    ESP_LOGI(TAG, "Servo %d opened", servo_id);
    return 0;
}

int close_servo(int argc, char **argv) {
    // This function can be used to close a servo
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: close-servo <servo_id>");
        return -1;
    }

    int servo_id = atoi(argv[1]);
    if(servo_id < 0 || servo_id > 3) {
        ESP_LOGE(TAG, "Invalid servo ID. Must be between 0 and 3.");
        return -1;
    }

    uint8_t data[8] = {(uint8_t)servo_id, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_CLOSE_ID, data, 1);
    ESP_LOGI(TAG, "Servo %d closed", servo_id);
    return 0;
}

int move_servo(int argc, char **argv) {
    // This function can be used to move a servo to a specified angle
    if(argc < 3) {
        ESP_LOGE(TAG, "Usage: move-servo <servo_id> <angle>");
        return -1;
    }

    int servo_id = atoi(argv[1]);
    int angle = atoi(argv[2]);
    
    if(servo_id < 0 || servo_id > 3) {
        ESP_LOGE(TAG, "Invalid servo ID. Must be between 0 and 3.");
        return -1;
    }
    
    if(angle < 0 || angle > 180) {
        ESP_LOGE(TAG, "Invalid angle. Must be between 0 and 180.");
        return -1;
    }

    uint8_t data[8] = {(uint8_t)servo_id, (uint8_t)(angle), 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SOL_SERVO_ANGLE_ID, data, 1);
    ESP_LOGI(TAG, "Servo %d moved to angle %d", servo_id, angle);
    return 0;
}

int set_weight_offset(int argc, char **argv) {
    // This function can be used to set the weight offset
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: set-weight-offset <offset>");
        return -1;
    }

    uint16_t offset = (uint16_t)atoi(argv[1]);

    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    memcpy(&data[0], &offset, sizeof(uint16_t)); // Copy offset to data[0] and data[1]
    can_send_message(CAN_WEIGHTS_SET_ADS_OFFSET_ID, data, 1);
    ESP_LOGI(TAG, "Weight offset set to %d", offset);
    return 0;
}

int print_pressure_info(int argc, char **argv) {
    
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SENSOR_PRESSURE_INFO_ID, data, 1);
    return 0;
}

int print_can_connected_slaves(int argc, char **argv) {
    // This function can be used to print CAN connected slaves
    can_connected_slaves_t slaves = tanwa_data_read_can_connected_slaves();
    ESP_LOGI(TAG, "CAN Connected Slaves:");
    ESP_LOGI(TAG, "Weights: %d, Solenoid: %d, Sensor: %d, Power: %d, Utility: %d",
             slaves.weights, slaves.solenoid, slaves.sensor, slaves.power, slaves.utility);
    return 0;
}

int set_power_channel(int argc, char **argv) {
    // This function can be used to set the power channel
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: set-pwr-channel <channel>");
        return -1;
    }

    int channel = atoi(argv[1]);
    
    if(channel < 0 || channel > 3) {
        ESP_LOGE(TAG, "Invalid power channel. Must be between 0 and 3.");
        return -1;
    }

    uint8_t data[8] = {(uint8_t)channel, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_POWER_SET_CHANNEL_ID, data, 1);
    ESP_LOGI(TAG, "Power channel set to %d", channel);
    return 0;
}

int print_power_channel(int argc, char **argv) {
    // This function can be used to print the current power channel
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_POWER_GET_CHANNEL_ID, data, 1);
    return 0;
}

int set_pressure_data_rate(int argc, char **argv) {
    // This function can be used to set the pressure data rate
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: set-press-data-rate <data_rate>");
        return -1;
    }

    uint8_t data_rate = (uint8_t)atoi(argv[1]);

    uint8_t data[8] = {data_rate, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_SENSOR_SET_PRESSURE_DATA_RATE_ID, data, 1);
    ESP_LOGI(TAG, "Pressure data rate set to %d", data_rate);
    return 0;
}

int clear_weights_data(int argc, char **argv) {
    // This function can be used to clear the weights data
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    can_send_message(CAN_WEIGHTS_CLEAR_SD_ID, data, 1);
    ESP_LOGI(TAG, "Weights data cleared");
    return 0;
}   

int tanwa_change_state(int argc, char **argv) {
    // This function can be used to change the state of the system
    if (argc != 2) {
        return -1;
    }

    int state = atoi(argv[1]);
    if (state == 11) {
        if (state_machine_get_current_state() == HOLD) {
            ESP_LOGI(TAG, "Leaving hold state");
            if (state_machine_get_previous_state() == COUNTDOWN) {
                state_machine_force_change_state(RDY_TO_LAUNCH);
            } else {
                state_machine_change_to_previous_state(true);
            }
        } else {
            state_machine_force_change_state(HOLD);
            ESP_LOGI(TAG, "HOLD");
        }
        return 0;
    }
    if (state_machine_force_change_state(state) != STATE_MACHINE_OK) {
        return -1;
    }

    return 0;
}

int tanwa_countdown(int argc, char **argv) {

    com_data_t data = tanwa_data_read_com_data();

    if(data.arm_state == 1) {
        ESP_LOGE(TAG, "Liquid arm state is not 1");
        return -1;
    }

    if(state_machine_change_state(COUNTDOWN) != STATE_MACHINE_OK) {
        ESP_LOGE(TAG, "Failed to change state to COUNTDOWN");
        return -1;
    }

    return 0;
}

 // Place for the console configuration

 static esp_console_cmd_t cmd [] = {
 // example command:
 // cmd     help description   hint  function      args
    {"reset", "Reset the device", NULL, reset_device, NULL, NULL, NULL},
    {"arm-igniters", "Arm the igniters", NULL, arm_igniters, NULL, NULL, NULL},
    {"disarm-igniters", "Disarm the igniters", NULL, disarm_igniters, NULL, NULL, NULL},
    {"fire-igniters", "Fire the igniters", NULL, fire_igniters, NULL, NULL, NULL},
    {"read-com-data", "Read COM data", NULL, read_com_data, NULL, NULL, NULL},
    {"read-weight-data", "Read weight data", NULL, read_weight_data, NULL, NULL, NULL},
    {"read-sensor-data", "Read sensor data", NULL, read_sensor_data, NULL, NULL, NULL},
    {"read-solenoid-data", "Read solenoid data", NULL, read_solenoid_data, NULL, NULL, NULL},
    {"read-power-data", "Read power data", NULL, read_power_data, NULL, NULL, NULL},
    {"read-utility-data", "Read utility data", NULL, read_utility_data, NULL, NULL, NULL},
    {"tanwa-data", "Print all data", NULL , tanwa_data_print, NULL, NULL, NULL},
    {"open-solenoid", "Open solenoid", NULL, open_solenoid, NULL, NULL, NULL},
    {"open-solenoid-time", "Open solenoid for specified time", NULL, open_solenoid_time, NULL, NULL, NULL},
    {"close-solenoid", "Close solenoid", NULL, close_solenoid, NULL, NULL, NULL},
    {"open-relay", "Open relay", NULL, open_relay, NULL, NULL, NULL},
    {"open-relay-time", "Open relay for specified time", NULL, open_relay_time, NULL, NULL, NULL},
    {"close-relay", "Close relay", NULL, close_relay, NULL, NULL, NULL},
    {"open-servo", "Open servo", NULL, open_servo, NULL, NULL, NULL},
    {"close-servo", "Close servo", NULL, close_servo, NULL, NULL, NULL},
    {"move-servo", "Move servo to specified angle", NULL, move_servo, NULL, NULL, NULL},
    {"set-weight-offset", "Set weight offset", NULL, set_weight_offset, NULL, NULL, NULL},
    {"press-info", "Print info about pressure", NULL, print_pressure_info, NULL, NULL, NULL},
    {"can-connected-slaves", "Print CAN connected slaves", NULL, print_can_connected_slaves, NULL, NULL, NULL},
    {"set-pwr-channel", "Set power channel", NULL, set_power_channel, NULL, NULL, NULL},
    {"pwr-channel", "Print power channel", NULL, print_power_channel, NULL, NULL, NULL},
    {"set-press-data-rate", "Set pressure data rate", NULL, set_pressure_data_rate, NULL, NULL, NULL},
    {"weights-clear-sd", "Clear weights data", NULL, clear_weights_data, NULL, NULL, NULL},
    {"change-state", "Change state of the system", NULL, tanwa_change_state, NULL, NULL, NULL},
    {"countdown", "Start countdown", NULL, tanwa_countdown, NULL, NULL, NULL},
};

esp_err_t console_config_init() {
    esp_err_t ret;
    ret = console_init();
    ret = console_register_commands(cmd, sizeof(cmd) / sizeof(cmd[0]));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(ret));
        return ret;
    }
    return ret;
}