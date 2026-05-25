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
#include "timers_config.h"

#define TAG "CONSOLE_CONFIG"

// Definicje kolorów ANSI dla poprawy czytelności
#define CLR_RST  "\x1b[0m"
#define CLR_BLD  "\x1b[1m"
#define CLR_RED  "\x1b[31m"
#define CLR_GRN  "\x1b[32m"
#define CLR_YLW  "\x1b[33m"
#define CLR_BLU  "\x1b[34m"
#define CLR_MAG  "\x1b[35m"
#define CLR_CYN  "\x1b[36m"



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

    sys_timer_start(TIMER_IGNITION_OFF, IGNITION_OFF_TIMER, TIMER_TYPE_ONE_SHOT);
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
    can_power_data_t data = tanwa_data_read_can_power_data();
    
    // Konwersja jednostek (zakładając mV i mA z magistrali CAN)
    float v_sys = (float)data.VOLTAGE_24V_SYS / 1000.0f;
    float i_sys = (float)data.current_24V_SYS / 1000.0f;
    float v_sol = (float)data.VOLTAGE_24V_SOL / 1000.0f;
    float i_sol = (float)data.current_24V_SOL / 1000.0f;

    // Nagłówek tabeli zasilania (jeśli funkcja jest wywoływana samodzielnie)
    // Jeśli wywołujesz to wewnątrz tanwa_data_print, ramki się ładnie zgrają.
    printf("----------------------------------------------------------------------------\n");
    printf("| %-20s | %s%8.2f V%s | %-20s | %8.2f mA |\n", 
           "24V System Voltage", CLR_YLW, v_sys, CLR_RST, "System Current", i_sys);
    printf("| %-20s | %s%8.2f V%s | %-20s | %8.2f mA |\n", 
           "24V Solenoid Voltage", CLR_YLW, v_sol, CLR_RST, "Solenoid Current", i_sol);
    printf("----------------------------------------------------------------------------\n");

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
    // Nagłówek raportu
    printf("\n%s%s======================== [ TANWA SYSTEM DATA REPORT ] ========================%s\n", 
           CLR_BLD, CLR_CYN, CLR_RST);

    // --- 1. COM & IGNITION DATA ---
    com_data_t com = tanwa_data_read_com_data();
    printf("%s%s%-25s%s\n", CLR_BLD, CLR_YLW, "1. COM & IGNITION DATA", CLR_RST);
    printf("----------------------------------------------------------------------------\n");
    printf("| %-20s | %-15.2f | %-20s | %-12s |\n", "I_Sense", com.i_sense, "Abort Button", com.abort_button ? "PRESSED" : "OK");
    printf("| %-20s | %s%-15s%s | %-20s | %-12.2f |\n", "Arm State", 
           com.arm_state ? CLR_RED : CLR_GRN, com.arm_state ? "ARMED" : "DISARMED", CLR_RST,
           "Temp Internal", com.temperature_1);
    printf("----------------------------------------------------------------------------\n\n");

    // --- 2. WEIGHT DATA ---
    can_weight_data_t weight = tanwa_data_read_can_weight_data();
    printf("%s%s%-25s%s\n", CLR_BLD, CLR_YLW, "2. LOAD CELL / WEIGHT DATA", CLR_RST);
    printf("----------------------------------------------------------------------------\n");
    printf("| %-20s | %10.2f g | %-20s | %10.2f g |\n", "Rocket Weight", weight.rocket_weight, "Tank Weight", weight.tank_weight);
    printf("| %-20s | %10.2f g | %-20s | %10.2f g |\n", "Load Cell 1", weight.ads1_weight1, "Load Cell 2", weight.ads1_weight2);
    printf("----------------------------------------------------------------------------\n\n");

    // --- 3. SENSORS & PRESSURE ---
    can_sensor_pressure_data_t press = tanwa_data_read_can_sensor_pressure_data();
    can_sensor_temp_data_t s_temp = tanwa_data_read_can_sensor_temp_data();
    printf("%s%s%-25s%s\n", CLR_BLD, CLR_YLW, "3. SENSORS & PRESSURE", CLR_RST);
    printf("----------------------------------------------------------------------------\n");
    printf("| %-15s | %8.2f bar | %-15s | %8.2f bar |\n", "Press 1", press.pressure1, "Press 5", press.pressure5);
    printf("| %-15s | %8.2f bar | %-15s | %8.2f bar |\n", "Press 2", press.pressure2, "Press 6", press.pressure6);
    printf("| %-15s | %8.2f bar | %-15s | %8.2f bar |\n", "Press 3", press.pressure3, "Press 7", press.pressure7);
    printf("| %-15s | %8.2f bar | %-15s | %8.2f bar |\n", "Press 4", press.pressure4, "Press 8", press.pressure8);
    printf("| %-15s | %8.2f C   | %-15s | %8.2f C   |\n", "Sensor Temp 1", (float)s_temp.temperature1, "Sensor Temp 2", (float)s_temp.temperature2);
    printf("| %-15s | %8.2f C   | %-15s | %-12s |\n", "Sensor Temp 3", (float)s_temp.temperature3, "Status", "OK");
    printf("----------------------------------------------------------------------------\n\n");

    // --- 4. SOLENOIDS & SERVOS ---
    can_solenoid_data_t sol = tanwa_data_read_can_solenoid_data();
    printf("%s%s%-25s%s\n", CLR_BLD, CLR_YLW, "4. SOLENOIDS & SERVOS", CLR_RST);
    printf("----------------------------------------------------------------------------\n");
    printf("| Solenoids: [%d] [%d] [%d] [%d] [%d] [%d] | Servos: [%d] [%d] [%d] [%d] |\n",
           sol.state_sol1, sol.state_sol2, sol.state_sol3, sol.state_sol4, sol.state_sol5, sol.state_sol6,
           sol.servo_state1, sol.servo_state2, sol.servo_state3, sol.servo_state4);
    printf("| Servo Angles: %3d deg | %3d deg | %3d deg | %3d deg                  |\n",
           sol.servo_angle1, sol.servo_angle2, sol.servo_angle3, sol.servo_angle4);
    printf("----------------------------------------------------------------------------\n\n");

    // --- 5. POWER SYSTEM ---
    printf("%s%s%-25s%s\n", CLR_BLD, CLR_YLW, "5. POWER SYSTEM STATUS", CLR_RST);
    read_power_data(argc, argv);
    printf("\n");

    // --- 6. UTILITY & STATE ---
    printf("%s%s%-25s%s\n", CLR_BLD, CLR_YLW, "6. SYSTEM STATUS", CLR_RST);
    state_t current_state = state_machine_get_current_state();
    printf("| %-20s | %s%s%-20d%s |\n", "Current State", CLR_BLD, CLR_GRN, (int)current_state, CLR_RST);
    
    print_can_connected_slaves(argc, argv);

    printf("\n%s%s================================ [ END OF REPORT ] ================================%s\n\n", 
           CLR_BLD, CLR_CYN, CLR_RST);

    return 0;
}

int open_solenoid(int argc, char **argv) {
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: sol-open <solenoid_id> [open_time_ms]");
        return -1;
    }

    int solenoid_id = atoi(argv[1]);
    if(solenoid_id < 0 || solenoid_id > 5) {
        ESP_LOGE(TAG, "Invalid solenoid ID. Must be between 0 and 5.");
        return -1;
    }

    uint8_t data[8] = {(uint8_t)solenoid_id, 0, 0, 0, 0, 0, 0, 0};

    if(argc == 3) {
        uint16_t open_time = (uint16_t)atoi(argv[2]);
        memcpy(&data[1], &open_time, sizeof(uint16_t)); // Copy open_time to data[1] and data[2]
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 3);
        ESP_LOGI(TAG, "Solenoid %d opened for %d ms", solenoid_id, open_time);
    } else {
        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
        ESP_LOGI(TAG, "Solenoid %d opened", solenoid_id);
    }
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
    if(argc < 2) {
        ESP_LOGE(TAG, "Usage: open-relay <relay_id> [open_time_ms]");
        return -1;
    }

    int relay_id = atoi(argv[1]);
    if(relay_id < 0 || relay_id > 3) {
        ESP_LOGE(TAG, "Invalid relay ID. Must be between 0 and 3.");
        return -1;
    }

    if(argc == 3) {
        uint16_t open_time = (uint16_t)atoi(argv[2]);
        relay_time_open(&(tanwa_hardware.relay[relay_id]), open_time);
        ESP_LOGI(TAG, "Relay %d opened for %d ms", relay_id, open_time);
    } else {
        relay_open(&(tanwa_hardware.relay[relay_id]));
        ESP_LOGI(TAG, "Relay %d opened", relay_id);
    }
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
    if(argc < 3) {
        ESP_LOGE(TAG, "Usage: open-servo <servo_id> <time>");
        return -1;
    }

    int servo_id = atoi(argv[1]);
    if(servo_id < 0 || servo_id > 3) {
        ESP_LOGE(TAG, "Invalid servo ID. Must be between 0 and 3.");
        return -1;
    }

    uint16_t open_time = (uint16_t)atoi(argv[2]);

    uint8_t data[8] = {(uint8_t)servo_id, 0, 0, 0, 0, 0, 0, 0};
    memcpy(data+1, &open_time, sizeof(uint16_t));
    can_send_message(CAN_SOL_SERVO_OPEN_ID, data, 3);
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
    can_send_message(CAN_SOL_SERVO_ANGLE_ID, data, 2);
    ESP_LOGI(TAG, "Servo %d moved to angle %d", servo_id, angle);
    return 0;
}

int set_weight_offset(int argc, char **argv) {
    // This function can be used to set the weight offset
    if(argc < 4) {
        ESP_LOGE(TAG, "Usage: set-weight-offset <offset>");
        return -1;
    }

    uint8_t channel = (uint8_t)atoi(argv[1]);
    if(channel > 3) {
        ESP_LOGE(TAG, "Invalid channel. Must be between 0 and 3.");
        return -1;
    }

    uint8_t ads_num = (uint8_t)atoi(argv[2]);
    if(ads_num > 1) {
        ESP_LOGE(TAG, "Invalid ADS number. Must be 0 or 1.");
        return -1;
    }

    uint32_t offset = atoi(argv[3]);

    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    memcpy(&data[2], &offset, sizeof(uint32_t));
    data[1] = channel; // Set channel
    data[0] = ads_num; // Set ADS number
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

    if(data.arm_state == false) {
        ESP_LOGE(TAG, "Liquid arm state is not 1");
        return -1;
    }

    if(state_machine_change_state(COUNTDOWN) != STATE_MACHINE_OK) {
        ESP_LOGE(TAG, "Failed to change state to COUNTDOWN");
        return -1;
    }

    return 0;
}

int change_buzzer_state(int argc, char **argv) {
    // Oczekujemy komendy w stylu: buzzer [state] [freq] (np. buzzer 1 2000)
    // Zatem argc musi wynosić dokładnie 3
    if (argc != 3) {
        ESP_LOGW("CONSOLE", "Złe użycie! Format: buzzer <0/1> <czestotliwosc>");
        return -1;
    }

    bool state = (atoi(argv[1]) != 0); // bezpieczna konwersja na bool
    uint16_t freq_s = (uint16_t)atoi(argv[2]); // pobieramy częstotliwość z argv[2]
    
    buzzer_toggle(state, freq_s);

    return 0;
}

 // Place for the console configuration

 static esp_console_cmd_t cmd [] = {
 // example command:
 // cmd     help description   hint  function      args
    {"reset", "Reset the device", NULL, reset_device, NULL, NULL, NULL},
    {"igniters-arm", "Arm the igniters", NULL, arm_igniters, NULL, NULL, NULL},
    {"igniters-disarm", "Disarm the igniters", NULL, disarm_igniters, NULL, NULL, NULL},
    {"igniters-fire", "Fire the igniters", NULL, fire_igniters, NULL, NULL, NULL},
    {"com-data", "Read COM data", NULL, read_com_data, NULL, NULL, NULL},
    {"weight-data", "Read weight data", NULL, read_weight_data, NULL, NULL, NULL},
    {"sensor-data", "Read sensor data", NULL, read_sensor_data, NULL, NULL, NULL},
    {"solenoid-data", "Read solenoid data", NULL, read_solenoid_data, NULL, NULL, NULL},
    {"power-data", "Read power data", NULL, read_power_data, NULL, NULL, NULL},
    {"utility-data", "Read utility data", NULL, read_utility_data, NULL, NULL, NULL},
    {"tanwa-data", "Print all data", NULL , tanwa_data_print, NULL, NULL, NULL},
    {"sol-open", "Open solenoid", NULL, open_solenoid, NULL, NULL, NULL},
    {"sol-close", "Close solenoid", NULL, close_solenoid, NULL, NULL, NULL},
    {"rel-open", "Open relay", NULL, open_relay, NULL, NULL, NULL},
    {"rel-close", "Close relay", NULL, close_relay, NULL, NULL, NULL},
    {"servo-open", "Open servo", NULL, open_servo, NULL, NULL, NULL},
    {"servo-close", "Close servo", NULL, close_servo, NULL, NULL, NULL},
    {"servo-move", "Move servo to specified angle", NULL, move_servo, NULL, NULL, NULL},
    {"weight-set-offset", "Set weight offset", NULL, set_weight_offset, NULL, NULL, NULL},
    {"press-info", "Print info about pressure", NULL, print_pressure_info, NULL, NULL, NULL},
    {"can-connected-slaves", "Print CAN connected slaves", NULL, print_can_connected_slaves, NULL, NULL, NULL},
    {"pwr-set-channel", "Set power channel", NULL, set_power_channel, NULL, NULL, NULL},
    {"pwr-channel", "Print power channel", NULL, print_power_channel, NULL, NULL, NULL},
    {"press-set-data-rate", "Set pressure data rate", NULL, set_pressure_data_rate, NULL, NULL, NULL},
    {"change-state", "Change state of the system", NULL, tanwa_change_state, NULL, NULL, NULL},
    {"countdown", "Start countdown", NULL, tanwa_countdown, NULL, NULL, NULL},
    {"buzzer", "Change buzzer state", NULL, change_buzzer_state, NULL, NULL, NULL},
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