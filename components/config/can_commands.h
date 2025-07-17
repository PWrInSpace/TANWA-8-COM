#ifndef PWRINSPACE_CAN_COMMANDS_H
#define PWRINSPACE_CAN_COMMANDS_H

#include "esp_err.h"

/** PLACE YOUR CAN CALLBACKS AND CAN MESSAGES HERE IN FORMAT*/
typedef enum {
    CAN_TEMPLATE_MESSAGE_ID = 0xFF,

    // Solenoid commands
    CAN_SOL_RESET_ID = 0x0C0,
    CAN_SOL_GET_STATUS_ID = 0x3CF,
    CAN_SOL_GET_DATA_ID = 0x2CE,
    CAN_SOL_OPEN_SOL_ID = 0x0CD,
    CAN_SOL_CLOSE_SOL_ID = 0x0CC,
    CAN_SOL_SERVO_OPEN_ID = 0x0CB,
    CAN_SOL_SERVO_CLOSE_ID = 0x0CA,
    CAN_SOL_SERVO_ANGLE_ID = 0x0C9,
    CAN_SOL_MOTOR_MOVE_ID = 0x0C8,

    CAN_SOL_STATUS_ID = 0x3CD,
    CAN_SOL_DATA_ID = 0x3C0,

    // Power commands
    CAN_POWER_RESET_ID = 0x0B0,
    CAN_POWER_SET_CHANNEL_ID = 0x0BD,
    CAN_POWER_GET_CHANNEL_ID = 0x3B2,
    CAN_POWER_GET_STATUS_ID = 0x0BF,
    CAN_POWER_GET_DATA_ID = 0x0BE,

    CAN_POWER_STATUS_ID = 0x3B1,
    CAN_POWER_DATA_ID = 0x3B0,
    CAN_POWER_CHANNEL_ID = 0x3B3,

    //Sensor commands
    CAN_SENSOR_RESET_ID = 0x0D0,
    CAN_SENSOR_GET_PRESSURE_ID = 0x2DD,
    CAN_SENSOR_GET_TEMPERATURE_ID = 0x2DC,
    CAN_SENSOR_SET_PRESSURE_DATA_RATE_ID = 0x3DB,
    CAN_SENSOR_GET_STATUS_ID = 0x3DF,
    CAN_SENSOR_GET_DATA_ID = 0x2DE,
    CAN_SENSOR_GET_PRESSURE_INFO_ID = 0x3DA,

    CAN_SENSOR_STATUS_ID = 0x3DD,
    CAN_SENSOR_DATA_ID = 0x3D0,
    CAN_SENSOR_PRESSURE1_ID = 0x3D11,
    CAN_SENSOR_PRESSURE2_ID = 0x3D12,
    CAN_SENSOR_TEMPERATURE_ID = 0x3D2,
    CAN_SENSOR_PRESSURE_INFO_ID = 0x3D3,

    //Utility commands
    CAN_UTIL_RESET_ID = 0x0E0,
    CAN_UTIL_GET_STATUS_ID = 0x3EF,
    CAN_UTIL_GET_DATA_ID = 0x2EF,

    CAN_UTIL_STATUS_ID = 0x3EE,
    CAN_UTIL_DATA_ID = 0x3ED,
    
    //Weights commands
    CAN_WEIGHTS_RESET_ID = 0x0F0,
    CAN_WEIGHTS_GET_STATUS_ID = 0x3FF,
    CAN_WEIGHTS_GET_BOARD_DATA_ID = 0x2FE,
    CAN_WEIGHTS_START_MEASURE_ID = 0x0FD,
    CAN_WEIGHTS_ADS_TARE_ID = 0x3FC,
    CAN_WEIGHTS_SET_ADS_GAIN_ID = 0x3FB,
    CAN_WEIGHTS_SET_ADS_CH_ID = 0x3FA,
    CAN_WEIGHTS_SET_ADS_OFFSET_ID = 0x3F9,
    CAN_WEIGHTS_GET_ADS_ALL_WEIGHT_ID = 0x2F9,
    CAN_WEIGHTS_GET_ADS_CH_WEIGHT_ID = 0x2F8,
    CAN_WEIGHTS_CLEAR_SD_ID = 0x2F7,
    CAN_WEIGHTS_GET_WEIGHTS_ID = 0x2F4,

    CAN_WEIGHTS_STATUS_ID = 0x3F4,
    CAN_WEIGHTS_BOARD_DATA_ID = 0x3F11,
    CAN_WEIGHTS_ADS1_ALL_CH_WEIGHT1_ID = 0x3F12,
    CAN_WEIGHTS_ADS1_ALL_CH_WEIGHT2_ID = 0x3F13,
    CAN_WEIGHTS_ADS2_ALL_CH_WEIGHT1_ID = 0x3F14,
    CAN_WEIGHTS_ADS2_ALL_CH_WEIGHT2_ID = 0x3F15,
    CAN_WEIGHTS_ADS_CH_WEIGHT_ID = 0x3F2,
    CAN_WEIGHTS_WEIGHTS_ID = 0x3F3,

 /*    CAN_MSG_ID_1 = ...,
 *     CAN_MSG_ID_2 = ...,
 *     CAN_MSG_ID_3 = ...,
 *     CAN_MSG_ID_4 = ...,
 *     CAN_MSG_ID_5 = ...,
 */
} can_message_id_t;

/*PLACE YOUR FUNCTIONS ACCORDING TO THE TEMPLATE
* typedef esp_err_t (*can_command_handler_t)(uint8_t *data, uint8_t length);
* 
* REGISTER THEM IN can_config.c FILE
*/

esp_err_t example_command_handler(uint8_t *data, uint8_t length);
esp_err_t parse_solenoid_status(uint8_t *data, uint8_t length);
esp_err_t parse_solenoid_data(uint8_t *data, uint8_t length);
esp_err_t parse_power_status(uint8_t *data, uint8_t length);
esp_err_t parse_power_data(uint8_t *data, uint8_t length);
esp_err_t parse_power_channel(uint8_t *data, uint8_t length);
esp_err_t parse_sensor_status(uint8_t *data, uint8_t length);
esp_err_t parse_sensor_data(uint8_t *data, uint8_t length);
esp_err_t parse_sensor_pressure1(uint8_t *data, uint8_t length);
esp_err_t parse_sensor_pressure2(uint8_t *data, uint8_t length);
esp_err_t parse_sensor_temperature(uint8_t *data, uint8_t length);
esp_err_t parse_sensor_pressure_info(uint8_t *data, uint8_t length);
esp_err_t parse_util_status(uint8_t *data, uint8_t length);
esp_err_t parse_util_data(uint8_t *data, uint8_t length);
esp_err_t parse_weights_status(uint8_t *data, uint8_t length);
esp_err_t parse_weights_board_data(uint8_t *data, uint8_t length);
esp_err_t parse_weights_ads1_all_ch_weight1(uint8_t *data, uint8_t length);
esp_err_t parse_weights_ads1_all_ch_weight2(uint8_t *data, uint8_t length);
esp_err_t parse_weights_ads2_all_ch_weight1(uint8_t *data, uint8_t length);
esp_err_t parse_weights_ads2_all_ch_weight2(uint8_t *data, uint8_t length);
esp_err_t parse_weights_ads_ch_weight(uint8_t *data, uint8_t length);
esp_err_t parse_weights(uint8_t *data, uint8_t length);


#endif //PWRINSPACE_CAN_COMMANDS_H