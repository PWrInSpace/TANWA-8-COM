#ifndef PWRINSPACE_CAN_COMMANDS_H
#define PWRINSPACE_CAN_COMMANDS_H

#include "esp_err.h"

/** PLACE YOUR CAN CALLBACKS AND CAN MESSAGES HERE IN FORMAT*/
typedef enum {
    CAN_TEMPLATE_MESSAGE_ID = 0xFF,

    CAN_SOL_STATUS_ID = 0x3CD,
    CAN_SOL_DATA_ID = 0x3C0,
    CAN_SOL_GET_STATUS_ID = 0x3CF,
    CAN_SOL_GET_DATA_ID = 0x2CE,

    CAN_SOL_OPEN_SOL_ID = 0x0CD,
    CAN_SOL_CLOSE_SOL_ID = 0x0CC,
    CAN_SOL_SERVO_OPEN_ID = 0x0CB,
    CAN_SOL_SERVO_CLOSE_ID = 0x0CA,
    CAN_SOL_SERVO_ANGLE_ID = 0x0C9,
    CAN_SOL_MOTOR_MOVE_ID = 0x0C8,

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

#endif //PWRINSPACE_CAN_COMMANDS_H