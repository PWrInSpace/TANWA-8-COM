#ifndef ENS_LOGIC_H
#define ENS_LOGIC_H

#include "ens_config.h"

void on_send(ens_send_cb_data_t *send_data);
bool can_send_data();
void reset_transmit_timer();
bool is_ack_time_exeeded(uint64_t current_time_ms);
esp_now_send_status_t _get_last_message_status();
uint64_t get_interval_ms();
bool is_from_mcb(const uint8_t *mac);
bool is_broadcast(const uint8_t *mac);
void set_mission_state(ens_recv_cb_data_t *recv_data);
bool is_cmd_msg(ens_recv_cb_data_t *recv_data);
bool is_sleep_interval(uint64_t interval_ms);
ens_status_t logic_init(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]);

#endif