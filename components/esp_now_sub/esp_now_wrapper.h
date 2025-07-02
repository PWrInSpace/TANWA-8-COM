#ifndef ESP_NOW_WRAPPER_H
#define ESP_NOW_WRAPPER_H

#include "ens_config.h"

ens_status_t _esp_now_init(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]);
ens_status_t get_recieved_data(ens_recv_cb_data_t *recv_data);
void esp_go_to_sleep(uint64_t interval_ms);
ens_status_t send_data(uint8_t *dest_mac, uint8_t *data, int len);
ens_status_t get_send_data_ack(ens_send_cb_data_t *send_data);
ens_status_t get_mac_address(uint8_t *mac_address);


#endif // ESP_NOW_WRAPPER_H