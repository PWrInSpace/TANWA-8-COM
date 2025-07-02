#ifndef ENS_CONFIG_H
#define ENS_CONFIG_H

#include <stdint.h>
#include <stdlib.h>

#include "esp_now.h"

#define TX_NACK_TIMEOUT_MS 600000

#define MAX_NACK_TIMEOUT 1800000
#define MIN_NACK_TIMEOUT 120000

#define MAX_PERIOD_TIMEOUT 300000
#define MIN_PERIOD_TIMEOUT 10

#define STATE_MSG_SIZE 1

#define MAX_TX_BUFFER_SIZE 250

#define TIMER_INIT_MS 0

#define MAX_DATA_LENGTH sizeof(recv_cb_cmd_t)



typedef void (*data_to_transmit)(uint8_t *buffer, size_t buffer_size, size_t *tx_data_size);
typedef void (*on_data_rx)(uint8_t *buffer, size_t buffer_size);

typedef enum {

    ENS_OK = 0,
    ENS_NULL_ERR,
    ENS_PERIOD_ERR,
    ENS_ESP_NOW_ERR,
    ENS_QUEUE_ERR,
    ENS_MUTEX_ERR,
    ENS_TIMER_ERR,
    ENS_NACK_TMR_ERR,
    ENS_RX_OVRFLW_ERR

} ens_status_t;

typedef enum{

    INIT_MS = 0,
    IDLE_MS,
    ARMED_MS,
    FILLING_MS,
    ARMED_TO_LAUNCH_MS,
    RDY_TO_LAUNCH_MS,
    COUNTDOWN_MS,
    FLIGHT_MS,
    FIRST_STAGE_MS,
    SECOND_STAGE_MS,
    ON_GROUND_MS,
    HOLD_MS,
    ABORT_MS,
    SLEEP_MS,
    ENS_ENUM_MAX

} ens_transmit_period_t;

typedef struct {

    uint8_t dev_mac_address[ESP_NOW_ETH_ALEN];
    bool disable_sleep;
    uint32_t tx_nack_timeout_ms;
    data_to_transmit _data_to_transmit;
    on_data_rx _on_data_rx;

} ens_init_struct_t;

typedef union {
    struct command {
        uint32_t command;
        int32_t payload;
    } cmd;
    uint8_t raw[sizeof(struct command)];
} recv_cb_cmd_t;

typedef struct {
    esp_now_send_status_t message_status;
    uint8_t mac[ESP_NOW_ETH_ALEN];
} ens_send_cb_data_t;

typedef struct {
    uint8_t src_mac[ESP_NOW_ETH_ALEN];
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];
    recv_cb_cmd_t cmd;
    size_t data_size;
} ens_recv_cb_data_t;

uint8_t *get_obc_mac_address(void);
uint8_t *get_broadcast_mac_address(void);

#endif