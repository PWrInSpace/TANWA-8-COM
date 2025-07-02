// Copyright 2022 PWrInSpace, Kuba
#ifndef LORA_TASK_H
#define LORA_TASK_H

#include <stdint.h>

#include "lora.h"

#define LORA_TASK_CRC_ENABLE 0
#define LORA_TASK_FREQUENCY_KHZ 868000
#define LORA_TASK_BANDWIDTH 8
#define LORA_TASK_SPREADING_FACTOR 1
#define LORA_TASK_RECEIVE_WINDOW 1500
#define LORA_TASK_TRANSMIT_MS 1800

#define PRIVILAGE_MASK 0x01
#define BORADCAST_DEV_ID 0x00

#define PACKET_PREFIX "SP3MIK"

typedef uint8_t lora_dev_id;
typedef void (*lora_task_process_rx_packet)(uint8_t *packet, size_t packet_size);
typedef size_t (*lora_task_get_tx_packet)(uint8_t *buffer, size_t buffer_size);

typedef enum {
    LORA_IDLE,
    LORA_SLEEP,
    LORA_TRANSMIT,
    LORA_RECEIVE,
} lora_state_t;

typedef struct {
    lora_task_process_rx_packet process_rx_packet_fnc;
    lora_task_get_tx_packet get_tx_packet_fnc;
    lora_struct_t *lora;
    uint32_t frequency_khz;
    uint32_t transmiting_period;
} lora_api_config_t;

/**
 * @brief Initialize lora api and run lora task
 *q
 * @param frequency_khz lora frequency kzh
 * @param transmiting_period lora transmiting period
 * @return true :D
 * @return false :C
 */
bool initialize_lora(uint32_t frequency_khz, uint32_t transmiting_period);

/**
 * @brief Attach this function to d0 interrupt handler
 *
 * @param arg nothing
 */
void lora_task_irq_notify(void *arg);

bool lora_change_frequency(uint32_t frequency_khz);

/**
 * @brief Initialize lora task
 *
 * @param cfg pointer to config
 * @return true :D
 * @return false :C
 */
bool lora_task_init(lora_api_config_t *cfg);

void lora_task(void* pvParameters);

#endif