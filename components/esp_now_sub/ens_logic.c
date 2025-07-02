#include <string.h>

#include "esp_now.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "ens_logic.h"

#define TAG "ENS_LOGIC"

#define INIT_FUNC_ARRAY_SIZE 8

#define US_TO_MS(US) (US/1000)
#define MS_TO_US(MS) (MS*1000)

static struct {
    esp_now_send_status_t last_message_status;
    uint32_t tx_nack_timeout_ms;
    uint32_t last_tx_time_ms;
    ens_transmit_period_t current_mission_state;
    uint32_t transmit_timer_ms;
    uint8_t dev_mac_address[ESP_NOW_ETH_ALEN];
    uint16_t transmit_periods[ENS_ENUM_MAX];
} gb;

static ens_status_t init_config_struct(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){
    
    if(init_struct == NULL){
        return ENS_NULL_ERR;
    }

    memcpy(gb.dev_mac_address, init_struct->dev_mac_address, ESP_NOW_ETH_ALEN);

    return ENS_OK;
}

static ens_status_t init_transmit_periods(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){

    if(transmit_periods == NULL){
        return ENS_NULL_ERR;
    }

    for(int i = 0; i < ENS_ENUM_MAX; i++){

        if(transmit_periods[i] > MAX_PERIOD_TIMEOUT || transmit_periods[i] < MIN_PERIOD_TIMEOUT){
            return ENS_PERIOD_ERR;
        }

        gb.transmit_periods[i] = transmit_periods[i];
    }

    return ENS_OK;
}

static ens_status_t init_timers(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){

    if(TX_NACK_TIMEOUT_MS > MAX_NACK_TIMEOUT || TX_NACK_TIMEOUT_MS < MIN_NACK_TIMEOUT){

        return ENS_NACK_TMR_ERR;
    }

    gb.tx_nack_timeout_ms = TX_NACK_TIMEOUT_MS;
    gb.last_tx_time_ms = US_TO_MS(esp_timer_get_time());
    gb.transmit_timer_ms = TIMER_INIT_MS;

    return ENS_OK;
}

static ens_status_t init_mission_state(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){

    if(transmit_periods == NULL){
        return ENS_NULL_ERR;
    }

    gb.current_mission_state = INIT_MS;

    if(transmit_periods[gb.current_mission_state] < MIN_PERIOD_TIMEOUT || transmit_periods[gb.current_mission_state] > MAX_PERIOD_TIMEOUT){
        return ENS_PERIOD_ERR;
    }

    gb.transmit_timer_ms = US_TO_MS(esp_timer_get_time());

    return ENS_OK;
}

ens_status_t logic_init(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){

    if(init_config_struct(init_struct, transmit_periods) != ENS_OK){
        return ENS_NULL_ERR;
    }

    if(init_transmit_periods(init_struct, transmit_periods) != ENS_OK){
        return ENS_PERIOD_ERR;
    }

    if(init_timers(init_struct, transmit_periods) != ENS_OK){
        return ENS_TIMER_ERR;
    }

    if(init_mission_state(init_struct, transmit_periods) != ENS_OK){
        return ENS_TIMER_ERR;
    }

    gb.last_message_status = ESP_NOW_SEND_SUCCESS;
    gb.current_mission_state = INIT_MS;

    return ENS_OK;
}

esp_now_send_status_t _get_last_message_status(void){

    return gb.last_message_status;
}

bool can_send_data(){

    if(gb.transmit_timer_ms == TIMER_INIT_MS){

        return true;
    }

    uint64_t current_time_ms = US_TO_MS(esp_timer_get_time());
    
    return current_time_ms - gb.transmit_timer_ms > get_interval_ms();
}

inline void reset_transmit_timer(){
    
    gb.transmit_timer_ms = US_TO_MS(esp_timer_get_time());
}

inline bool is_ack_time_exeeded(uint64_t current_time_ms){
    return current_time_ms - gb.last_tx_time_ms > gb.tx_nack_timeout_ms;
}

inline uint64_t get_interval_ms(){

    return gb.transmit_periods[gb.current_mission_state];
}

inline bool is_sleep_interval(uint64_t interval_ms){

    return interval_ms == gb.transmit_periods[SLEEP_MS];
}

void on_send(ens_send_cb_data_t *send_data){

    uint64_t current_time_ms = US_TO_MS(esp_timer_get_time());

    gb.last_message_status = send_data->message_status;

    if(send_data->message_status == ESP_NOW_SEND_SUCCESS){

        gb.last_tx_time_ms = current_time_ms;

    }
    else{

        if(is_ack_time_exeeded(current_time_ms)){

            gb.current_mission_state = SLEEP_MS;
        }
    }
}

inline bool is_from_mcb(const uint8_t *mac){

    uint8_t *obc_mac = get_obc_mac_address();

    return memcmp(mac, obc_mac, ESP_NOW_ETH_ALEN) == 0;
}

inline bool is_broadcast(const uint8_t *mac){

    return memcmp(mac, get_broadcast_mac_address(), ESP_NOW_ETH_ALEN) == 0;
}

void set_mission_state(ens_recv_cb_data_t *recv_data){

    if(recv_data->data_size == STATE_MSG_SIZE && recv_data->cmd.raw[0] < ENS_ENUM_MAX){

        ESP_LOGI(TAG, "Setting mission state to %d", recv_data->cmd.raw[0]);

        gb.current_mission_state = recv_data->cmd.raw[0];
    }
}

static inline bool is_addressed_to_me(const uint8_t *mac){

    return memcmp(mac, gb.dev_mac_address, ESP_NOW_ETH_ALEN) == 0;
}

inline bool is_cmd_msg(ens_recv_cb_data_t *recv_data){

    return is_addressed_to_me(recv_data->dest_mac);
}

