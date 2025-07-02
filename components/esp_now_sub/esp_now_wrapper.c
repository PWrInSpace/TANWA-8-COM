#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sleep.h"

#include "ens_config.h"

static struct {
    QueueHandle_t send_cb_queue;
    QueueHandle_t recv_cb_queue;
} gb;

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
static void send_cb(const uint8_t *mac, esp_now_send_status_t status);

ens_status_t _esp_now_init(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){

    if(init_struct == NULL){
        return ENS_NULL_ERR;
    }

    gb.send_cb_queue = xQueueCreate(1, sizeof(ens_send_cb_data_t));
    if(gb.send_cb_queue == NULL){
        return ENS_QUEUE_ERR;
    }

    gb.recv_cb_queue = xQueueCreate(1, sizeof(ens_recv_cb_data_t));
    if(gb.recv_cb_queue == NULL){
        return ENS_QUEUE_ERR;
    }

    if(nvs_flash_init() != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_netif_init() != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_event_loop_create_default() != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if(esp_wifi_init(&cfg) != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_wifi_set_mac(WIFI_IF_STA , init_struct->dev_mac_address) != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_wifi_start() != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_now_init() != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    if(esp_now_register_send_cb(send_cb) != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }
    if(esp_now_register_recv_cb(recv_cb) != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    return ENS_OK;
}





void esp_go_to_sleep(uint64_t interval_ms){

    esp_sleep_enable_timer_wakeup(interval_ms * 1000);
    esp_deep_sleep_start();
}

static void send_cb(const uint8_t *mac, esp_now_send_status_t status){

    ens_send_cb_data_t send_data;

    send_data.message_status = status;

    memcpy(send_data.mac, mac, ESP_NOW_ETH_ALEN);

    xQueueSend(gb.send_cb_queue, &send_data, 0);
    
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len){

    if(info->src_addr == NULL || data == NULL || len == 0 || len > MAX_DATA_LENGTH){
        return;
    }

    ens_recv_cb_data_t recv_data;

    memcpy(recv_data.src_mac, info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(recv_data.dest_mac, info->des_addr, ESP_NOW_ETH_ALEN);
    memcpy(&recv_data.cmd, data, len);
    recv_data.data_size = len;

    xQueueSend(gb.recv_cb_queue, &recv_data, 0);

}

ens_status_t get_recieved_data(ens_recv_cb_data_t *recv_data){

    if(recv_data == NULL){
        return ENS_NULL_ERR;
    }

    if(xQueueReceive(gb.recv_cb_queue, recv_data, 0) != pdTRUE){
        return ENS_QUEUE_ERR;
    }

    return ENS_OK;
}

ens_status_t send_data(uint8_t *dest_mac, uint8_t *data, int len){

    if(dest_mac == NULL || data == NULL || len == 0 || len > MAX_DATA_LENGTH){
        return ENS_NULL_ERR;
    }

    esp_err_t err = esp_now_send(dest_mac, data, len);

    if(err != ESP_OK){
        return ENS_ESP_NOW_ERR;
    }

    return ENS_OK;
}

ens_status_t get_send_data_ack(ens_send_cb_data_t *send_data){

    if(send_data == NULL){
        return ENS_NULL_ERR;
    }

    if(xQueueReceive(gb.send_cb_queue, send_data, 0) != pdTRUE){
        return ENS_QUEUE_ERR;
    }

    return ENS_OK;
}