#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "esp_sleep.h"
#include "esp_log.h"

#include "ens_task.h"

#define TAG "ENS_TASK"

static struct {
    bool disable_sleep;
    QueueHandle_t rx_queue;
    SemaphoreHandle_t sleep_lock;
    SemaphoreHandle_t interval_mutex;
    uint8_t tx_buffer[MAX_DATA_LENGTH];
    size_t tx_data_size;

    on_data_rx _on_data_rx;
    data_to_transmit _data_to_transmit;
    TaskHandle_t sub_task_handle;
} gb;

static void sub_task(void *pvParameters);
static void on_recive(ens_recv_cb_data_t *recv_data);
static void manage_recieved_data(ens_recv_cb_data_t *recv_data);

ens_status_t ens_init(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]){

    if(init_struct->_data_to_transmit == NULL){
        return ENS_NULL_ERR;
    }

    gb.disable_sleep = init_struct->disable_sleep;

    gb._data_to_transmit = init_struct->_data_to_transmit;

    if(init_struct->_on_data_rx == NULL){

        gb.rx_queue = xQueueCreate(1, sizeof(recv_cb_cmd_t));

        if(gb.rx_queue == NULL){
            return ENS_QUEUE_ERR;
        }
    }
    else{

        gb._on_data_rx = init_struct->_on_data_rx;
        gb.rx_queue = NULL;
    }

    gb.sleep_lock = xSemaphoreCreateMutex();
    gb.interval_mutex = xSemaphoreCreateMutex();

    if(gb.sleep_lock == NULL || gb.interval_mutex == NULL){

        return ENS_MUTEX_ERR;
    }

    if(_esp_now_init(init_struct, transmit_periods) != ENS_OK){
        ESP_LOGE(TAG, "ESP-NOW initialization failed");
        return ENS_ESP_NOW_ERR;
    }

    if(logic_init(init_struct, transmit_periods) != ENS_OK){
        ESP_LOGE(TAG, "ENS logic initialization failed");
        return ENS_QUEUE_ERR;
    }

    xTaskCreatePinnedToCore(sub_task, "ens_sub_task", 4096, NULL, 5, &gb.sub_task_handle, 0);

    return ENS_OK;

}

ens_status_t ens_deinit(void){

    if(gb.sub_task_handle != NULL){

        vTaskDelete(gb.sub_task_handle);
        gb.sub_task_handle = NULL;
    }

    if(gb.rx_queue != NULL){

        vQueueDelete(gb.rx_queue);
        gb.rx_queue = NULL;
    }

    if(gb.sleep_lock != NULL){

        vSemaphoreDelete(gb.sleep_lock);
        gb.sleep_lock = NULL;
    }

    if(gb.interval_mutex != NULL){

        vSemaphoreDelete(gb.interval_mutex);
        gb.interval_mutex = NULL;
    }

    return ENS_OK; 
}

ens_status_t ens_sleep_lock(void){
    return xSemaphoreTake(gb.sleep_lock, portMAX_DELAY) ==  pdTRUE ? ENS_OK : ENS_MUTEX_ERR;
}

ens_status_t ens_sleep_unlock(void){
    return xSemaphoreGive(gb.sleep_lock) == pdTRUE ? ENS_OK : ENS_MUTEX_ERR;
}

ens_status_t ens_enable_sleep(bool enable){
    
    gb.disable_sleep = !enable;

    return ENS_OK;
}

static void ens_go_to_sleep(uint64_t interval_ms){

    if(gb.disable_sleep == false){

        if(xSemaphoreTake(gb.sleep_lock, portMAX_DELAY) == pdTRUE){

            esp_go_to_sleep(interval_ms);
        }
    }
}

static void check_for_packet(){

    ens_recv_cb_data_t recv_data;

    if(get_recieved_data(&recv_data) == ENS_OK){

        on_recive(&recv_data);
    }
}

static void on_recive(ens_recv_cb_data_t *recv_data){

    if(recv_data == NULL || recv_data->data_size == 0 || recv_data->data_size > MAX_DATA_LENGTH || !is_from_mcb(recv_data->src_mac)){
        return;
    }

    manage_recieved_data(recv_data);

}

static void check_for_send_ack(){
    
    ens_send_cb_data_t send_data;
    
    if(get_send_data_ack(&send_data) == ENS_OK){

        on_send(&send_data);
    }
}

static void send_packet(){

    if(gb._data_to_transmit != NULL){

        gb._data_to_transmit(gb.tx_buffer, MAX_DATA_LENGTH, &gb.tx_data_size);
    }

    if(gb.tx_data_size <= MAX_DATA_LENGTH && gb.tx_data_size > 0){
        
        send_data(get_obc_mac_address(), gb.tx_buffer, gb.tx_data_size);
    }
}



static void manage_recieved_data(ens_recv_cb_data_t *recv_data){

    if(is_broadcast(recv_data->dest_mac)){

        set_mission_state(recv_data); 
        // ESP_LOGI(TAG, "Received broadcast message from %02x:%02x:%02x:%02x:%02x:%02x",
        //     recv_data->src_mac[0], recv_data->src_mac[1], recv_data->src_mac[2],
        //     recv_data->src_mac[3], recv_data->src_mac[4], recv_data->src_mac[5]); 
        // ESP_LOGI(TAG, "Command: %d, Payload: %d", recv_data->cmd.cmd.command, recv_data->cmd.cmd.payload);
    }
    else if(is_cmd_msg(recv_data)){

        // ESP_LOGI(TAG, "Command: %d, Payload: %d from %02x:%02x:%02x:%02x:%02x:%02x",
        //     recv_data->cmd.cmd.command, recv_data->cmd.cmd.payload,
        //     recv_data->src_mac[0], recv_data->src_mac[1], recv_data->src_mac[2],
        //     recv_data->src_mac[3], recv_data->src_mac[4], recv_data->src_mac[5]);

        if(gb._on_data_rx == NULL){

            xQueueSend(gb.rx_queue, &recv_data->cmd, 0);
        }
        else{
            
            gb._on_data_rx(recv_data->cmd.raw, recv_data->data_size);
        }
    }

}

static void sub_task(void *pvParameters){

    while(1){

        check_for_send_ack();

        if(can_send_data()){

            send_packet();

            reset_transmit_timer();
        }
            
        check_for_packet();

        uint64_t temp_interval_ms = get_interval_ms();

        if(is_sleep_interval(temp_interval_ms)){

            ens_go_to_sleep(temp_interval_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}