// Copyright 2022 PWrInSpace, Kuba
#include "lora_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "lora.pb-c.h"
#include "cmd_commands.h"

#include "board_config.h"
#include "mcu_gpio_config.h"
#include "mcu_spi_config.h"
#include "mcu_misc_config.h"
#include "timers_config.h"
#include "state_machine_config.h"
#include "state_machine.h"

#include "system_timer.h"

#include "esp_log.h"

#define TAG "LORA_TASK"

#define LORA_TASK_STACK_SIZE 8192
#define LORA_TASK_PRIORITY 2
#define LORA_TASK_CORE 0

static struct {
    lora_struct_t lora;
    lora_task_process_rx_packet process_packet_fnc;
    lora_task_get_tx_packet get_tx_packet_fnc;
    lora_state_t lora_state;
    uint8_t tx_buffer[256];
    size_t tx_buffer_size;

    TimerHandle_t receive_window_timer;
    TaskHandle_t task;

} gb;

static size_t lora_packet(uint8_t* buffer, size_t buffer_size);
static void lora_process(uint8_t* packet, size_t packet_size); 

lora_struct_t lora = {
    ._spi_transmit = _lora_spi_transmit,
    ._delay = _lora_delay_ms,
    ._gpio_set_level = _lora_gpio_set_level,
    .log = _lora_log,
    .rst_gpio_num = CONFIG_LORA_RS,
    .cs_gpio_num = CONFIG_LORA_CS,
    .d0_gpio_num = CONFIG_LORA_D0,
    .implicit_header = 0,
    .frequency = 0,
};
lora_api_config_t lora_api = {
    .lora = &lora,
    .process_rx_packet_fnc = lora_process,
    .get_tx_packet_fnc = lora_packet,
};

static bool wait_until_irq(void) {
    return ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE ? true : false;
}



void IRAM_ATTR lora_task_irq_notify(void *arg) {

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(gb.task, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void notify_end_of_rx_window(void) { 
    xTaskNotifyGive(gb.task);
    //ESP_LOGI(TAG, "END OF WINDOW");
}

static void on_receive_window_timer(TimerHandle_t timer) { notify_end_of_rx_window(); }

static void lora_change_state_to_receive() {
    ESP_LOGD(TAG, "Changing state to receive");
    if (gb.lora_state == LORA_RECEIVE) {
        return;
    }

    lora_map_d0_interrupt(&lora, LORA_IRQ_D0_RXDONE);
    lora_set_receive_mode(&lora);
    gb.lora_state = LORA_RECEIVE;
}

static void lora_change_state_to_transmit() {
    ESP_LOGD(TAG, "Changing state to transmit");
    if (gb.lora_state == LORA_TRANSMIT) {
        return;
    }

    gb.lora_state = LORA_TRANSMIT;
}

void turn_on_receive_window_timer(void) {
    if (xTimerIsTimerActive(gb.receive_window_timer) == pdTRUE) {
        xTimerReset(gb.receive_window_timer, portMAX_DELAY);
        //ESP_LOGE(TAG, "TIMER IS ACTIVE");
        return;
    }
    xTimerStart(gb.receive_window_timer, portMAX_DELAY);
}

void turn_of_receive_window_timer(void) {
    if (xTimerIsTimerActive(gb.receive_window_timer) == pdTRUE) {
        xTimerStop(gb.receive_window_timer, portMAX_DELAY);
    }
}

static size_t on_lora_receive(uint8_t *rx_buffer, size_t buffer_len) {
    size_t len = 0;
    // if (lora_received(&gb.lora) == LORA_OK) {
    //     len = lora_receive_packet(&gb.lora, rx_buffer, buffer_len);
    //     rx_buffer[len] = '\0';
    //     ESP_LOGD(TAG, "Received %s, len %d", rx_buffer, len);
    //     lora_map_d0_interrupt(&gb.lora, LORA_IRQ_D0_RXDONE);
    //     lora_set_receive_mode(&gb.lora);
    // }
    turn_of_receive_window_timer();

    //ESP_LOGI(TAG, "Waiting for D0 RXDONE interrupt");
    
    lora_map_d0_interrupt(&lora, LORA_IRQ_D0_TXDONE);
    if(lora_received(&lora) == LORA_OK) {
        len = lora_receive_packet(&lora, rx_buffer, buffer_len);
        rx_buffer[len] = '\0';
        ESP_LOGD(TAG, "Received %s, len %d", rx_buffer, len);
    }
    return len;
}

static void transmint_packet(void) {
    if (lora_api.get_tx_packet_fnc == NULL) {
        //ESP_LOGI(TAG, "No get_tx_packet_fnc function set");
        return;
    }

    gb.tx_buffer_size = lora_api.get_tx_packet_fnc(gb.tx_buffer, sizeof(gb.tx_buffer));
    lora_send_packet(&lora, gb.tx_buffer, gb.tx_buffer_size);
}

static void on_lora_transmit() {
    lora_change_state_to_receive();
    turn_of_receive_window_timer();
    turn_on_receive_window_timer();
}

static bool check_prefix(uint8_t* packet, size_t packet_size) {
    if (packet_size < sizeof(PACKET_PREFIX)) {
        return false;
    }

    uint8_t prefix[] = PACKET_PREFIX;
    for (int i = 0; i < sizeof(PACKET_PREFIX) - 1; ++i) {
        if (packet[i] != prefix[i]) {
            return false;
        }
    }

    return true;
}

static uint8_t calculate_checksum(uint8_t* buffer, size_t size) {
    uint8_t sum = 0;
    for (size_t i = 0; i < size; ++i) {
        sum += buffer[i];
    }

    return sum;
}

static void lora_process(uint8_t* packet, size_t packet_size) {
    if (packet_size > 40) {
        ESP_LOGI(TAG, "Recevied packet is too big");
        // errors_set(ERROR_TYPE_LAST_EXCEPTION, ERROR_EXCP_LORA_DECODE, 100);
        return;
    }

    if (check_prefix(packet, packet_size) == false) {
        ESP_LOGE(TAG, "LoRa invalid prefix");
        return;
    }


    uint8_t prefix_size = sizeof(PACKET_PREFIX) - 1;
    if (calculate_checksum(packet + prefix_size, packet_size - prefix_size - 1) != packet[packet_size - 1]) {
        ESP_LOGE(TAG, "Invalid checksum");
        return;
    }

    ESP_LOGI(TAG, "Received packet: %s", packet + prefix_size);

    struct lo_ra_command_t* received = lo_ra_command_new(&lora_api.workspace, sizeof(lora_api.workspace));
    size_t decoded_size = 0;
    decoded_size = lo_ra_command_decode(received, packet + prefix_size, packet_size - prefix_size - 1);
    if (decoded_size > 0) {
        ESP_LOGI(TAG, "Received LORA_ID %d, DEV_ID %d, COMMAND %d, PLD %d", received->lora_dev_id,
                 received->sys_dev_id, received->command, received->payload);
        // cmd_message_t received_command = cmd_create_message(received->command, received->payload);
        if(lora_command_parsing(received->lora_dev_id, received->command, received->payload) == false) {
            ESP_LOGE(TAG, "Unable to prcess command :C");
            return;
        }
    } else {
        ESP_LOGE(TAG, "Unable to decode received package");
    }
}
static size_t add_prefix(uint8_t* buffer, size_t size) {
    if (size < 6) {
        return 0;
    }

    memcpy(buffer, PACKET_PREFIX, sizeof(PACKET_PREFIX) - 1);

    return sizeof(PACKET_PREFIX) - 1;
}

// static size_t lora_create_settings_packet(uint8_t* buffer, size_t size) {
//     LoRaSettings frame = LO_RA_SETTINGS__INIT;
//     create_protobuf_settings_frame(&frame);

//     uint8_t data_size = 0;
//     uint8_t prefix_size = 0;
//     prefix_size = add_prefix(buffer, size);
//     data_size = lo_ra_settings__pack(&frame, buffer + prefix_size);

//     return prefix_size + data_size;
// }
#include "board_data.h"
void create_porotobuf_data_frame(struct lo_ra_frame_t *frame) {
    
    // tanwa_data_t tanwa_data = tanwa_data_read();   // fill struct with 0
    // // mcb
    // //frame->obc_state = data.mcb.state;
    // frame->tanwa_state = tanwa_data.state;
    // frame->uptime = 2137;
    // frame->pressure_injector_fuel = tanwa_data.com_data.pressure_1;
    // frame->pressure_injector_oxi = tanwa_data.com_data.pressure_2;
    // frame->pressure_combustion_chamber = tanwa_data.com_data.pressure_3;
    // frame->igniter_cont1 = tanwa_data.com_data.igniter_cont_1;
    // frame->igniter_cont2 = tanwa_data.com_data.igniter_cont_2;

    // //ESP_LOGI(TAG, "IGNITER CONT 1: %d", tanwa_data.com_data.igniter_cont_1);
    // //ESP_LOGI(TAG, "IGNITER CONT 2: %d", tanwa_data.com_data.igniter_cont_2);
    // frame->status_oxy= 1;
    // frame->status_fuel = 1;
    // frame->status_arm.is_present = true;
    // frame->status_arm.value = tanwa_data.com_data.arm_state;
    // ESP_LOGI(TAG, "ARM STATE: %d", tanwa_data.com_data.arm_state);

    // //ESP_LOGI(TAG, "ARM STATE: %d", tanwa_data.com_liquid_data.arm_state);
    // frame->tanwa_battery = tanwa_data.com_data.vbat;
    // frame->temp_injector = 0;
    // frame->temp_combustion_chamber = 0;
    // frame->temp_external_tank = 0;
    // // hx rck
    // frame->engine_thrust = 0;
    // frame->rocket_weight = 0;
    // frame->tank_weight = 0;

    // frame->engine_work_time = 696969699;
    // frame->pressure_fuel = tanwa_data.com_data.pressure_4;
    // frame->pressure_after_fill = 0;
    // frame->pressure_before_fill = 0;
    // frame->pressure_oxy = 0;
    // frame->status_fill = tanwa_data.com_data.solenoid_state_fill;

    // frame->status_depr = tanwa_data.com_data.solenoid_state_depr;
    //frame->status_vent = tanwa_data.com_data.solenoid_add_state;

}

static size_t lora_create_data_packet(uint8_t* buffer, size_t size) {

    struct lo_ra_frame_t *frame = lo_ra_frame_new(&lora_api.workspace, sizeof(lora_api.workspace));
    create_porotobuf_data_frame(frame);

    // ESP_LOGI(TAG, "FRAME:");
    // for (int i = 0; i < sizeof(frame); ++i) {
    //     ESP_LOGI(TAG, "%d: %d", i, ((uint8_t*)&frame)[i]);
    // }


    uint8_t data_size = 0;
    uint8_t prefix_size = 0;
    prefix_size = add_prefix(buffer, size);
    data_size = lo_ra_frame_encode(frame, buffer + prefix_size, sizeof(struct lo_ra_frame_t));

    ESP_LOGI(TAG, "LoRa frame packed size: %d", data_size);

    return prefix_size + data_size;
}

static size_t lora_packet(uint8_t* buffer, size_t buffer_size) 
{   size_t size = 0;
    size = lora_create_data_packet(buffer, buffer_size);
    //ESP_LOGI(TAG, "Sending LoRa frame -> size: %d", size);

    return size;
}


bool initialize_lora(uint32_t frequency_khz, uint32_t transmiting_period) {
    if(_lora_add_device() == false) {
        ESP_LOGE(TAG, "Failed to add LoRa device");
        return false;
    }
    if(_lora_gpio_attach_d0_isr(lora_task_irq_notify) == false) {
        ESP_LOGE(TAG, "Failed to attach D0 ISR");
        return false;
    }

    lora_api.frequency_khz = frequency_khz;
    lora_api.transmiting_period = transmiting_period;
    
    if(lora_task_init(&lora_api) == false) {
        ESP_LOGE(TAG, "Failed to initialize LoRa task");
        return false;
    }
    return true;
}

bool lora_task_init(lora_api_config_t *cfg) {
    assert(cfg != NULL);
    if (cfg == NULL) {
        return false;
    }

    if (cfg->process_rx_packet_fnc == NULL || cfg->get_tx_packet_fnc == NULL) {
        return false;
    }

    gb.process_packet_fnc = cfg->process_rx_packet_fnc;
    gb.get_tx_packet_fnc = cfg->get_tx_packet_fnc;
    //memcpy(&lora, &lora, sizeof(lora_struct_t));

    lora_init(&lora);
    lora_set_frequency(&lora, cfg->frequency_khz * 1e3);
    lora_set_bandwidth(&lora, LORA_TASK_BANDWIDTH);
    lora_map_d0_interrupt(&lora, LORA_IRQ_D0_RXDONE);
    //lora_set_receive_mode(&lora);

    if (LORA_TASK_CRC_ENABLE) {
        lora_enable_crc(&lora);
    } else {
        lora_disable_crc(&lora);
    }

    gb.receive_window_timer =
        xTimerCreate("Transmit timer", pdMS_TO_TICKS(cfg->transmiting_period), pdFALSE, NULL,
                     on_receive_window_timer);
    ESP_LOGD(TAG, "Starting timer");

    lora_change_state_to_receive();
    turn_on_receive_window_timer();

    ESP_LOGI(TAG, "Reading LoRa registers");
    int16_t read_val_one = lora_read_reg(&lora, 0x0d);
    int16_t read_val_two = lora_read_reg(&lora, 0x0c);
    ESP_LOGI(TAG, "LORA_READ: %04x, %04x", read_val_one, read_val_two);

    xTaskCreatePinnedToCore(lora_task, "LoRa task", LORA_TASK_STACK_SIZE, NULL, LORA_TASK_PRIORITY,
                            &gb.task, LORA_TASK_CORE);

    if (gb.task == NULL) {
        ESP_LOGI(TAG, "Failed to create LoRa task");
        return false;
    }

    return true;
}

bool lora_change_frequency(uint32_t frequency_khz) 
{
    if (frequency_khz < 4e5 || frequency_khz > 1e6) {
        return false;
    }
    if (lora_set_frequency(&lora, frequency_khz * 1000) != LORA_OK) {
        return false;
    }
    return true;
}


void lora_task(void *arg) 
{
    uint8_t rx_buffer[256];
    size_t rx_packet_size = 0;

    while (1) {
        if (wait_until_irq() == true) {
            // on transmit
            if (gb.lora_state == LORA_TRANSMIT) {
                //ESP_LOGI(TAG, "ON transmit");
                on_lora_transmit();
            // on receive
            } else {
                //ESP_LOGI(TAG, "ON receive");
                rx_packet_size = on_lora_receive(rx_buffer, sizeof(rx_buffer));
                //ESP_LOGI(TAG, "Received packet size: %d", rx_packet_size);
                if (rx_packet_size > 0 && lora_api.process_rx_packet_fnc != NULL) {
                    //ESP_LOGI(TAG, "*****************Processing packet");
                    lora_api.process_rx_packet_fnc(rx_buffer, rx_packet_size);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                lora_change_state_to_transmit();
                transmint_packet();
                // qucik fix
                turn_on_receive_window_timer();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// void lora_task(void *arg) 
// {
//     uint8_t rx_buffer[256];
//     size_t rx_packet_size = 0;

//     while (1) {
//         if (wait_until_irq() == true) {
//             // on transmit
//             if (gb.lora_state == LORA_TRANSMIT) {
//                 //ESP_LOGI(TAG, "ON transmit");
//                 on_lora_transmit();
//             // on receive
//             } else {
//                 //ESP_LOGI(TAG, "ON receive");
//                 rx_packet_size = on_lora_receive(rx_buffer, sizeof(rx_buffer));
//                 if (rx_packet_size > 0 && lora_api.process_rx_packet_fnc != NULL) {
//                     //ESP_LOGI(TAG, "*****************Processing packet");
//                     lora_api.process_rx_packet_fnc(rx_buffer, rx_packet_size);
//                     vTaskDelay(pdMS_TO_TICKS(100));
//                 }
//                 lora_change_state_to_transmit();
//                 //ESP_LOGI(TAG, "Transmitting packet");
//                 transmint_packet();
//                 // qucik fix
//                 turn_on_receive_window_timer();
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
