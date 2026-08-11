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
#include "board_data.h"

#include "system_timer.h"

#include "esp_log.h"

#define TAG "LORA_TASK"

#define LORA_TASK_STACK_SIZE 8192
#define LORA_TASK_PRIORITY 4
#define LORA_TASK_CORE 1

#define TELEMETRY_SCALE_BATTERY 100
#define TELEMETRY_SCALE_PRESSURE 100
#define TELEMETRY_SCALE_TEMP 100
#define TELEMETRY_SCALE_WEIGHT 100
#define TELEMETRY_SCALE_THRUST 100

#define SCALE_TO_U32(v, s) ((uint32_t)((v) * (s) + 0.5f))
#define SCALE_TO_I32(v, s) ((int32_t)((v) * (s) + ((v) >= 0 ? 0.5f : -0.5f)))

#define TANWA_FLAG_CAN_WEIGHTS 0
#define TANWA_FLAG_CAN_UTILITY 1
#define TANWA_FLAG_CAN_SENSOR 2
#define TANWA_FLAG_CAN_POWER 3
#define TANWA_FLAG_CAN_SOLENOID 4
#define TANWA_FLAG_IGNITER1_CONT 5
#define TANWA_FLAG_IGNITER2_CONT 6
#define TANWA_FLAG_SOFT_ARM 7
#define TANWA_FLAG_ABORT_BUTTON 8
#define TANWA_FLAG_FILL_N2O 9
#define TANWA_FLAG_DEPR_N2O 10
#define TANWA_FLAG_FILL_N2 11
#define TANWA_FLAG_DEPR_N2 12
#define TANWA_FLAG_DROID_N2O 13
#define TANWA_FLAG_DROID_N2 14
#define TANWA_FLAG_HEATING_TANK  15
#define TANWA_FLAG_HEATING_VALVE 16

#define SET_FLAG(word, bit, cond) ((word) |= ((cond) ? (1UL << (bit)) : 0UL))
#define FRAME_SET(field, v) do { (field).is_present = true; (field).value = (v); } while (0) // do-while(0) intentional: makes the multi-statement macro a single statement

static uint8_t message_buffer[2048];
static StaticMessageBuffer_t message_buffer_static;

static struct {
    lora_struct_t *lora;
    lora_task_process_rx_packet process_packet_fnc;
    lora_task_get_tx_packet get_tx_packet_fnc;
    lora_state_t lora_state;
    uint8_t tx_buffer[512]; // Buffer for LoRa transmission
    size_t tx_buffer_size;
    MessageBufferHandle_t rx_queue;

    TimerHandle_t receive_window_timer;
    TaskHandle_t task;
    uint32_t receive_window_period;
    TaskHandle_t receive_task;

    const uint16_t *state_periods;
    size_t state_periods_count;
    int last_period_state;
} gb;

static size_t lora_packet(uint8_t* buffer, size_t buffer_size);
static void lora_process(uint8_t* packet, size_t packet_size);
static void transmit_packet(void);

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
    if (gb.task != NULL) {
        vTaskNotifyGiveFromISR(gb.task, &higher_priority_task_woken);
    }
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

static void lora_change_state_to_transmit(void) {
    if (gb.lora_state == LORA_TRANSMIT) return;
    lora_map_d0_interrupt(&lora, LORA_IRQ_D0_TXDONE);
    gb.lora_state = LORA_TRANSMIT;
}

void lora_set_state_periods(const uint16_t *periods_ms, size_t count) {
    gb.state_periods = periods_ms;
    gb.state_periods_count = count;
    gb.last_period_state = -1;
}

static void apply_state_period(void) {
    if (gb.state_periods == NULL) return;

    state_t state = state_machine_get_current_state();
    if ((int)state == gb.last_period_state || (size_t)state >= gb.state_periods_count) return;

    if (lora_change_period(gb.state_periods[state])) {
        gb.last_period_state = (int)state;
        ESP_LOGI(TAG, "Telemetry period %u ms (state %d)", gb.state_periods[state], (int)state);
    } else {
        ESP_LOGW(TAG, "Failed to set telemetry period %u ms (state %d)", gb.state_periods[state], (int)state);
    }
}

void turn_on_receive_window_timer(void) {
    apply_state_period();

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

static size_t   on_lora_receive(uint8_t *rx_buffer, size_t buffer_len) {
    size_t len = 0;
    if (lora_received(gb.lora) == LORA_OK) {
        len = lora_receive_packet(gb.lora, rx_buffer, buffer_len);
        rx_buffer[len] = '\0';
        ESP_LOGI(TAG, "Received %d bytes", len);
        xMessageBufferSend(gb.rx_queue, rx_buffer, len, 100);
        ESP_LOGD(TAG, "Received, len %d", len);
    }
    return len;
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
    ESP_LOGI(TAG, "Received packet");

    if (check_prefix(packet, packet_size) == false) {
        ESP_LOGE(TAG, "LoRa invalid prefix");
        return;
    }


    uint8_t prefix_size = sizeof(PACKET_PREFIX) - 1;
    // if (calculate_checksum(packet + prefix_size, packet_size - prefix_size - 1) != packet[packet_size - 1]) {
    //     ESP_LOGE(TAG, "Invalid checksum");
    //     return;
    // }

    struct obc_lo_ra_frame_t* received = obc_lo_ra_frame_new(&lora_api.workspace, sizeof(lora_api.workspace));

    size_t decoded_size = obc_lo_ra_frame_decode(received, packet + prefix_size, packet_size - prefix_size);

    if (received->frame != obc_lo_ra_frame_frame_app_frame_e && received->frame != obc_lo_ra_frame_frame_mcb_frame_e) {
        ESP_LOGE(TAG, "Received frame is not an app or mcb frame");
        return;
    }

    if (decoded_size == 0) {
        ESP_LOGE(TAG, "Unable to decode received package");
        return;
    }

    if (received->frame == obc_lo_ra_frame_frame_app_frame_e) {
        if (!received->app_frame_p->lora_dev_id.is_present || !received->app_frame_p->command.is_present) {
            ESP_LOGE(TAG, "Incomplete LoRa command (missing id/command)");
            return;
        }

        int32_t payload = received->app_frame_p->payload.is_present ? received->app_frame_p->payload.value : 0;

        if (lora_command_parsing(received->app_frame_p->lora_dev_id.value, received->app_frame_p->command.value, payload) == false) {
            ESP_LOGE(TAG, "Unable to process command :C");
            return;
        }
    }

    if (received->frame == obc_lo_ra_frame_frame_mcb_frame_e) {
        // w tym przypadku nie musimy dokonywać żadnego parsowanie, jedynie potrzebujemy informacji o tym że ramka doszła
        transmit_packet();
        lora_set_receive_mode(&lora);
    }
}

static size_t add_prefix(uint8_t* buffer, size_t size) {
    if (size < 6) {
        return 0;
    }

    memcpy(buffer, PACKET_PREFIX, sizeof(PACKET_PREFIX) - 1);

    return sizeof(PACKET_PREFIX) - 1;
}

void create_porotobuf_data_frame(struct obc_tanwa_frame_t *frame) {
    //ESP_LOGI(TAG, "Creating LoRa data frame");
    if (frame == NULL) {
        ESP_LOGE(TAG, "Frame is NULL");
        return;
    }

    tanwa_data_t tanwa_data = tanwa_data_read();

    FRAME_SET(frame->tanwa_battery, SCALE_TO_U32(tanwa_data.can_power_data.VOLTAGE_24V_SYS, TELEMETRY_SCALE_BATTERY));
    FRAME_SET(frame->tanwa_state, tanwa_data.state);
    FRAME_SET(frame->tanwa_thrust, SCALE_TO_I32(tanwa_data.can_weight_data.ads1_weight2, TELEMETRY_SCALE_THRUST));
    FRAME_SET(frame->tanwa_tank_weight, SCALE_TO_U32(tanwa_data.can_weight_data.ads1_weight3, TELEMETRY_SCALE_WEIGHT));
    FRAME_SET(frame->tanwa_temp_post_n2o_fill, SCALE_TO_I32(tanwa_data.can_sensor_temp_data.temperature1, TELEMETRY_SCALE_TEMP));
    FRAME_SET(frame->tanwa_temp_filling_wall, SCALE_TO_I32(tanwa_data.can_sensor_temp_data.temperature2, TELEMETRY_SCALE_TEMP));

    FRAME_SET(frame->tanwa_post_fill_n2o_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure8, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_cutoff_n2o_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure5, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_droid_n2o_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure3, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_pre_reg_n2_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure7, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_post_reg_n2_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure6, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_post_fill_n2_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure1, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_droid_n2_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure2, TELEMETRY_SCALE_PRESSURE));
    FRAME_SET(frame->tanwa_comb_chamber_pres, SCALE_TO_I32(tanwa_data.can_sensor_pressure_data.pressure4, TELEMETRY_SCALE_PRESSURE));

    uint32_t flags = 0;
    SET_FLAG(flags, TANWA_FLAG_CAN_WEIGHTS, tanwa_data.can_connected_slaves.weights);
    SET_FLAG(flags, TANWA_FLAG_CAN_UTILITY, tanwa_data.can_connected_slaves.utility);
    SET_FLAG(flags, TANWA_FLAG_CAN_SENSOR, tanwa_data.can_connected_slaves.sensor);
    SET_FLAG(flags, TANWA_FLAG_CAN_POWER, tanwa_data.can_connected_slaves.power);
    SET_FLAG(flags, TANWA_FLAG_CAN_SOLENOID, tanwa_data.can_connected_slaves.solenoid);
    SET_FLAG(flags, TANWA_FLAG_IGNITER1_CONT, tanwa_data.com_data.igniter_cont_1);
    SET_FLAG(flags, TANWA_FLAG_IGNITER2_CONT, tanwa_data.com_data.igniter_cont_2);
    SET_FLAG(flags, TANWA_FLAG_SOFT_ARM, tanwa_data.com_data.arm_state);
    SET_FLAG(flags, TANWA_FLAG_ABORT_BUTTON, tanwa_data.com_data.abort_button);
    SET_FLAG(flags, TANWA_FLAG_FILL_N2O, tanwa_data.can_solenoid_data.state_sol1);
    SET_FLAG(flags, TANWA_FLAG_DEPR_N2O, tanwa_data.can_solenoid_data.state_sol2);
    SET_FLAG(flags, TANWA_FLAG_FILL_N2, tanwa_data.can_solenoid_data.state_sol3);
    SET_FLAG(flags, TANWA_FLAG_DEPR_N2, tanwa_data.can_solenoid_data.state_sol4);
    SET_FLAG(flags, TANWA_FLAG_DROID_N2O, tanwa_data.can_solenoid_data.state_sol5);
    SET_FLAG(flags, TANWA_FLAG_DROID_N2, tanwa_data.can_solenoid_data.state_sol6);
    SET_FLAG(flags, TANWA_FLAG_HEATING_TANK, tanwa_data.com_data.relay_state3);
    SET_FLAG(flags, TANWA_FLAG_HEATING_VALVE, tanwa_data.com_data.relay_state2);
    FRAME_SET(frame->tanwa_flags, flags);
}

static size_t lora_create_data_packet(uint8_t* buffer, size_t size) {
    
    //ESP_LOGI(TAG, "Creating LoRa data packet");
    lora_api.frame = obc_lo_ra_frame_new(lora_api.workspace, sizeof(lora_api.workspace));
    lora_api.frame->frame = obc_lo_ra_frame_frame_tanwa_frame_e;
    struct obc_tanwa_frame_t tanwa_frame = {0};
    lora_api.frame->tanwa_frame_p = &tanwa_frame;

    //ESP_LOGI(TAG, "LoRa frame created");
    create_porotobuf_data_frame(lora_api.frame->tanwa_frame_p);

    // ESP_LOGI(TAG, "FRAME:");
    // for (int i = 0; i < sizeof(struct lo_ra_frame_t); ++i) {
    //     ESP_LOGI(TAG, "%d: %d", i, ((uint8_t*)&lora_api.frame)[i]);
    // }


    uint8_t data_size = 0;
    uint8_t prefix_size = 0;
    prefix_size = add_prefix(buffer, size);
    data_size = obc_lo_ra_frame_encode(lora_api.frame, buffer + prefix_size, size - prefix_size);

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
    ESP_LOGI(TAG, "initialize_lora: lora global=%p, lora_api.lora=%p", (void*)&lora, (void*)lora_api.lora);
    
    if(lora_task_init(&lora_api) == false) {
        ESP_LOGE(TAG, "Failed to initialize LoRa task");
        return false;
    }
    return true;
}

static void lora_receive_task(void *arg) {
    uint8_t msg[255];

    while (1) {
        size_t msg_size = xMessageBufferReceive(gb.rx_queue, &msg, 255, portMAX_DELAY);
        ESP_LOGI(TAG, "Received %d", msg_size);
        gb.process_packet_fnc(msg, msg_size);
    }
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
    gb.receive_window_period = cfg->transmiting_period;
    gb.rx_queue = xMessageBufferCreateStatic(2048, message_buffer, &message_buffer_static);
    gb.tx_buffer_size = 512;
    // Ensure the gb.lora instance is initialized with the global `lora`
    // so that function pointers and configuration are available to the task.
     /* Use pointer to the global initialized `lora` instance so all
         callers operate on the same object (avoids stale copies). */
     gb.lora = &lora;

    memset(lora_api.workspace, 0, sizeof(lora_api.workspace));
    lora_api.frame = obc_lo_ra_frame_new(lora_api.workspace, sizeof(lora_api.workspace));

    lora_init(&lora);
    lora_set_frequency(&lora, cfg->frequency_khz * 1e3);
    lora_set_bandwidth(&lora, LORA_TASK_BANDWIDTH);
    lora_set_tx_power(&lora, LORA_TASK_TX_POWER);
    lora_set_spreading_factor(&lora, LORA_TASK_SPREADING_FACTOR);   
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
    // turn_on_receive_window_timer();

    ESP_LOGI(TAG, "Reading LoRa registers");
    int16_t read_val_one = lora_read_reg(&lora, 0x0d);
    int16_t read_val_two = lora_read_reg(&lora, 0x0c);
    ESP_LOGI(TAG, "lora_task_init: gb=%p, gb.lora=%p, global lora=%p, lora._spi_transmit=%p", (void*)&gb, (void*)gb.lora, (void*)&lora, (void*)lora._spi_transmit);
    ESP_LOGI(TAG, "LORA_READ: %04x, %04x", read_val_one, read_val_two);

    xTaskCreatePinnedToCore(lora_task, "LoRa task", LORA_TASK_STACK_SIZE, NULL, LORA_TASK_PRIORITY,
                            &gb.task, LORA_TASK_CORE);

    xTaskCreatePinnedToCore(lora_receive_task, "Receive task", LORA_TASK_STACK_SIZE, NULL, LORA_TASK_PRIORITY - 1,
                            &gb.receive_task, LORA_TASK_CORE);

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

bool lora_change_period(uint32_t period_ms) {
    if (period_ms < LORA_TASK_MIN_TRANSMIT_MS || period_ms > LORA_TASK_MAX_TRANSMIT_MS) return false;
    if (gb.receive_window_timer == NULL) return false;

    if (xTimerChangePeriod(gb.receive_window_timer, pdMS_TO_TICKS(period_ms), portMAX_DELAY) != pdPASS) return false;

    lora_api.transmiting_period = period_ms;
    return true;
}

static void transmit_packet(void) {
    size_t size = 0;
    if (gb.get_tx_packet_fnc != NULL) {
        size = gb.get_tx_packet_fnc(gb.tx_buffer, gb.tx_buffer_size);
    }

    if (size > 0 && size <= gb.tx_buffer_size) {
        lora_send_packet(&lora, gb.tx_buffer, (int16_t)size);
        ESP_LOGW(TAG, "Sending packet");
    }
}

static void on_lora_transmit(void) {
    lora_change_state_to_receive();
    turn_on_receive_window_timer();
}

void lora_task(void *arg) {
    uint8_t rx_buffer[512];
    size_t rx_packet_size = 0;

    while (1) {
        //on transmit
        if (gb.lora_state == LORA_TRANSMIT) {
            ESP_LOGI(TAG, "ON transmit");
            lora_change_state_to_receive();

            // on receive
        } else {
            ESP_LOGI(TAG, "ON receive");

            TickType_t start_tick = xTaskGetTickCount();
            TickType_t delay_ticks = pdMS_TO_TICKS(gb.receive_window_period);

            // Wyczyść ewentualne zaległe powiadomienia z przerwań
            ulTaskNotifyTake(pdTRUE, 0);

            while (1) {
                TickType_t current_tick = xTaskGetTickCount();
                if (current_tick - start_tick >= delay_ticks) {
                    break;  // Czas okienka minął
                }
                TickType_t remaining_ticks = delay_ticks - (current_tick - start_tick);

                // Czekamy na przerwanie (RXDONE) przez pozostały czas
                if (ulTaskNotifyTake(pdTRUE, remaining_ticks) == pdTRUE) {
                    // Wybudzono nas przerwaniem - jest ramka!
                    rx_packet_size = on_lora_receive(rx_buffer, sizeof(rx_buffer));

                    if (rx_packet_size > 0) {
                        // lora_receive_packet przełącza układ w tryb Idle,
                        // więc musimy z powrotem przełączyć go na tryb odbioru ciągłego
                        // (Continuous RX)
                        lora_set_receive_mode(gb.lora);
                    }
                }
            }
            if (state_machine_get_current_state() >= LIFT_OFF) {
                lora_change_state_to_transmit();
                transmit_packet();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}