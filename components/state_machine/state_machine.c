///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 14.02.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "state_machine.h"

#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "string.h"
#include "stdbool.h"
#include "esp_log.h"

#define TAG "STATE_MACHINE"

///===-----------------------------------------------------------------------------------------===//

static struct {
    state_config_t *states;
    uint8_t states_quantity;
    uint8_t previous_state;
    uint8_t current_state;

    end_looped_function end_function;
    uint32_t end_fct_frq_ms;

    TaskHandle_t state_task;
    SemaphoreHandle_t current_state_mutex;
} sm;

static bool check_state_id(state_config_t *states, uint8_t states_quantity) {
    assert(states != NULL);

    for (int i = 0; i < states_quantity; ++i) {
        // id is less than states_quantity
        if (states[i].id >= states_quantity) {
            return false;
        }

        // ids are unique
        for (int j = i + 1; j < states_quantity; ++j) {
            if (states[i].id == states[j].id) {
                return false;
            }
        }
    }

    return true;
}

static bool state_machine_check_new_state(state_id new_state) {
    assert(sm.current_state < sm.states_quantity);
    if (sm.current_state >= sm.states_quantity) {
        return false;
    }

    if (sm.state_task == NULL) {
        return false;
    }

    if (sm.states[sm.current_state + 1].id != new_state) {
        return false;
    }

    return true;
}

void state_machine_init() {
    sm.states = NULL;
    sm.states_quantity = 0;
    sm.previous_state = 0;
    sm.current_state = 0;
    sm.end_function = NULL;
    sm.end_fct_frq_ms = 0;

    sm.state_task = NULL;
    sm.current_state_mutex = xSemaphoreCreateMutex();
}

static void state_machine_loop(void *arg) {
    bool sm_completed = false;
    uint8_t state = 0;
    while (1) {
        assert(sm.current_state < sm.states_quantity);
        if (ulTaskNotifyTake(pdTRUE, 0)) {
            xSemaphoreTake(sm.current_state_mutex, portMAX_DELAY);
            state = sm.current_state;
            xSemaphoreGive(sm.current_state_mutex);

            if (sm.states[sm.current_state].callback != NULL) {
                sm.states[sm.current_state].callback(sm.states[sm.current_state].arg);
            }

            if (state + 1 >= sm.states_quantity) {
                ESP_LOGI(TAG, "End function enable");
                sm_completed = true;
            }

        }

        if (sm_completed == true) {  // end function
            if (sm.end_function != NULL) {
                sm.end_function();
                vTaskDelay(sm.end_fct_frq_ms / portTICK_PERIOD_MS - 10);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

///===-----------------------------------------------------------------------------------------===//

state_machine_status_t state_machine_set_states(state_config_t *states, uint8_t states_quantity) {
    assert(states != NULL);
    if (states == NULL) {
        return STATE_MACHINE_STATES_ERROR;
    }

    if (states_quantity == 0) {
        return STATE_MACHINE_STATES_NUMBER_ERROR;
    }

    if (check_state_id(states, states_quantity) == false) {
        return STATE_MACHINE_STATES_ERROR;
    }

    sm.states = states;
    sm.states_quantity = states_quantity;
    return STATE_MACHINE_OK;
}

state_machine_status_t state_machine_set_end_function(end_looped_function function, uint32_t freq_ms) {
    assert(function != NULL);
    if (function == NULL) {
        return STATE_MACHINE_NULL_FUNCTION;
    }

    sm.end_function = function;
    sm.end_fct_frq_ms = freq_ms;

    return STATE_MACHINE_OK;
}

state_machine_status_t state_machine_change_state(state_id new_state) {
    if (xSemaphoreTake(sm.current_state_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return STATE_MACHINE_STATE_CHANGE_ERROR;
    }

    if (state_machine_check_new_state(new_state)) {
        sm.previous_state = sm.current_state;
        sm.current_state += 1;
        ESP_LOGI(TAG, "Changing state from %d to %d", sm.current_state - 1, sm.current_state);
        xSemaphoreGive(sm.current_state_mutex);
        xTaskNotifyGive(sm.state_task);
        return STATE_MACHINE_OK;
    }
    xSemaphoreGive(sm.current_state_mutex);

    return STATE_MACHINE_STATE_CHANGE_ERROR;
}

state_machine_status_t state_machine_force_change_state(state_id new_state) {
    if (new_state >= sm.states_quantity) {
        return STATE_MACHINE_STATE_CHANGE_ERROR;
    }

    ESP_LOGI(TAG, "Changing state from %d to %d", sm.current_state, new_state);
    xSemaphoreTake(sm.current_state_mutex, portMAX_DELAY);
    sm.previous_state = sm.current_state;
    sm.current_state = new_state;
    xSemaphoreGive(sm.current_state_mutex);
    xTaskNotifyGive(sm.state_task);
    return STATE_MACHINE_OK;
}

state_machine_status_t state_machine_change_to_previous_state(bool run_callback) {
    ESP_LOGI(TAG, "Changing state from %d to %d", sm.current_state, sm.previous_state);
    xSemaphoreTake(sm.current_state_mutex, portMAX_DELAY);
    sm.current_state = sm.previous_state;
    xSemaphoreGive(sm.current_state_mutex);

    if (run_callback == true) {
        xTaskNotifyGive(sm.state_task);
    }

    return STATE_MACHINE_OK;
}

state_machine_status_t state_machine_change_state_ISR(state_id new_state) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreTake(sm.current_state_mutex, portMAX_DELAY);
    if (state_machine_check_new_state(new_state)) {
        sm.current_state += 1;
        vTaskNotifyGiveFromISR(sm.state_task, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return STATE_MACHINE_OK;
    }
    xSemaphoreGive(sm.current_state_mutex);

    return STATE_MACHINE_STATE_CHANGE_ERROR;
}

state_id state_machine_get_current_state(void) {
    state_id current_state;
    xSemaphoreTake(sm.current_state_mutex, portMAX_DELAY);
    current_state = sm.current_state;
    xSemaphoreGive(sm.current_state_mutex);

    return current_state;
}

state_id state_machine_get_previous_state(void) {
    state_id previous_state;
    xSemaphoreTake(sm.current_state_mutex, portMAX_DELAY);
    previous_state = sm.previous_state;
    xSemaphoreGive(sm.current_state_mutex);

    return previous_state;
}

state_id state_machine_get_current_state_ISR(void) {
    state_id current_state;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreTakeFromISR(sm.current_state_mutex, &xHigherPriorityTaskWoken);
    current_state = sm.current_state;
    xSemaphoreGiveFromISR(sm.current_state_mutex, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return current_state;
}

state_machine_status_t state_machine_run(state_machine_task_cfg_t *cfg) {
    if (sm.states == NULL) {
        return STATE_MACHINE_RUN_ERROR;
    }

    xTaskCreatePinnedToCore(
        state_machine_loop,
        "state_machine",
        cfg->stack_depth,
        NULL,
        cfg->priority,
        &sm.state_task,
        cfg->core_id);

    assert(sm.state_task != NULL);
    if (sm.state_task == NULL) {
        return STATE_MACHINE_RUN_ERROR;
    }

    xTaskNotifyGive(sm.state_task);
    return STATE_MACHINE_OK;
}

state_machine_status_t state_machine_destroy(void) {
    if (sm.state_task != NULL) {
        vTaskDelete(sm.state_task);
    }

    if (sm.current_state_mutex != NULL) {
        vSemaphoreDelete(sm.current_state_mutex);
    }

    sm.states = NULL;
    sm.states_quantity = 0;
    sm.current_state = 0;
    sm.end_function = NULL;
    sm.end_fct_frq_ms = 0;
    sm.state_task = NULL;
    sm.current_state_mutex = NULL;

    return STATE_MACHINE_OK;
}