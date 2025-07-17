///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 30.05.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "board_data.h"

#include <memory.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#define TAG "TANWA_DATA"

static tanwa_data_t tanwa_data;
SemaphoreHandle_t tanwa_data_mutex;

bool tanwa_data_init(void) {
    tanwa_data_mutex = NULL;
    tanwa_data_mutex = xSemaphoreCreateMutex();
    if (tanwa_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    memset(&tanwa_data, 0, sizeof(tanwa_data_t));
    return true;
}

tanwa_data_t tanwa_data_read(void) {

    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        tanwa_data_t data_copy;
        memcpy(&data_copy, &tanwa_data, sizeof(tanwa_data_t));
        xSemaphoreGive(tanwa_data_mutex);
        return data_copy;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        tanwa_data_t empty_data = {0};
        return empty_data;
    }
}

com_data_t tanwa_data_read_com_data(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        com_data_t com_data = tanwa_data.com_data;
        xSemaphoreGive(tanwa_data_mutex);
        return com_data;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        com_data_t empty_com_data = {0};
        return empty_com_data;
    }
}



void tanwa_data_update_com_data(com_data_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.com_data, data, sizeof(com_data_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_connected_slaves_t tanwa_data_read_can_connected_slaves(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_connected_slaves_t slaves = tanwa_data.can_connected_slaves;
        xSemaphoreGive(tanwa_data_mutex);
        return slaves;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_connected_slaves_t empty_slaves = {0};
        return empty_slaves;
    }
}

void tanwa_data_update_can_connected_slaves(can_connected_slaves_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_connected_slaves, data, sizeof(can_connected_slaves_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_weight_status_t tanwa_data_read_can_weight_status(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_weight_status_t status = tanwa_data.can_weight_status;
        xSemaphoreGive(tanwa_data_mutex);
        return status;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_weight_status_t empty_status = {0};
        return empty_status;
    }
}

void tanwa_data_update_can_weight_status(can_weight_status_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_weight_status, data, sizeof(can_weight_status_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_weight_data_t tanwa_data_read_can_weight_data(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_weight_data_t data = tanwa_data.can_weight_data;
        xSemaphoreGive(tanwa_data_mutex);
        return data;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_weight_data_t empty_data = {0};
        return empty_data;
    }
}

void tanwa_data_update_can_weight_data(can_weight_data_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_weight_data, data, sizeof(can_weight_data_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_solenoid_status_t tanwa_data_read_can_solenoid_status(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_solenoid_status_t status = tanwa_data.can_solenoid_status;
        xSemaphoreGive(tanwa_data_mutex);
        return status;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_solenoid_status_t empty_status = {0};
        return empty_status;
    }
}

void tanwa_data_update_can_solenoid_status(can_solenoid_status_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_solenoid_status, data, sizeof(can_solenoid_status_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_solenoid_data_t tanwa_data_read_can_solenoid_data(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_solenoid_data_t data = tanwa_data.can_solenoid_data;
        xSemaphoreGive(tanwa_data_mutex);
        return data;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_solenoid_data_t empty_data = {0};
        return empty_data;
    }
}

void tanwa_data_update_can_solenoid_data(can_solenoid_data_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_solenoid_data, data, sizeof(can_solenoid_data_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_sensor_status_t tanwa_data_read_can_sensor_status(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_sensor_status_t status = tanwa_data.can_sensor_status;
        xSemaphoreGive(tanwa_data_mutex);
        return status;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_sensor_status_t empty_status = {0};
        return empty_status;
    }
}

void tanwa_data_update_can_sensor_status(can_sensor_status_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_sensor_status, data, sizeof(can_sensor_status_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_sensor_temp_data_t tanwa_data_read_can_sensor_temp_data(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_sensor_temp_data_t data = tanwa_data.can_sensor_temp_data;
        xSemaphoreGive(tanwa_data_mutex);
        return data;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_sensor_temp_data_t empty_data = {0};
        return empty_data;
    }
}

void tanwa_data_update_can_sensor_temp_data(can_sensor_temp_data_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_sensor_temp_data, data, sizeof(can_sensor_temp_data_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_sensor_pressure_data_t tanwa_data_read_can_sensor_pressure_data(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_sensor_pressure_data_t data = tanwa_data.can_sensor_pressure_data;
        xSemaphoreGive(tanwa_data_mutex);
        return data;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_sensor_pressure_data_t empty_data = {0};
        return empty_data;
    }
}

void tanwa_data_update_can_sensor_pressure_data(can_sensor_pressure_data_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_sensor_pressure_data, data, sizeof(can_sensor_pressure_data_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_utility_status_t tanwa_data_read_can_utility_status(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_utility_status_t status = tanwa_data.can_utility_status;
        xSemaphoreGive(tanwa_data_mutex);
        return status;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_utility_status_t empty_status = {0};
        return empty_status;
    }
}

void tanwa_data_update_can_utility_status(can_utility_status_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_utility_status, data, sizeof(can_utility_status_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_power_status_t tanwa_data_read_can_power_status(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_power_status_t status = tanwa_data.can_power_status;
        xSemaphoreGive(tanwa_data_mutex);
        return status;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_power_status_t empty_status = {0};
        return empty_status;
    }
}

void tanwa_data_update_can_power_status(can_power_status_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_power_status, data, sizeof(can_power_status_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}

can_power_data_t tanwa_data_read_can_power_data(void) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        can_power_data_t data = tanwa_data.can_power_data;
        xSemaphoreGive(tanwa_data_mutex);
        return data;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        can_power_data_t empty_data = {0};
        return empty_data;
    }
}

void tanwa_data_update_can_power_data(can_power_data_t *data) {
    if (xSemaphoreTake(tanwa_data_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&tanwa_data.can_power_data, data, sizeof(can_power_data_t));
        xSemaphoreGive(tanwa_data_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}