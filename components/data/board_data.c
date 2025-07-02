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