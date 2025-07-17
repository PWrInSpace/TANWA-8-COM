#include "app_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include "driver/twai.h"

#include "esp_log.h"

#include "can_api.h"

#define APP_TASK_STACK_SIZE CONFIG_APP_TASK_STACK_SIZE
#define APP_TASK_PRIORITY CONFIG_APP_TASK_PRIORITY
#define APP_TASK_CORE_ID CONFIG_APP_TASK_CORE_ID

static TaskHandle_t app_task_handle = NULL;

esp_err_t app_task_init(void) {
    
    if(xTaskCreatePinnedToCore(app_task, "app_task", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, &app_task_handle, APP_TASK_CORE_ID) == pdPASS) {
        ESP_LOGI("APP_TASK", "App task created successfully");
    } else {
        ESP_LOGE("APP_TASK", "Failed to create app task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t app_task_deinit(void) {
    if (app_task_handle != NULL) {
        vTaskDelete(app_task_handle);
        app_task_handle = NULL;
    }
    
    return ESP_OK;
}

int gpio_state = 0;

void app_task(void *arg) {

    while(1) {
    
        uint8_t data[8] = {0};
        can_send_message(CAN_SOL_GET_DATA_ID, data, 0);
        can_send_message(CAN_SOL_GET_STATUS_ID, data, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI("APP_TASK", "Sending CAN messages");

        can_send_message(CAN_SOL_OPEN_SOL_ID, data, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        can_send_message(CAN_SOL_CLOSE_SOL_ID, data, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}