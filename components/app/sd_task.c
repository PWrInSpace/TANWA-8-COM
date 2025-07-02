// Copyright 2022 PWr in Space, Kuba
#include "sd_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "board_data.h"

#include "sd_task.h"

#include "esp_timer.h"

#include "esp_log.h"
#include "esp_timer.h"

#define TAG "SD_TASK"

#define SD_TASK_STACK_SIZE 8192
#define SD_TASK_PRIORITY 5
#define SD_TASK_CORE_ID 0

extern SemaphoreHandle_t mutex_spi;
int64_t timer = 0;

static struct {
    sd_card_t sd_card;

    TaskHandle_t sd_task;
    QueueHandle_t data_queue;
    QueueHandle_t log_queue;
    SemaphoreHandle_t data_write_mutex;  // prevent race condition during path changing
    SemaphoreHandle_t spi_mutex;

    void *data_from_queue;
    size_t data_from_queue_size;
    char data_buffer[SD_DATA_BUFFER_MAX_SIZE];
    char log_buffer[SD_LOG_BUFFER_MAX_SIZE];

    char data_path[SD_PATH_SIZE];
    char log_path[SD_PATH_SIZE];

    uint32_t try_to_mount_counter;

    error_handler error_handler_fnc;
    create_sd_frame create_sd_frame_fnc;
} mem = {
    .sd_task = NULL,
    .log_queue = NULL,
    .data_write_mutex = NULL,
};

static void report_error(SD_TASK_ERR error_code) {
    if (mem.error_handler_fnc == NULL) {
        return;
    }

    mem.error_handler_fnc(error_code);
}


static bool write_to_sd(FILE *file, char *data, size_t size) {
    if (file == NULL) {
        return false;
    }

    xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
    fwrite(data, 1, size, file);
    xSemaphoreGive(mem.spi_mutex);

    return true;
}

static void get_data_from_queue_and_save(FILE * data_file) {
    size_t frame_size;
    if (xQueueReceive(mem.data_queue, mem.data_from_queue, 0) == pdFALSE) {
            report_error(SD_QUEUE_READ);
    } else {
        frame_size = mem.create_sd_frame_fnc(mem.data_buffer, sizeof(mem.data_buffer),
                                             mem.data_from_queue, mem.data_from_queue_size);
        if (write_to_sd(data_file, mem.data_buffer, frame_size) == false) {
            xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
            SD_remount(&mem.sd_card);
            xSemaphoreGive(mem.spi_mutex);
            report_error(SD_WRITE);
        }
    }
}

static void prepare_data_file_and_save(void) {
    xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
    FILE *data_file = fopen(mem.data_path, "a");
     if (data_file == NULL) {
        ESP_LOGE(TAG, "Can not open the file %s", mem.data_path);
        // return;
    }
    xSemaphoreGive(mem.spi_mutex);

    int received_data_counter = 0;
    // ESP_LOGI(TAG, "Saving to sd");
    while (uxQueueMessagesWaiting(mem.data_queue) > 0) {
        get_data_from_queue_and_save(data_file);

        received_data_counter++;
        if (received_data_counter > SD_MAX_DATA_RECEIVE) {
            ESP_LOGI(TAG, "TIMEOUT");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
    fclose(data_file);
    xSemaphoreGive(mem.spi_mutex);
}

static bool check_sd_status(void) {
    if (mem.sd_card.mounted == true) {
        return true;
    }

    if (mem.try_to_mount_counter < SD_TRY_TO_REMOUNT_DELAY) {
        mem.try_to_mount_counter++;
        return false;
    }
    mem.try_to_mount_counter = 0;

    xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
    bool result = SD_mount(&mem.sd_card);
    xSemaphoreGive(mem.spi_mutex);
    xQueueReset(mem.data_queue);

    return result;
}

static void data_check_and_save(void) {
    if (check_sd_status() == false) {
        return;
    }

    if (uxQueueMessagesWaiting(mem.data_queue) < SD_DATA_DROP_VALUE) {
        return;
    }

    prepare_data_file_and_save();
}

static void log_check_and_save(void) {
    if (check_sd_status() == false) {
        return;
    }

    int received_data_counter = 0;
    while (uxQueueMessagesWaiting(mem.log_queue) > 0) {
        xQueueReceive(mem.log_queue, &mem.log_buffer, 0);
        xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
        if (SD_write(&mem.sd_card, mem.log_path, mem.log_buffer, sizeof(mem.log_buffer)) == false) {
            report_error(SD_WRITE);
        }
        xSemaphoreGive(mem.spi_mutex);

        received_data_counter++;
        if (received_data_counter > SD_MAX_DATA_RECEIVE) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void terminate_task(void) {
    ESP_LOGI(TAG, "Terminating SD TASK");
    vQueueDelete(mem.data_queue);
    vQueueDelete(mem.log_queue);
    mem.data_queue = NULL;
    mem.log_queue = NULL;
    free(mem.data_from_queue);
    vTaskDelete(NULL);
}

static void check_terminate_condition(void) {
    if (ulTaskNotifyTake(pdTRUE, 0) == 0) {
        return;
    }

    prepare_data_file_and_save();
    log_check_and_save();
    terminate_task();
}

static void sdTask(void *args) {
    ESP_LOGI(TAG, "RUNNING SD TASK");
    timer = esp_timer_get_time();
    while (1) {
        //ESP_LOGI(TAG, "Checking SD card status");
        if (xSemaphoreTake(mem.data_write_mutex, 10) == pdTRUE) {
            //ESP_LOGI(TAG, "Checking SD card status");
            data_check_and_save();
            xSemaphoreGive(mem.data_write_mutex);
        } else {
            report_error(SD_MUTEX);
        }

        log_check_and_save();
        check_terminate_condition();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool check_if_file_exists(char *path) {
    bool res;
    xSemaphoreTake(mem.spi_mutex, portMAX_DELAY);
    res = SD_file_exists(path);
    xSemaphoreGive(mem.spi_mutex);
    return res;
}

static bool create_unique_path(char *path, size_t size) {
    char temp_path[SD_PATH_SIZE] = {0};
    int ret = 0;
    for (int i = 0; i < 1000; ++i) {
        ret = snprintf(temp_path, sizeof(temp_path), SD_MOUNT_POINT "/%s%d.txt", path, i);
        if (ret == SD_PATH_SIZE) {
            return false;
        }

        if (check_if_file_exists(temp_path) == false) {
            memcpy(path, temp_path, size);
            return true;
        }
    }
    return false;
}

static bool initialize_sd_card(sd_task_cfg_t *task_cfg) {
    sd_card_config_t card_cfg = {
        .spi_host = task_cfg->spi_host,
        .cs_pin = task_cfg->cs_pin,
        .mount_point = SDCARD_MOUNT_POINT,
    };

    bool ret = SD_init(&mem.sd_card, &card_cfg);
    if (ret == false) {
        ESP_LOGW(TAG, "Unable to initialize SD card");
        // report_error(SD_INIT);
        return false;
    }

    mem.error_handler_fnc = task_cfg->error_handler_fnc;

    memcpy(mem.data_path, task_cfg->data_path, task_cfg->data_path_size);
    memcpy(mem.log_path, task_cfg->log_path, task_cfg->log_path_size);
    if (create_unique_path(mem.data_path, sizeof(mem.data_path)) == false) {
        ESP_LOGE(TAG, "Unable to create unique path");
    }

    if (create_unique_path(mem.log_path, sizeof(mem.log_path)) == false) {
        ESP_LOGE(TAG, "Unable to create unique path");
    }

    ESP_LOGI(TAG, "Using data path %s", mem.data_path);
    ESP_LOGI(TAG, "Using log path %s", mem.log_path);

    return true;
}

static bool initialize_task(sd_task_cfg_t *task_cfg) {
    if (task_cfg->create_sd_frame_fnc == NULL) {
        return false;
    }
    mem.create_sd_frame_fnc = task_cfg->create_sd_frame_fnc;

    mem.data_from_queue_size = task_cfg->data_size;
    mem.data_from_queue = malloc(task_cfg->data_size);
    if (mem.data_from_queue == NULL) {
        return false;
    }

    mem.data_queue = xQueueCreate(SD_DATA_QUEUE_SIZE, mem.data_from_queue_size);
    if (mem.data_queue == NULL) {
        free(mem.data_from_queue);
        return false;
    }

    mem.log_queue = xQueueCreate(SD_LOG_QUEUE_SIZE, sizeof(char[SD_LOG_BUFFER_MAX_SIZE]));
    if (mem.log_queue == NULL) {
        vQueueDelete(mem.data_queue);
        free(mem.data_from_queue);
        mem.data_queue = NULL;
        return false;
    }

    // prevent race condition during path changing
    mem.data_write_mutex = xSemaphoreCreateMutex();
    if (mem.data_write_mutex == NULL) {
        vQueueDelete(mem.data_queue);
        vQueueDelete(mem.log_queue);
        mem.data_queue = NULL;
        mem.log_queue = NULL;
        free(mem.data_from_queue);
        return false;
    }

    xTaskCreatePinnedToCore(sdTask, "sd task", task_cfg->stack_depth, NULL, task_cfg->priority,
                            &mem.sd_task, task_cfg->core_id);

    if (mem.sd_task == NULL) {
        vQueueDelete(mem.data_queue);
        vQueueDelete(mem.log_queue);
        mem.data_queue = NULL;
        mem.log_queue = NULL;
        free(mem.data_from_queue);
        return false;
    }

    return true;
}

static size_t convert_data_to_frame(char *buf, size_t buf_size, void* data, size_t size) {
    tanwa_data_t* tanwa_data = (tanwa_data_t*)data;
    // Create a char buffer from the data with newline ending
    size_t frame_size = 0;
    frame_size = snprintf(buf, buf_size, " ");
    return frame_size;
}

void on_error(SD_TASK_ERR error) {
    if (error == SD_WRITE || error == SD_QUEUE_READ) {
        ESP_LOGE(TAG, "!!! SD ERROR CODE: SD_WRITE !!!");
    } else {
        ESP_LOGE(TAG, "!!! SD ERROR CODE: SD_QUEUE_READ !!!");
    }
}

bool init_sd_card(void) {
    // esp_timer_init();
    ESP_LOGI(TAG, "Initializing sd task");
    sd_task_cfg_t cfg = {
        .cs_pin = CONFIG_SD_CS,
        .data_path = "data",
        .data_path_size = 9,
        .spi_host = SPI3_HOST,
        .log_path = "log",
        .log_path_size = 5,
        .stack_depth = SD_TASK_STACK_SIZE,
        .priority = SD_TASK_PRIORITY,
        .core_id = SD_TASK_CORE_ID,
        .error_handler_fnc = on_error,
        .data_size = sizeof(tanwa_data_t),
        .create_sd_frame_fnc = convert_data_to_frame,
        .spi_mutex = mutex_spi,
    };

    return SDT_init(&cfg);
}

bool SDT_init(sd_task_cfg_t *task_cfg) {
    mem.spi_mutex = task_cfg->spi_mutex;

    if (initialize_sd_card(task_cfg) == false) {
        ESP_LOGE(TAG, "Unable to initialzie sd card");
        return false;
    }

    if (initialize_task(task_cfg) == false) {
        ESP_LOGE(TAG, "Unable to initialzie sd task");
        return false;
    }

    return true;
}

bool SDT_send_data(void *data, size_t data_size) {
    if (mem.data_queue == NULL) {
        //ESP_LOGE(TAG, "Data queue is not initialized");
        return false;
    }

    if (data_size != mem.data_from_queue_size) {
        return false;
    }

    if (mem.sd_card.mounted == false) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return false;
    }

    if (xQueueSend(mem.data_queue, data, 0) == pdFALSE) {
        ESP_LOGE(TAG, "Unable to add data to sd mem.queue");
        return false;
    }

    return true;
}

bool SDT_send_log(char *data, size_t data_size) {
    if (mem.log_queue == NULL) {
        return false;
    }

    if (data_size > SD_DATA_BUFFER_MAX_SIZE) {
        return false;
    }

    if (xQueueSend(mem.log_queue, data, 0) == pdFALSE) {
        ESP_LOGW(TAG, "Unable to add data to sd mem.queue");
        return false;
    }

    return true;
}

bool SDT_change_data_path(char *new_path, size_t path_size) {
    if (xSemaphoreTake(mem.data_write_mutex, 100) == pdFALSE) {
        return false;
    }

    memcpy(mem.data_path, new_path, path_size);
    if (create_unique_path(mem.data_path, sizeof(mem.data_path)) == false) {
        ESP_LOGE(TAG, "Unable to create unique path");
    }

    xSemaphoreGive(mem.data_write_mutex);
    return true;
}

void SDT_terminate_task(void) { xTaskNotifyGive(mem.sd_task); }