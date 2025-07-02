// Copyright 2022 PWr in Space, Kuba
#ifndef SD_TASK_H
#define SD_TASK_H

#include <stdint.h>
#include "sdcard.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define SD_LOG_BUFFER_MAX_SIZE 256
#define SD_DATA_BUFFER_MAX_SIZE 512
#define SD_MOUNT_POINT "/sdcard"

#define SD_DATA_QUEUE_SIZE 20
#define SD_LOG_QUEUE_SIZE 20
#define SD_MAX_DATA_RECEIVE 25
#define SD_TRY_TO_REMOUNT_DELAY 1000
#define SD_DATA_DROP_VALUE 10

#define SD_PATH_SIZE 40

typedef enum {
    SD_INIT,
    SD_QUEUE_READ,
    SD_WRITE,
    SD_MUTEX,
} SD_TASK_ERR;


typedef void (*error_handler)(SD_TASK_ERR error_code);
typedef size_t (*create_sd_frame)(char *buffer, size_t buffer_size, void* data, size_t size);


typedef struct {
    spi_host_device_t spi_host;
    uint8_t cs_pin;
    char *data_path;
    size_t data_path_size;
    char *log_path;
    size_t log_path_size;

    size_t data_size;

    uint32_t stack_depth;
    BaseType_t core_id;
    UBaseType_t priority;

    SemaphoreHandle_t spi_mutex;

    error_handler error_handler_fnc;
    create_sd_frame create_sd_frame_fnc;
} sd_task_cfg_t;

/**
 * @brief Initalize sd card task
 * 
 * @return true :D
 * @return false :C
 */
bool init_sd_card(void);

/**
 * @brief Initialzie sd task
 *
 * @param cfg pointer to task conifg
 * @return true :)
 * @return false :C
 */
bool SDT_init(sd_task_cfg_t *cfg);

/**
 * @brief Send data to save
 *
 * @param data pointer to data string
 * @param data_size data string size
 * @return true :)
 * @return false :C
 */
bool SDT_send_data(void *data, size_t data_size);

/**
 * @brief Send log to save
 *
 * @param log pointer to log string
 * @param log_size log stirng size
 * @return true :)
 * @return false :C
 */
bool SDT_send_log(char *log, size_t log_size);

/**
 * @brief Change path for data saving
 * 
 * @param new_path new path string
 * @param path_size new path size 
 * @return true :)
 * @return false  :C
 */
bool SDT_change_data_path(char *new_path, size_t path_size);

/**
 * @brief Terminate sd task
 *
 */
void SDT_terminate_task(void);
#endif