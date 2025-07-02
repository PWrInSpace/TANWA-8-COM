#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "board_config.h"
#include "setup_task.h"
#include "mcu_spi_config.h"
#include "sd_task.h"

#define TAG "APP"

extern board_config_t config;

void app_main(void) {
    
    // CONFIGURE THE MESSAGE

    ESP_LOGI(TAG, "%s TANWA board starting", config.board_name);
    
    if(setup_task_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize setup task");
        return;
    }

    while(1) {
        led_toggle(&(config.status_led));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // esp_err_t err;

    // err = mcu_spi_init();
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "SPI initialization failed");
    // }

    // ESP_LOGI(TAG, "Initializing SD Card...");

    // if (!init_sd_card()) {
    //     ESP_LOGE(TAG, "SD Card initialization failed");
    // } else {
    //     ESP_LOGI(TAG, "### SD Card initialization success ###");
    // }
}

// #include <stdio.h>
// #include "esp_log.h"
// #include "esp_err.h"
// #include "driver/sdspi_host.h"
// #include "driver/spi_common.h"
// #include "sdmmc_cmd.h"
// #include "esp_vfs_fat.h"

// #define TAG "SD_SIMPLE"

// // Dostosuj te piny do swojego hardware!
// #define PIN_NUM_MISO  10
// #define PIN_NUM_MOSI  11
// #define PIN_NUM_CLK   9
// #define PIN_NUM_CS    15

// void app_main(void)
// {
//     esp_err_t ret;
//     sdmmc_card_t* card;
//     const char mount_point[] = "/sdcard";
//     ESP_LOGI(TAG, "Initializing SD card");

//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << PIN_NUM_MOSI) | (1ULL << PIN_NUM_CS),
//         .mode = GPIO_MODE_INPUT_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     };

//     gpio_config(&io_conf);

//     gpio_config_t clk_conf = {
//         .pin_bit_mask = (1ULL << PIN_NUM_CLK),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     };

//     gpio_config(&clk_conf);

//     // Ustawienie pinu MISO jako wejście
//     gpio_config_t miso_conf = {
//         .pin_bit_mask = (1ULL << PIN_NUM_MISO),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     };
//     gpio_config(&miso_conf);    

//     // Konfiguracja hosta SPI
//     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
//     host.max_freq_khz = 400; // Ustawienie maksymalnej częstotliwości na 400 kHz
//     spi_bus_config_t bus_cfg = {
//         .mosi_io_num = PIN_NUM_MOSI,
//         .miso_io_num = PIN_NUM_MISO,
//         .sclk_io_num = PIN_NUM_CLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 4000,
//     };
//     ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to initialize bus.");
//         return;
//     }

//     // Konfiguracja slotu SPI dla SD
//     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
//     slot_config.gpio_cs = PIN_NUM_CS;
//     slot_config.host_id = host.slot;

//     // Montowanie systemu plików FAT
//     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
//         .format_if_mount_failed = false,
//         .max_files = 3,
//         .allocation_unit_size = 16 * 1024
//     };

//     ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to mount filesystem. Error: %s", esp_err_to_name(ret));
//         return;
//     }
//     ESP_LOGI(TAG, "SD card mounted.");

//     // Przykład: zapis do pliku
//     FILE* f = fopen("/sdcard/hello.txt", "w");
//     if (f == NULL) {
//         ESP_LOGE(TAG, "Failed to open file for writing");
//     } else {
//         fprintf(f, "Hello SD card!\n");
//         fclose(f);
//         ESP_LOGI(TAG, "File written");
//     }

//     // Odmontowanie karty
//     esp_vfs_fat_sdcard_unmount(mount_point, card);
//     ESP_LOGI(TAG, "SD card unmounted");

//     // Zwolnienie magistrali SPI
//     spi_bus_free(host.slot);
// }