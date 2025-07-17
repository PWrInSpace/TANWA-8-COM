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


/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// #include <string.h>
// #include <sys/unistd.h>
// #include <sys/stat.h>
// #include "esp_vfs_fat.h"
// #include "sdmmc_cmd.h"
// #include "sd_test_io.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #if SOC_SDMMC_IO_POWER_EXTERNAL
// #include "sd_pwr_ctrl_by_on_chip_ldo.h"
// #endif

// #define EXAMPLE_MAX_CHAR_SIZE    64

// static const char *TAG = "example";

// #define MOUNT_POINT "/sdcard"

// // #ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
// const char* names[] = {"CLK ", "MOSI", "MISO", "CS  "};
// const int pins[] = {9,
//                     11,
//                     10,
//                     15};

// const int pin_count = sizeof(pins)/sizeof(pins[0]);
// #if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
// const int adc_channels[] = {CONFIG_EXAMPLE_ADC_PIN_CLK,
//                             CONFIG_EXAMPLE_ADC_PIN_MOSI,
//                             CONFIG_EXAMPLE_ADC_PIN_MISO,
//                             CONFIG_EXAMPLE_ADC_PIN_CS};
// #endif //CONFIG_EXAMPLE_ENABLE_ADC_FEATURE

// pin_configuration_t config = {
//     .names = names,
//     .pins = pins,
// #if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
//     .adc_channels = adc_channels,
// #endif
// };
// // #endif //CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS

// // Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// // You can also change the pin assignments here by changing the following 4 lines.
// #define PIN_NUM_MISO  10
// #define PIN_NUM_MOSI  11
// #define PIN_NUM_CLK   9
// #define PIN_NUM_CS    15

// static esp_err_t s_example_write_file(const char *path, char *data)
// {
//     ESP_LOGI(TAG, "Opening file %s", path);
//     FILE *f = fopen(path, "w");
//     if (f == NULL) {
//         ESP_LOGE(TAG, "Failed to open file for writing");
//         return ESP_FAIL;
//     }
//     fprintf(f, data);
//     fclose(f);
//     ESP_LOGI(TAG, "File written");

//     return ESP_OK;
// }

// static esp_err_t s_example_read_file(const char *path)
// {
//     ESP_LOGI(TAG, "Reading file %s", path);
//     FILE *f = fopen(path, "r");
//     if (f == NULL) {
//         ESP_LOGE(TAG, "Failed to open file for reading");
//         return ESP_FAIL;
//     }
//     char line[EXAMPLE_MAX_CHAR_SIZE];
//     fgets(line, sizeof(line), f);
//     fclose(f);

//     // strip newline
//     char *pos = strchr(line, '\n');
//     if (pos) {
//         *pos = '\0';
//     }
//     ESP_LOGI(TAG, "Read from file: '%s'", line);

//     return ESP_OK;
// }

// void app_main(void)
// {
//     esp_err_t ret;

//     // Options for mounting the filesystem.
//     // If format_if_mount_failed is set to true, SD card will be partitioned and
//     // formatted in case when mounting fails.
//     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
// #ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
//         .format_if_mount_failed = true,
// #else
//         .format_if_mount_failed = false,
// #endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
//         .max_files = 5,
//         .allocation_unit_size = 16 * 1024
//     };
//     sdmmc_card_t *card;
//     const char mount_point[] = MOUNT_POINT;
//     ESP_LOGI(TAG, "Initializing SD card");

//     // Use settings defined above to initialize SD card and mount FAT filesystem.
//     // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
//     // Please check its source code and implement error recovery when developing
//     // production applications.
//     ESP_LOGI(TAG, "Using SPI peripheral");

//     // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
//     // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
//     // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
//     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
//     host.max_freq_khz = 400; // 400 kHz for SD initialization (most safe)

//     // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
//     // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
//     // and the internal LDO power supply, we need to initialize the power supply first.
// #if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
//     sd_pwr_ctrl_ldo_config_t ldo_config = {
//         .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
//     };
//     sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

//     ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
//         return;
//     }
//     host.pwr_ctrl_handle = pwr_ctrl_handle;
// #endif

//     spi_bus_config_t bus_cfg = {
//         .mosi_io_num = PIN_NUM_MOSI,
//         .miso_io_num = PIN_NUM_MISO,
//         .sclk_io_num = PIN_NUM_CLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 4000,
//     };

//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << PIN_NUM_CS) | (1ULL << PIN_NUM_CLK),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     };
//     ret = gpio_config(&io_conf);

//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to configure GPIOs for CS and CLK");
//         return;
//     }

// // Set MISO and MOSI pins as input/output with pull-up resistors
// gpio_config_t miso_conf = {
//     .pin_bit_mask = (1ULL << PIN_NUM_MISO),
//     .mode = GPIO_MODE_INPUT,
//     .pull_up_en = GPIO_PULLUP_ENABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
// };
// ret = gpio_config(&miso_conf);
// if (ret != ESP_OK) {
//     ESP_LOGE(TAG, "Failed to configure GPIO for MISO");
//     return;
// }

// gpio_config_t mosi_conf = {
//     .pin_bit_mask = (1ULL << PIN_NUM_MOSI),
//     .mode = GPIO_MODE_OUTPUT,
//     .pull_up_en = GPIO_PULLUP_ENABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
// };
// ret = gpio_config(&mosi_conf);
// if (ret != ESP_OK) {
//     ESP_LOGE(TAG, "Failed to configure GPIO for MOSI");
//     return;
// }

//     ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to initialize bus.");
//         return;
//     }

//     // This initializes the slot without card detect (CD) and write protect (WP) signals.
//     // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
//     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
//     slot_config.gpio_cs = PIN_NUM_CS;
//     slot_config.host_id = host.slot;

//     ESP_LOGI(TAG, "Mounting filesystem");

//     do{
//         ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
//         if (ret != ESP_OK) {
//             if (ret == ESP_FAIL) {
//                 ESP_LOGE(TAG, "Failed to mount filesystem. "
//                      "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
//             } else {
//                 ESP_LOGE(TAG, "Failed to initialize the card (%s). "
//                      "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
//                 check_sd_card_pins(&config, pin_count);
//                 vTaskDelay(200 / portTICK_PERIOD_MS);
//             }
//         }
//     } while (ret != ESP_OK);

//     if (ret != ESP_OK) {
//         if (ret == ESP_FAIL) {
//             ESP_LOGE(TAG, "Failed to mount filesystem. "
//                      "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
//         } else {
//             ESP_LOGE(TAG, "Failed to initialize the card (%s). "
//                      "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));

// // #ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
//             check_sd_card_pins(&config, pin_count);
// // #endif
//         }
//         return;
//     }
//     ESP_LOGI(TAG, "Filesystem mounted");

//     // Card has been initialized, print its properties
//     sdmmc_card_print_info(stdout, card);

//     // Use POSIX and C standard library functions to work with files.

//     // First create a file.
//     const char *file_hello = MOUNT_POINT"/hello.txt";
//     char data[EXAMPLE_MAX_CHAR_SIZE];
//     snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
//     ret = s_example_write_file(file_hello, data);
//     if (ret != ESP_OK) {
//         return;
//     }

//     const char *file_foo = MOUNT_POINT"/foo.txt";

//     // Check if destination file exists before renaming
//     struct stat st;
//     if (stat(file_foo, &st) == 0) {
//         // Delete it if it exists
//         unlink(file_foo);
//     }

//     // Rename original file
//     ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
//     if (rename(file_hello, file_foo) != 0) {
//         ESP_LOGE(TAG, "Rename failed");
//         return;
//     }

//     ret = s_example_read_file(file_foo);
//     if (ret != ESP_OK) {
//         return;
//     }

//     // Format FATFS
// #ifdef CONFIG_EXAMPLE_FORMAT_SD_CARD
//     ret = esp_vfs_fat_sdcard_format(mount_point, card);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(ret));
//         return;
//     }

//     if (stat(file_foo, &st) == 0) {
//         ESP_LOGI(TAG, "file still exists");
//         return;
//     } else {
//         ESP_LOGI(TAG, "file doesn't exist, formatting done");
//     }
// #endif // CONFIG_EXAMPLE_FORMAT_SD_CARD

//     const char *file_nihao = MOUNT_POINT"/nihao.txt";
//     memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
//     snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Nihao", card->cid.name);
//     ret = s_example_write_file(file_nihao, data);
//     if (ret != ESP_OK) {
//         return;
//     }

//     //Open file for reading
//     ret = s_example_read_file(file_nihao);
//     if (ret != ESP_OK) {
//         return;
//     }

//     // All done, unmount partition and disable SPI peripheral
//     esp_vfs_fat_sdcard_unmount(mount_point, card);
//     ESP_LOGI(TAG, "Card unmounted");

//     //deinitialize the bus after all devices are removed
//     spi_bus_free(host.slot);

//     // Deinitialize the power control driver if it was used
// #if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
//     ret = sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to delete the on-chip LDO power control driver");
//         return;
//     }
// #endif
// }