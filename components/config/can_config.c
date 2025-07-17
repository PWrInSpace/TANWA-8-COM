#include "can_config.h"
#include "can_api.h"
#include "can_commands.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/twai.h"
#include "driver/gpio.h"

#define TAG "CAN_CONFIG"

can_command_t can_commands[] = {
    // Example command registration
    {CAN_TEMPLATE_MESSAGE_ID, example_command_handler},
    {CAN_SOL_STATUS_ID, parse_solenoid_status},
    {CAN_SOL_DATA_ID, parse_solenoid_data},
    // Add your CAN commands here
};

esp_err_t can_config_init(void) {
    esp_err_t err;

    // Register CAN commands
    err = can_register_commands(can_commands, sizeof(can_commands) / sizeof(can_commands[0]));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN command registration failed");
        return err;
    }

    gpio_config_t io_config = {
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << 37,
    };

    // Configure GPIO for CAN
    err = gpio_config(&io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO configuration failed: %s", esp_err_to_name(err));
        return err;
    }

    gpio_set_level(37, 0); // Set GPIO 37 to low
    
    // Initialize CAN driver
    err = can_task_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN driver initialization failed");
        return err;
    }

    // Start CAN driver
    err = can_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN driver start failed");
        return err;
    }

    return ESP_OK;
}