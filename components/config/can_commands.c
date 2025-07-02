#include "esp_log.h"

#include "can_commands.h"

esp_err_t example_command_handler(uint8_t *data, uint8_t length) {
    
    // Process the data received in the CAN message
    // For example, print the first byte of data
    ESP_LOGI("CAN_COMMANDS", "Example command handler");
    
    return ESP_OK;
}