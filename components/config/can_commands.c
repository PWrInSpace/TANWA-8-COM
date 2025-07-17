#include "esp_log.h"

#include "can_commands.h"

esp_err_t example_command_handler(uint8_t *data, uint8_t length) {
    
    // Process the data received in the CAN message
    // For example, print the first byte of data
    ESP_LOGI("CAN_COMMANDS", "Example command handler");
    
    return ESP_OK;
}

esp_err_t parse_solenoid_status(uint8_t *data, uint8_t length) {

    for(int i = 0; i < length; i++) {
        ESP_LOGI("CAN_COMMANDS", "Solenoid status byte %d: %02X", i, data[i]);
    }
    
    return ESP_OK;
}

esp_err_t parse_solenoid_data(uint8_t *data, uint8_t length) {

    for(int i = 0; i < length; i++) {
        ESP_LOGI("CAN_COMMANDS", "Solenoid data byte %d: %02X", i, data[i]);
    }
    
    return ESP_OK;
}