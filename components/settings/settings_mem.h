#ifndef SETTINGS_MEM_H
#define SETTINGS_MEM_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define SETTINGS_TAG "SettingsTag"

#define SETTINGS_NAMESPACE "Settings"
#define SETTINGS_LORA_FREQ_KHZ "lora_MHz"
#define SETTINGS_LORA_TRANSMIT_MS "lora_ms"
#define SETTINGS_COUNTDOWN_TIME "countdownTime"
#define SETTINGS_IGNIT_TIME "ignitTime"
#define SETTINGS_FLASH_ON "flash"
#define FUEL_VALVE_INITIAL_ANGLE "fuelInitAngle"
#define OXIDIZER_VALVE_INITIAL_ANGLE "oxiInitAngle"
#define FUEL_OPEN_TIME_MS "fuelOpenTime"
#define OXIDIZER_FULL_OPEN_TIME_MS "oxiFOpenTime"
#define FUEL_FULL_OPEN_TIME_MS "fuelFOpenTime"


typedef struct {
    int32_t loraFreq_KHz;
    int32_t lora_transmit_ms;
    int32_t countdownTime;
    int32_t ignitTime;
    int32_t fuel_open_time_ms;
    int32_t oxidizer_full_open_time_ms;
    int32_t fuel_full_open_time_ms;
    int32_t fuel_valve_initial_angle;
    int32_t oxidizer_valve_initial_angle;
    int32_t flash_on;
} Settings;


esp_err_t settings_init();
esp_err_t settings_save_all();
esp_err_t settings_read_all();
void settings_init_default();
Settings settings_get_all();
esp_err_t settings_read(char* setting_name, int32_t *Val);
esp_err_t settings_save(char* setting_name, int32_t newVal);

#endif