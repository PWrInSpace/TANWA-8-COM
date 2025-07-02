///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 15.05.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "mcu_spi_config.h"

#include "mcu_gpio_config.h"

#include "esp_log.h"

#define TAG "MCU_SPI"

static mcu_spi_config_t spi_config = MCU_SPI_DEFAULT_CONFIG();
SemaphoreHandle_t mutex_spi;

esp_err_t mcu_spi_init(void) {
    esp_err_t ret = ESP_OK;
    if (spi_config.spi_init_flag) {
      return ESP_OK;
    }

    ret = spi_bus_initialize(spi_config.host_id, &spi_config.bus_config, SDSPI_DEFAULT_DMA);
    ESP_ERROR_CHECK(ret);

    // ret = spi_bus_add_device(spi_config.host_id, &spi_config.dev_config,
    //                         &spi_config.spi_handle);
    ESP_ERROR_CHECK(ret);
    mutex_spi = xSemaphoreCreateMutex();
    spi_config.spi_init_flag = true;
    return ret;
}

esp_err_t mcu_spi_deinit(void) {
    esp_err_t ret = ESP_OK;
    if (!spi_config.spi_init_flag) {
      return ESP_OK;
    }
    ret = spi_bus_remove_device(spi_config.spi_handle);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_free(spi_config.host_id);
    ESP_ERROR_CHECK(ret);
    spi_config.spi_init_flag = false;
    return ret;
}

bool _lora_add_device(void) {
    esp_err_t ret;

    spi_device_interface_config_t dev = {.clock_speed_hz = 400000,
                                         .mode = 0,
                                         .spics_io_num = -1,
                                         .queue_size = 1,
                                         .flags = 0,
                                         .pre_cb = NULL};
    ret = spi_bus_add_device(spi_config.host_id, &dev, &spi_config.spi_handle);
    ESP_ERROR_CHECK(ret);

    return ret == ESP_OK ? true : false;
}

bool _lora_spi_transmit(uint8_t _in[2], uint8_t _out[2]) {
    spi_transaction_t t = {.flags = 0,
                          .length = 8 * sizeof(uint8_t) * 2,
                          .tx_buffer = _out,
                          .rx_buffer = _in};
    xSemaphoreTake(mutex_spi, portMAX_DELAY);
    _mcu_gpio_set_level(LORA_CS_GPIO_INDEX, 0);
    spi_device_transmit(spi_config.spi_handle, &t);
    _mcu_gpio_set_level(LORA_CS_GPIO_INDEX, 1);
    xSemaphoreGive(mutex_spi);
    return true;
}