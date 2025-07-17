#include "relay_driver.h"
#include "esp_timer.h"

static relay_driver_gpio_set_level_t _gpio_set_level = NULL;

relay_driver_err_t relay_driver_init(relay_driver_gpio_set_level_t gpio_set_level) {

    _gpio_set_level = gpio_set_level;

    if(_gpio_set_level == NULL) {
        return RELAY_DRIVER_ERROR; // GPIO set level function not provided
    }

    return RELAY_DRIVER_OK;
}

relay_driver_err_t relay_open(relay_driver_t *driver) {

    if (driver == NULL) {
        return RELAY_DRIVER_ERROR;
    }

    if(driver->state == RELAY_ON) {
        return RELAY_DRIVER_OK; // Already open
    }

    if(!_gpio_set_level(driver->gpio_pin, 1)) {
        return RELAY_DRIVER_ERROR; // Failed to set GPIO level
    }

    driver->state = RELAY_ON;
    return RELAY_DRIVER_OK;
}

relay_driver_err_t relay_close(relay_driver_t *driver) {

    if (driver == NULL) {
        return RELAY_DRIVER_ERROR;
    }

    if(driver->state == RELAY_OFF) {
        return RELAY_DRIVER_OK; // Already closed
    }

    if(!_gpio_set_level(driver->gpio_pin, 0)) {
        return RELAY_DRIVER_ERROR; // Failed to set GPIO level
    }

    driver->state = RELAY_OFF;
    return RELAY_DRIVER_OK;
}

relay_driver_err_t relay_toggle(relay_driver_t *driver) {

    if (driver == NULL) {
        return RELAY_DRIVER_ERROR;
    }

    if(driver->state == RELAY_ON) {
        return relay_close(driver);
    } else {
        return relay_open(driver);
    }
}

relay_driver_err_t relay_get_state(relay_driver_t *driver, relay_driver_state_t *state) {

    if (driver == NULL || state == NULL) {
        return RELAY_DRIVER_ERROR;
    }

    *state = driver->state;
    return RELAY_DRIVER_OK;
}

typedef struct {
    relay_driver_t *driver;
} relay_timer_arg_t;

static void relay_timer_callback(void* arg) {
    relay_timer_arg_t *timer_arg = (relay_timer_arg_t*)arg;
    relay_close(timer_arg->driver);
    free(timer_arg);
}

relay_driver_err_t relay_time_open(relay_driver_t *driver, uint32_t timeout_ms) {

    if (driver == NULL) {
        return RELAY_DRIVER_ERROR;
    }

    if(driver->state == RELAY_ON) {
        return RELAY_DRIVER_OK; // Already open
    }

    if(!_gpio_set_level(driver->gpio_pin, 1)) {
        return RELAY_DRIVER_ERROR; // Failed to set GPIO level
    }

    driver->state = RELAY_ON;

    // Przygotuj argument dla timera
    relay_timer_arg_t *timer_arg = malloc(sizeof(relay_timer_arg_t));
    if (!timer_arg) {
        return RELAY_DRIVER_ERROR;
    }
    timer_arg->driver = driver;

    // Utwórz i uruchom jednorazowy timer
    const esp_timer_create_args_t timer_args = {
        .callback = relay_timer_callback,
        .arg = timer_arg,
        .name = "relay_timer"
    };
    esp_timer_handle_t timer_handle;
    if (esp_timer_create(&timer_args, &timer_handle) != ESP_OK) {
        free(timer_arg);
        return RELAY_DRIVER_ERROR;
    }
    esp_timer_start_once(timer_handle, timeout_ms * 1000); // czas w mikrosekundach

    return RELAY_DRIVER_OK;
}

