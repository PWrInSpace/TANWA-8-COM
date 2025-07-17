#ifndef PWRINSPACE_RELAY_DRIVER_H
#define PWRINSPACE_RELAY_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RELAY_OFF = 0,
    RELAY_ON = 1
} relay_driver_state_t;

typedef enum {
    RELAY_DRIVER_OK = 0,
    RELAY_DRIVER_ERROR = 1
} relay_driver_err_t;

typedef struct relay_driver_t {
    uint8_t gpio_pin;
    relay_driver_state_t state;
} relay_driver_t;

typedef bool (*relay_driver_gpio_set_level_t)(uint8_t pin, bool level);

relay_driver_err_t relay_driver_init(relay_driver_gpio_set_level_t gpio_set_level);
relay_driver_err_t relay_open(relay_driver_t *driver);
relay_driver_err_t relay_close(relay_driver_t *driver);
relay_driver_err_t relay_toggle(relay_driver_t *driver);
relay_driver_err_t relay_get_state(relay_driver_t *driver, relay_driver_state_t *state);
relay_driver_err_t relay_time_open(relay_driver_t *driver, uint32_t timeout_ms);

#endif // PWRINSPACE_RELAY_DRIVER_H