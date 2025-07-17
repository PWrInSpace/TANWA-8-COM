// Copyright 2022 PWrInSpace, Kuba
#include "mission_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_timer.h"

uint32_t get_uptime_ms(void) {
    return esp_timer_get_time() / 1000.0;
}

static struct {
    int64_t tzero_time;
    bool enable;
    int64_t disableValue;
    SemaphoreHandle_t time_mutex;
} gb = {
    .tzero_time = 0,
    .enable = false,
    .disableValue = 999,
    .time_mutex = NULL,
};

bool mission_timer_init(int64_t timer_disable_value) {
    gb.time_mutex = xSemaphoreCreateMutex();
    if (gb.time_mutex == NULL) {
        return false;
    }
    gb.disableValue = timer_disable_value;
    return true;
}

bool mission_timer_start(int64_t countdown_begin_time_ms) {
    assert(countdown_begin_time_ms < 0);
    if (countdown_begin_time_ms > 0) {
        return false;
    }

    if (xSemaphoreTake(gb.time_mutex, 1000) != pdTRUE) {
        return false;
    }
    gb.tzero_time = get_uptime_ms() - countdown_begin_time_ms;
    gb.enable = true;
    xSemaphoreGive(gb.time_mutex);

    return true;
}

int mission_timer_get_time() {
    int ret = 0;
    if (xSemaphoreTake(gb.time_mutex, 100) != pdTRUE) {
        return gb.disableValue;
    }

    if (gb.enable == true) {
        ret = get_uptime_ms() - gb.tzero_time;
    } else {
        ret = gb.disableValue;
    }

    xSemaphoreGive(gb.time_mutex);

    return ret;
}

bool mission_timer_is_enable() {
    return gb.enable;
}

bool mission_timer_stop() {
    if (xSemaphoreTake(gb.time_mutex, 1000) != pdTRUE) {
        return false;
    }

    gb.tzero_time = 0;
    gb.enable = false;

    xSemaphoreGive(gb.time_mutex);

    return true;
}

bool mission_timer_set_disable_value(int64_t value) {
    if (xSemaphoreTake(gb.time_mutex, 100) != pdTRUE) {
        return false;
    }

    gb.disableValue = value;
    xSemaphoreGive(gb.time_mutex);
    return true;
}