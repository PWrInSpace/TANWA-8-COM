#ifndef ENS_TASK_H
#define ENS_TASK_H

#include "ens_logic.h"
#include "esp_now_wrapper.h"

ens_status_t ens_sleep_lock(void);
ens_status_t ens_sleep_unlock(void);
ens_status_t ens_enable_sleep(bool enable);
ens_status_t ens_init(ens_init_struct_t *init_struct, uint16_t transmit_periods[ENS_ENUM_MAX]);

#endif // ENS_TASK_H