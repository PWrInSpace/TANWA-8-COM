#ifndef PWRINSPACE_APP_TASK_H
#define PWRINSPACE_APP_TASK_H

#include "esp_err.h"

void app_task(void *arg);
esp_err_t app_task_init(void);
esp_err_t app_task_deinit(void);
void app_task(void *arg);


#include "stdbool.h"

extern volatile  bool buzz_flg;
extern volatile  bool buzz_flg_off;

#endif //PWRINSPACE_APP_TASK_H