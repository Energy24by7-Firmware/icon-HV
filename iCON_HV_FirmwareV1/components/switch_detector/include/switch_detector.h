#ifndef SWITCH_DETECTOR_H
#define SWITCH_DETECTOR_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


#include <stdbool.h>

extern TaskHandle_t switch_monitor_taskTaskHandle;

void switch_detector_init(void);
bool get_switch_flag(void);
void reset_switch_flag(void);

void switch_monitor_task(void *pvParameter);

#endif // SWITCH_DETECTOR_H
