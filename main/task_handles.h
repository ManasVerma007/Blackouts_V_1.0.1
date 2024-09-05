#ifndef TASK_HANDLES_H
#define TASK_HANDLES_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t readFileTaskHandle;
extern TaskHandle_t ledGlowTaskHandle;

#endif // TASK_HANDLES_H