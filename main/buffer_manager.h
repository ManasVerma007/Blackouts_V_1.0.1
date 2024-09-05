#ifndef BUFFER_MANAGER_H
#define BUFFER_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stdlib.h>

typedef enum {
    BUFFER_EMPTY,
    BUFFER_FILLED,
    BUFFER_PROCESSING,
    BUFFER_PROCESSED
} BufferState;

typedef struct {
    unsigned char* data;
    size_t size;
    BufferState state;
    unsigned int noOfFrames;
    unsigned int channelCount;
    unsigned int frameStepTime;
} DecompressionBuffer;

extern DecompressionBuffer buffer1;
extern DecompressionBuffer buffer2;
extern DecompressionBuffer buffer3;
extern DecompressionBuffer buffer4;
extern DecompressionBuffer buffer5;

#endif 

extern QueueHandle_t decompressionBufferQueue;
