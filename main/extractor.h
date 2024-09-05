#ifndef EXTRACTOR_H
#define EXTRACTOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stdlib.h>

esp_err_t readFileParse(const char *filename);
typedef struct
{
    uint32_t frameNumber;
    long offset;
} FrameOffset;

typedef struct
{
    uint32_t firstFrameId;
    uint32_t size;
} CompressionBlockData;

typedef struct
{
    uint32_t OffsetData;
    uint32_t noOfFrames;
} CompressionBlockExtendedData;

void applyBrightnessGammaCorrection(void);



#endif // EXTRACTOR_H