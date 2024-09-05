#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "extractor.h"
#include "errno.h"
#include "zlib.h"
#include "math.h"
#include "models.c"
#define TINYFSEQ_IMPLEMENTATION
#include "tinyfseq.h"
#include <string.h>
#include "buffer_manager.h"
#include "esp_heap_caps.h"
#include "led_glow.h"
#include "time.h"

DecompressionBuffer buffer1;
DecompressionBuffer buffer2;
DecompressionBuffer buffer3;
DecompressionBuffer buffer4;

// create an array of buffers
DecompressionBuffer *buffers[4] = {&buffer1, &buffer2, &buffer3, &buffer4};

static int currentBufferIndex = 0;

#define TAG "extractor"

size_t freeMemPsram;
size_t freeMemInternal;

#define MAX_HEADER_SIZE 32

void initializeBuffer(DecompressionBuffer *buffer)
{
    buffer->data = NULL;
    buffer->size = 0;
    buffer->state = BUFFER_EMPTY;
    buffer->noOfFrames = 0;
    buffer->channelCount = 0;
}

void printHeader(TFHeader *header)
{
    printf("Channel Data Offset: %hu\n", header->channelDataOffset);
    printf("Minor Version: %hhu\n", header->minorVersion);
    printf("Major Version: %hhu\n", header->majorVersion);
    printf("Variable Data Offset: %hu\n", header->variableDataOffset);
    printf("Channel Count: %lu\n", header->channelCount);
    printf("Frame Count: %lu\n", header->frameCount);
    printf("Frame Step Time Millis: %hhu\n", header->frameStepTimeMillis);
    printf("Compression Type: %d\n", header->compressionType);
    printf("Compression Block Count: %hu\n", header->compressionBlockCount);
    printf("Channel Range Count: %hhu\n", header->channelRangeCount);
    printf("Sequence UID: %llu\n", header->sequenceUid);
}

esp_err_t readFileParse(const char *fileName)
{
    long fileSize;
    TFHeader header;
    uint8_t *endPointer;
    TFError error;
    long channelDataOffset;
    TFCompressionBlock *compressionBlocks = NULL;
    uint32_t compressionBlocksSizeTemp = 0;
    CompressionBlockExtendedData *compressionBlocksExtData = NULL;
    unsigned char *decompressed_data = NULL;
    uLongf decompressed_size, decompressed_sizeTemp;
    uint32_t lenTotalCompressionBlock;
    int zlibError = 0;
    uint8_t *fileBuffer = malloc(MAX_HEADER_SIZE * sizeof(uint8_t));

    printf("Filename: %s\n", fileName);
    FILE *file = fopen(fileName, "rb");
    if (file == NULL)
    {
        printf("Failed to open file\n");
        return ESP_FAIL;
    }

    fseek(file, 0, SEEK_END);
    fileSize = ftell(file);
    rewind(file);

    size_t bytesRead = fread(fileBuffer, 1, MAX_HEADER_SIZE, file); //*fileSize
    if (bytesRead != MAX_HEADER_SIZE)                               // *fileSize
    {
        printf("readFileIntoBuffer - Failed to read file\n");
        free(fileBuffer);
        fclose(file);
        return ESP_FAIL;
    }

    error = TFHeader_read(fileBuffer, fileSize, &header, &endPointer);
    if (error != TF_OK)
    {
        printf("Failed to read header: %d\n", error);
        free(fileBuffer);
        fclose(file);
        return ESP_FAIL;
        ;
    }
    printHeader(&header);
    channelDataOffset = header.channelDataOffset;
    if (header.compressionBlockCount > 0)
    {
        compressionBlocks = heap_caps_malloc(header.compressionBlockCount * sizeof(TFCompressionBlock), MALLOC_CAP_SPIRAM);
        compressionBlocksExtData = heap_caps_malloc(header.compressionBlockCount * sizeof(CompressionBlockExtendedData), MALLOC_CAP_SPIRAM);
        if (compressionBlocks == NULL)
        {
            printf("Failed to allocate memory for compression blocks or compression block data\n");
            free(fileBuffer);
            fclose(file);
            return ESP_FAIL;
        }
        lenTotalCompressionBlock = (header.variableDataOffset - MAX_HEADER_SIZE) * sizeof(uint8_t);
        fileBuffer = realloc(fileBuffer, lenTotalCompressionBlock);
        ESP_LOGI(TAG, "\n lenTotalCompressionBlock = %lu", lenTotalCompressionBlock);
        fseek(file, MAX_HEADER_SIZE, SEEK_SET);
        bytesRead = fread(fileBuffer, 1, lenTotalCompressionBlock, file); //*fileSize
        if (bytesRead != lenTotalCompressionBlock)                        // *fileSize
        {
            printf("reading Compression Blocks - Failed to read file\n");
            free(fileBuffer);
            free(compressionBlocks);
            free(compressionBlocksExtData);
            fclose(file);
            return ESP_FAIL;
        }
        endPointer = fileBuffer;
        uint8_t *startTemp = endPointer;
        uint32_t prevSizeTemp = 0;
        uint32_t currentOffsetTemp = channelDataOffset;
        for (uint32_t i = 0; i < header.compressionBlockCount; i++)
        {

            long remainingSize = lenTotalCompressionBlock - (endPointer - startTemp);

            if (remainingSize < 0)
            {
                printf("Error: remaining size is negative\n");
                free(fileBuffer);
                free(compressionBlocks);
                free(compressionBlocksExtData);
                fclose(file);
                return ESP_FAIL;
            }
            error = TFCompressionBlock_read(endPointer, remainingSize, &compressionBlocks[i], &endPointer);
            if (error != TF_OK)
            {
                printf("Failed to read compression block: %d\n", error);
                free(fileBuffer);
                free(compressionBlocks);
                free(compressionBlocksExtData);
                fclose(file);
                return ESP_FAIL;
            }
            compressionBlocksExtData[i].OffsetData = currentOffsetTemp + prevSizeTemp;
            currentOffsetTemp = compressionBlocksExtData[i].OffsetData;
            prevSizeTemp = compressionBlocks[i].size;
            compressionBlocksExtData[i].noOfFrames = 0; // just to initialize
            if (i > 0)
            {
                if (compressionBlocks[i].firstFrameId > compressionBlocks[i - 1].firstFrameId)
                {
                    compressionBlocksExtData[i - 1].noOfFrames = compressionBlocks[i].firstFrameId - compressionBlocks[i - 1].firstFrameId;
                    // printf("\ncompressionBlocksExtData[%lu].noOfFrames = %lu \n", i - 1, compressionBlocksExtData[i - 1].noOfFrames);
                }
                else if ((compressionBlocks[i].firstFrameId == 0) && (compressionBlocks[i - 1].firstFrameId > 0))
                {
                    compressionBlocksExtData[i - 1].noOfFrames = header.frameCount - compressionBlocks[i - 1].firstFrameId;
                    // printf("\ncompressionBlocksExtData[%lu].noOfFrames = %lu \n", i - 1, compressionBlocksExtData[i - 1].noOfFrames);
                }
                else if ((compressionBlocks[i].firstFrameId == 0) && (compressionBlocks[i - 1].firstFrameId == 0))
                {
                }

                if (i == (header.compressionBlockCount - 1))
                {
                    if (compressionBlocks[i].firstFrameId > 0)
                    {
                        compressionBlocksExtData[i].noOfFrames = header.frameCount - compressionBlocks[i].firstFrameId;
                    }
                }
            }
            // printf("\n compressionBlocks[%lu] firstFrameId = %lu size = %lu offset = %lu no of frames = %lu", i, compressionBlocks[i].firstFrameId, compressionBlocks[i].size, compressionBlocksExtData[i].OffsetData, compressionBlocksExtData[i].noOfFrames);
            // vTaskDelay(pdMS_TO_TICKS(10));
        }
        // print the compressionBlockCount
        // printf("\nheader.compressionBlockCount = %u\n", header.compressionBlockCount);
        for (uint32_t i = 0; i < header.compressionBlockCount; i++)
        {
            printf("\n block number = %ld\n", i);
            fseek(file, compressionBlocksExtData[i].OffsetData, SEEK_SET);
            if (compressionBlocks[i].size > 0)
            {
                fileBuffer = realloc(fileBuffer, compressionBlocks[i].size);
                // freeMemPsram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                // ESP_LOGI(TAG, "Available free memory in PSRAM: %zu bytes", freeMemPsram);
                // freeMemInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
                // ESP_LOGI(TAG, "Available free memory in internal memory: %zu bytes", freeMemInternal);

                if (fileBuffer == NULL)
                {
                    printf("Failed to allocate memory for block_data - for index = %ld\n", i);
                    free(fileBuffer);
                    free(compressionBlocks);
                    free(compressionBlocksExtData);
                    fclose(file);
                    return ESP_FAIL;
                }
                fread(fileBuffer, 1, compressionBlocks[i].size, file);
                decompressed_size = header.channelCount * compressionBlocksExtData[i].noOfFrames;
                decompressed_sizeTemp = decompressed_size;
                decompressed_data = heap_caps_malloc(decompressed_size, MALLOC_CAP_SPIRAM);
                zlibError = 0;

                // clock_t start = clock();
                zlibError = uncompress(decompressed_data, &decompressed_sizeTemp, fileBuffer, compressionBlocks[i].size);
                // clock_t end = clock();
                // double time_taken = ((double)(end - start)) / CLOCKS_PER_SEC;

                // printf("Time taken for decompression: %f seconds\n", time_taken);
                if (zlibError == Z_OK)
                {
                    DecompressionBuffer *currentBuffer = buffers[currentBufferIndex];
                    if (currentBuffer->state == BUFFER_PROCESSING)
                    {
                        // printf("Waiting for buffer to be processed\n");
                        while (currentBuffer->state != BUFFER_PROCESSED)
                        {
                            vTaskDelay(10);
                        }
                        if (currentBuffer->data != NULL)
                        {
                            // printf("Freeing buffer data\n");
                            free(currentBuffer->data);
                        }
                        else
                        {
                            // printf("Buffer data is already NULL\n");
                        }

                        initializeBuffer(currentBuffer);
                    }
                    if (currentBuffer->state == BUFFER_PROCESSED)
                    {
                        if (currentBuffer->data != NULL)
                        {
                            printf("Freeing buffer data\n");
                            free(currentBuffer->data);
                        }
                        else
                        {
                            // printf("Buffer data is already NULL\n");
                        }
                        initializeBuffer(currentBuffer);
                    }

                    if (currentBuffer->data == NULL && currentBuffer->state == BUFFER_EMPTY)
                    {
                        initializeBuffer(currentBuffer);

                        currentBuffer->data = heap_caps_malloc(decompressed_sizeTemp, MALLOC_CAP_SPIRAM);
                        freeMemPsram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                        // ESP_LOGI(TAG, "Available free memory in PSRAM: %zu bytes", freeMemPsram);
                        freeMemInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
                        // ESP_LOGI(TAG, "Available free memory in internal memory: %zu bytes", freeMemInternal);
                        if (currentBuffer->data != NULL) // Check if malloc was successful
                        {
                            memcpy(currentBuffer->data, decompressed_data, decompressed_sizeTemp);
                            // ESP_LOGI(TAG, "Available free memory in PSRAM: %zu bytes", freeMemPsram);
                            free(decompressed_data);
                            currentBuffer->state = BUFFER_FILLED;
                            currentBuffer->noOfFrames = compressionBlocksExtData[i].noOfFrames;
                            currentBuffer->channelCount = header.channelCount;
                            currentBuffer->frameStepTime = header.frameStepTimeMillis;
                            // printf("\nBuffer %d is filled\n", currentBufferIndex);
                            currentBufferIndex = (currentBufferIndex + 1) % 4;
                        }
                        else
                        {
                            // free(decompressed_data);
                            ESP_LOGE(TAG, "Failed to allocate memory for decompressed data");
                        }
                    }

                    else
                    {
                        // print the current buffer number and current buffer state
                        printf("\nCurrent buffer number = %d\n", currentBufferIndex);
                        printf("\nCurrent buffer state = %d\n", currentBuffer->state);
                        freeMemPsram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                        ESP_LOGI(TAG, "Available free memory in PSRAM: %zu bytes", freeMemPsram);
                        free(decompressed_data);
                        decompressed_data = NULL;
                    }
                }
                else
                {
                    printf("Failed to decompress data\n");
                    free(fileBuffer);
                    free(compressionBlocks);
                    free(compressionBlocksExtData);
                    fclose(file);
                    return ESP_FAIL;
                }

                // vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
    fclose(file);
    return ESP_OK;
}