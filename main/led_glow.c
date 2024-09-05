#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "FreeRTOSConfig.h"
#include "led_strip_encoder.h"
#include "esp_timer.h"
#include "extractor.h"
#include "led_glow.h"
#include "buffer_manager.h"
#include "models.c"
#include "math.h"
#include "freertos/queue.h"
#include "task_handles.h"
#include "correction_arrays.h"
#include <time.h>

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define NUM_STRIPS 8
uint8_t totalBuffers = 4;
#define MAX_PORTS 8
static int blockCount = 0;
// extern port_data *ports[MAX_PORTS];
static uint32_t frame_no = 0;

static const char *TAG = "LED";

DecompressionBuffer *ledBuffers[4] = {&buffer1, &buffer2, &buffer3, &buffer4};

// These are already in the global scope, accessible by both functions
rmt_channel_handle_t led_chans[NUM_STRIPS];
rmt_encoder_handle_t led_encoders[NUM_STRIPS];
int RMT_LED_STRIP_GPIO_NUM[NUM_STRIPS] = {4, 12, 19, 5, 0, 27, 32, 33}; // replace with your GPIO numbers
TickType_t startTick, endTick;
uint32_t ticksTaken;
void configure_led_channels(void)
{
    esp_err_t err;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };

    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };

    // Initialize RMT TX channels for each LED strip
    for (int i = 0; i < NUM_STRIPS; i++)
    {
        tx_chan_config.gpio_num = ports[i]->port; // Use predefined GPIO numbers
        err = rmt_new_tx_channel(&tx_chan_config, &led_chans[i]);
        if (err != ESP_OK)
        {
            printf("Failed to create RMT TX channel %d, GPIO %d\n", i, ports[i]->port);
            return; // Exit if channel creation fails
        }
        printf("RMT TX channel %d, GPIO %d created\n", i, ports[i]->port);
    }

    // Install LED strip encoder
    for (int i = 0; i < NUM_STRIPS; i++)
    {
        err = rmt_new_led_strip_encoder(&encoder_config, &led_encoders[i]);
        if (err != ESP_OK)
        {
            ESP_LOGE("LED_INIT", "Failed to install led strip encoder for channel %d", i);
            return; // Exit if encoder installation fails
        }
    }
    ESP_LOGI("LED_INIT", "Install led strip encoder");

    // Enable RMT TX channels
    for (int i = 0; i < NUM_STRIPS; i++)
    {
        err = rmt_enable(led_chans[i]);
        if (err != ESP_OK)
        {
            ESP_LOGE("LED_INIT", "Failed to enable RMT TX channel %d", i);
            return; // Exit if enabling the channel fails
        }
    }
    ESP_LOGI("LED_INIT", "Enable RMT TX channels");

    printf("Everything initialized\n");
}

void led_glow(void *param)
{
    bool bufferArrayFull = false;
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    TickType_t last_wake_time = xTaskGetTickCount(); // Initialize the last wake time

    while (1)
    {

        if (bufferArrayFull == true)
        {

            for (int i = 0; i < totalBuffers; i++)
            {
                // get tick count

                DecompressionBuffer *currentBuffer = ledBuffers[i];
                if (currentBuffer->state == BUFFER_PROCESSED)
                {
                    vTaskDelay((2));
                }
                else if (currentBuffer->state == BUFFER_FILLED)
                {
                    startTick = xTaskGetTickCount();
                    unsigned char *frameData = currentBuffer->data;
                    uint32_t totalChannels = currentBuffer->channelCount;
                    uint32_t totalFrames = currentBuffer->noOfFrames;
                    uint32_t ledsPerStrip = totalChannels / (NUM_STRIPS * 3);
                    uint32_t frameStepTime = currentBuffer->frameStepTime;
                    currentBuffer->state = BUFFER_PROCESSING;

                    for (uint32_t frame = 0; frame < totalFrames; frame++)
                    {
                        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(frameStepTime));

                        for (int strip = 0; strip < NUM_STRIPS; strip++)
                        {

                            int start_index = (ports[strip]->startChannel);
                            int end_index = (start_index + 3 * (ports[strip]->pixelCount));
                            int strip_data_size = (end_index - start_index);
                            unsigned char *reorderedData = malloc(strip_data_size);
                            if (!reorderedData)
                            {
                                printf("Failed to allocate memory for reordered data\n");
                                continue;
                            }

                            for (int i = 0; i < strip_data_size; i += 3)
                            {
                                if (i + 2 < strip_data_size)
                                {

                                    unsigned char r = frameData[start_index + i];
                                    unsigned char g = frameData[start_index + i + 1];
                                    unsigned char b = frameData[start_index + i + 2];
                                    r = brightnessAdjustment[strip][r];
                                    r = gammaCorrection[strip][r];
                                    g = brightnessAdjustment[strip][g];
                                    g = gammaCorrection[strip][g];
                                    b = brightnessAdjustment[strip][b];
                                    b = gammaCorrection[strip][b];
                                    reorderedData[i] = g;     // G
                                    reorderedData[i + 1] = r; // R`
                                    reorderedData[i + 2] = b; // B
                                }
                            }

                            vTaskSuspend(readFileTaskHandle);

                            ESP_ERROR_CHECK(rmt_transmit(led_chans[strip], led_encoders[strip], reorderedData, strip_data_size, &tx_config));
                            // ESP_ERROR_CHECK(rmt_transmit(led_chans[strip], led_encoders[strip], &frameData[start_index], strip_data_size, &tx_config));

                            vTaskDelay(2);
                            vTaskResume(readFileTaskHandle);
                            free(reorderedData);
                        }
                        frameData += totalChannels;
                        // for (int i = 0; i < 8; i++)
                        // {
                        //     ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chans[i], portMAX_DELAY));
                        // }
                    }
                    endTick = xTaskGetTickCount();
                    ticksTaken = endTick - startTick;
                    printf("block count: %d is parsed \n", blockCount);
                    blockCount++;
                    currentBuffer->state = BUFFER_PROCESSED;
                    printf("ticks taken: %lu \n", ticksTaken);
                }
                else
                {
                    vTaskDelay((2));
                }
            }
        }
        else
        {
            DecompressionBuffer *checkBuffer = ledBuffers[3];
            if (checkBuffer->state == BUFFER_FILLED)
            {
                bufferArrayFull = true;
                printf("ha yaha aaya \n");
            }
        }
    }
}

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/rmt_tx.h"
// #include "FreeRTOSConfig.h"
// #include "led_strip_encoder.h"
// #include "esp_timer.h"
// #include "extractor.h"
// #include "led_glow.h"
// #include "buffer_manager.h"
// #include "models.c"
// #include "math.h"
// #include "freertos/queue.h"
// #include "task_handles.h"
// #include "correction_arrays.h"
// #include <time.h>

// #define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
// #define NUM_STRIPS 8
// uint8_t totalBuffers = 4;
// #define MAX_PORTS 8

// // extern port_data *ports[MAX_PORTS];
// static uint32_t frame_no = 0;

// DecompressionBuffer *ledBuffers[4] = {&buffer1, &buffer2, &buffer3, &buffer4};

// // These are already in the global scope, accessible by both functions
// rmt_channel_handle_t led_chans[NUM_STRIPS];
// rmt_encoder_handle_t led_encoders[NUM_STRIPS];
// int RMT_LED_STRIP_GPIO_NUM[NUM_STRIPS] = {4, 12, 19, 5, 0, 27, 32, 33}; // replace with your GPIO numbers

// void configure_led_channels(void)
// {
//     esp_err_t err;
//     rmt_tx_channel_config_t tx_chan_config = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .mem_block_symbols = 64,
//         .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
//         .trans_queue_depth = 4,
//     };

//     led_strip_encoder_config_t encoder_config = {
//         .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
//     };

//     // Initialize RMT TX channels for each LED strip
//     for (int i = 0; i < NUM_STRIPS; i++)
//     {
//         tx_chan_config.gpio_num = ports[i]->port; // Use predefined GPIO numbers
//         err = rmt_new_tx_channel(&tx_chan_config, &led_chans[i]);
//         if (err != ESP_OK)
//         {
//             printf("Failed to create RMT TX channel %d, GPIO %d\n", i, ports[i]->port);
//             return; // Exit if channel creation fails
//         }
//         printf("RMT TX channel %d, GPIO %d created\n", i, ports[i]->port);
//     }

//     // Install LED strip encoder
//     for (int i = 0; i < NUM_STRIPS; i++)
//     {
//         err = rmt_new_led_strip_encoder(&encoder_config, &led_encoders[i]);
//         if (err != ESP_OK)
//         {
//             ESP_LOGE("LED_INIT", "Failed to install led strip encoder for channel %d", i);
//             return; // Exit if encoder installation fails
//         }
//     }
//     ESP_LOGI("LED_INIT", "Install led strip encoder");

//     // Enable RMT TX channels
//     for (int i = 0; i < NUM_STRIPS; i++)
//     {
//         err = rmt_enable(led_chans[i]);
//         if (err != ESP_OK)
//         {
//             ESP_LOGE("LED_INIT", "Failed to enable RMT TX channel %d", i);
//             return; // Exit if enabling the channel fails
//         }
//     }
//     ESP_LOGI("LED_INIT", "Enable RMT TX channels");

//     printf("Everything initialized\n");
// }

// void led_glow(void *param)
// {
//     bool bufferArrayFull = false;
//     rmt_transmit_config_t tx_config = {
//         .loop_count = 0,
//     };
//     while (1)
//     {
//         if (bufferArrayFull == true)
//         {
//             for (int i = 0; i < totalBuffers; i++)
//             {
//                 DecompressionBuffer *currentBuffer = ledBuffers[i];
//                 if (currentBuffer->state == BUFFER_PROCESSED)
//                 {
//                     vTaskDelay(pdMS_TO_TICKS(10));
//                     continue;
//                 }
//                 else if (currentBuffer->state == BUFFER_FILLED)
//                 {
//                     unsigned char *frameData = currentBuffer->data;
//                     uint32_t totalChannels = currentBuffer->channelCount;
//                     uint32_t totalFrames = currentBuffer->noOfFrames;
//                     uint32_t ledsPerStrip = totalChannels / (NUM_STRIPS * 3); // Assuming equal distribution of LEDs across strips
//                     uint32_t frameStepTime = currentBuffer->frameStepTime;
//                     currentBuffer->state = BUFFER_PROCESSING;
//                     for (uint32_t frame = 0; frame < totalFrames; frame++)
//                     {
//                         for (int strip = 0; strip < NUM_STRIPS; strip++)
//                         {
//                             int start_index = (ports[strip]->startChannel) - 1;
//                             int end_index = (start_index + 3 * (ports[strip]->pixelCount));
//                             int strip_data_size = (end_index - start_index);
//                             vTaskSuspend(readFileTaskHandle);
//                             // ESP_ERROR_CHECK(rmt_transmit(led_chans[strip], led_encoders[strip], reorderedData, strip_data_size, &tx_config));
//                             ESP_ERROR_CHECK(rmt_transmit(led_chans[strip], led_encoders[strip], &frameData[start_index], strip_data_size, &tx_config));
//                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chans[strip], portMAX_DELAY));
//                             vTaskResume(readFileTaskHandle);
//                         }
//                         frameData += totalChannels;
//                         // vTaskDelay(pdMS_TO_TICKS(20));
//                     }
//                     currentBuffer->state = BUFFER_PROCESSED;
//                 }
//                 else
//                 {
//                     vTaskDelay(pdMS_TO_TICKS(10));
//                 }
//             }
//         }
//         else
//         {
//             DecompressionBuffer *checkBuffer = ledBuffers[3];
//             if (checkBuffer->state == BUFFER_FILLED)
//             {
//                 bufferArrayFull = true;
//                 printf("ha yaha aaya \n");
//             }
//             else
//             {
//             }
//             vTaskDelay(pdMS_TO_TICKS(10)); // Wait before checking again
//         }
//     }
// }
