#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"
#include "esp_intr_alloc.h"
#include "extractor.h"
#include "models.c"
#include "led_glow.h"
#include "buffer_manager.h"
#include "common.h"
#include "task_handles.h"
#include "correction_arrays.h"
#include <math.h>

#define MOSI_GPIO 15
#define MISO_GPIO 2
#define CLK_GPIO 14
#define CS_GPIO 13

// #define MOSI_GPIO 23
// #define MISO_GPIO 19
// #define CLK_GPIO 18
// #define CS_GPIO 4

#define PIN_NUM_MISO MISO_GPIO
#define PIN_NUM_MOSI MOSI_GPIO
#define PIN_NUM_CLK CLK_GPIO
#define PIN_NUM_CS CS_GPIO

// #define START_FILENAME_INDEX 0
// #define NO_OF_FILES -1 // -1 indicated infinite number of files
// #define BYTES_PER_PACKET 50
// #define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

sdmmc_card_t *card;

port_data *ports[MAX_PORTS];
static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"
const char mount_point[] = MOUNT_POINT;
volatile uint8_t sdCardFormatted = false;

TaskHandle_t readFileTaskHandle = NULL;
TaskHandle_t ledGlowTaskHandle = NULL;
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)


ShowData *shows = NULL;
int globalShowCount = 0;
int buttonNumber = 1;
bool stop = false;

unsigned char gammaCorrection[MAX_PORTS][MAX_VALUES];
unsigned char brightnessAdjustment[MAX_PORTS][MAX_VALUES];

// Function to read and parse the shows file
void readShowsFile(const char *filePath)
{
    FILE *file = fopen(filePath, "r");
    if (!file)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    char line[200];
    int count = 0;

    while (fgets(line, sizeof(line), file))
    {
        ShowData *temp = realloc(shows, sizeof(ShowData) * (count + 1));
        if (!temp)
        {
            ESP_LOGE(TAG, "Failed to allocate memory");
            free(shows);
            fclose(file);
            return;
        }
        shows = temp;

        char repeatStr[6];
        char nextShowNameStr[50]; // Temporary buffer for the next show name
        int fieldsParsed = sscanf(line, "Show: %d FSEQ: %s repeat: %s next: %s", &shows[count].showNumber, shows[count].fseqFileName, repeatStr, nextShowNameStr);
        if (fieldsParsed != 4)
        {
            ESP_LOGW(TAG, "Line format incorrect, skipping: %s", line);
            continue;
        }

        // Convert repeatStr to boolean
        shows[count].repeat = (strcmp(repeatStr, "false") == 0) ? false : true;

        // Extract the number from nextShowNameStr and store it in nextShowNumber
        int nextShowNumber;
        if (sscanf(nextShowNameStr, "Show%d", &nextShowNumber) == 1)
        {
            shows[count].nextShowNumber = nextShowNumber;
        }
        else
        {
            ESP_LOGW(TAG, "Failed to parse next show number from: %s", nextShowNameStr);
            shows[count].nextShowNumber = -1; // Use -1 or another sentinel value to indicate failure
        }

        // Initialize the show state to SHOW_NOT_COMPLETE
        shows[count].state = SHOW_NOT_COMPLETE;

        count++;
    }

    fclose(file);
    globalShowCount = count; // Update the global show count
}

void read_config_file(const char *config_file)
{
    struct stat st;
    if (stat(config_file, &st) == 0)
    {
        printf("File %s found\n", config_file);

        FILE *file = fopen(config_file, "r");
        if (file == NULL)
        {
            printf("Failed to open config file\n");
            return;
        }

        // Read the config file line by line
        char line[1024];
        int port;
        int pixelCount;
        int startChannel;
        char colorOrder[4];
        int nulls;
        int brightness;
        float gamma;
        int num_ports = 0;
        while (fgets(line, sizeof(line), file))
        {
            // Print the line
            printf("Line: %s\n", line);

            sscanf(line, "Port: %d PixelCount: %d StartChannel: %d ColorOrder: %3s Nulls: %d Brightness%%: %d Gamma: %f",
                   &port, &pixelCount, &startChannel, colorOrder, &nulls, &brightness, &gamma);

            // Allocate and fill a port_data structure
            ports[num_ports] = (port_data *)malloc(sizeof(port_data));
            if (ports[num_ports] == NULL)
            {
                printf("Failed to allocate memory for port %d\n", num_ports);
                return;
            }
            ports[num_ports]->port = port;
            ports[num_ports]->pixelCount = pixelCount;
            ports[num_ports]->startChannel = startChannel;
            strcpy(ports[num_ports]->colorOrder, colorOrder);
            ports[num_ports]->nulls = nulls;
            ports[num_ports]->brightness = brightness;
            ports[num_ports]->gamma = gamma;
            num_ports++;
        }

        // Close the config file
        fclose(file);
    }
    else
    {
        printf("File %s not found\n", config_file);
    }
}

void SD_Init(void)
{
    sdCardFormatted = false;
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;

    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // host.max_freq_khz = 10000;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        sdCardFormatted = false;
        return;
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        sdCardFormatted = false;
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    sdCardFormatted = true;
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

static void read_file_task(void *param)
{
    int currentShowIndex = buttonNumber - 1; // Start with the first show as indicated by buttonNum
    ShowData *currentShow = NULL;

    while (1)
    {
        if (currentShowIndex >= 0 && currentShowIndex < globalShowCount)
        {
            currentShow = &shows[currentShowIndex];
            if (currentShow->state == SHOW_NOT_COMPLETE)
            {

                const char *ghost_file = currentShow->fseqFileName;
                char *formatted_ghost_file = malloc(strlen(MOUNT_POINT) + strlen(ghost_file) + 1);
                if (formatted_ghost_file == NULL)
                {
                    ESP_LOGE(TAG, "Failed to allocate memory for formatted ghost file name");
                    return; // Exit the task if memory allocation fails
                }
                sprintf(formatted_ghost_file, "%s/%s", MOUNT_POINT, ghost_file);

                esp_err_t ret = readFileParse(formatted_ghost_file); // Use the formatted file name
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to read file %s", formatted_ghost_file);
                    free(formatted_ghost_file); // Free memory before returning
                    return;
                }
                else
                {
                    printf("The file %s read successfully\n", formatted_ghost_file);
                }
                free(formatted_ghost_file); // Free memory after use
                // Check if the show should be repeated
                while (currentShow->repeat && !stop)
                {
                    printf("Repeating the show\n");
                    vTaskDelay(pdMS_TO_TICKS(100)); // Delay before repeating

                    formatted_ghost_file = malloc(strlen(MOUNT_POINT) + strlen(ghost_file) + 1);
                    if (formatted_ghost_file == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for formatted ghost file name on repeat");
                        return;
                    }
                    sprintf(formatted_ghost_file, "%s/%s", MOUNT_POINT, ghost_file);

                    ret = readFileParse(formatted_ghost_file);
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to repeat file %s", formatted_ghost_file);
                    }
                    else
                    {
                        printf("The file %s read successfully on repeat\n", formatted_ghost_file);
                    }
                    free(formatted_ghost_file); // Free memory after use in the repeat case
                }
                if (currentShow->nextShowNumber == -1)
                {
                    printf("Sab kuch maja ma!!!!!\n");
                    currentShow->state = SHOW_COMPLETE;
                }
                else
                {
                    currentShowIndex = currentShow->nextShowNumber - 1; // Adjust for 0-based indexing
                }
            }
            else
            {
                // printf("Show %d is already complete\n", currentShow->showNumber);
            }
        }
        else
        {
            printf("Invalid show index %d\n", currentShowIndex);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before processing the next show
    }

    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    ESP_LOGI(TAG, "Card unmounted");
    // vTaskDelete(NULL); // Optionally terminate this task
}

void printCorrectionArrays()
{
    for (int port = 0; port < MAX_PORTS; port++)
    {
        printf("Port %d:\n", port);

        // Print gamma correction values
        printf("  Gamma Correction:\n    ");
        for (int i = 0; i < MAX_VALUES; i++)
        {
            printf("%d ", gammaCorrection[port][i]);
            if ((i + 1) % 16 == 0) // New line every 16 values for better readability
                printf("\n    ");
        }

        // Print brightness adjustment values
        printf("  Brightness Adjustment:\n    ");
        for (int i = 0; i < MAX_VALUES; i++)
        {
            printf("%d ", brightnessAdjustment[port][i]);
            if ((i + 1) % 16 == 0) // New line every 16 values for better readability
                printf("\n    ");
        }

        printf("\n"); // Extra newline for spacing between ports
    }
}

void populateCorrectionArrays()
{
    for (int port = 0; port < MAX_PORTS; port++)
    {
        if (ports[port] != NULL)
        { // Ensure the port is configured
            float gamma = ports[port]->gamma;
            int brightness = ports[port]->brightness;

            for (int i = 0; i < MAX_VALUES; i++)
            {
                // Gamma correction calculation
                float gammaCorrectedValue = pow(i / 255.0, gamma) * 255.0 + 0.5;
                gammaCorrection[port][i] = gammaCorrectedValue > 255 ? 255 : (unsigned char)gammaCorrectedValue;

                // Brightness adjustment calculation
                float brightnessAdjustedValue = i * (brightness / 100.0);
                brightnessAdjustment[port][i] = brightnessAdjustedValue > 255 ? 255 : (unsigned char)brightnessAdjustedValue;
            }
        }
    }
}


static esp_err_t sdCard_read_dir(const char *dirpath)
{
    char entrypath[FILE_PATH_MAX];
    char entrysize[16];
    const char *entrytype;
    struct dirent *entry;
    struct stat entry_stat;
    char tempString[1000];
    if(sdCardFormatted == true)
    {
        DIR *dir = opendir(dirpath);
        const size_t dirpath_len = strlen(dirpath);

        /* Retrieve the base path of file storage to construct the full path */
        strlcpy(entrypath, dirpath, sizeof(entrypath));

        if (!dir) {
            ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
            /* Respond with 404 Not Found */
            //httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
            return ESP_FAIL;
        }
        while ((entry = readdir(dir)) != NULL) {
            entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

            strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
            if (stat(entrypath, &entry_stat) == -1) {
                ESP_LOGE(TAG, "Failed to stat %s : %s %s", entrytype, entry->d_name, entrypath);
                continue;
            }
            sprintf(entrysize, "%ld", entry_stat.st_size);
            #ifdef CONSOLE_UART
                printf("Found %s : %s Size : (%s bytes)", entrytype, entry->d_name, entrysize);
            #else
                sprintf(tempString,"\n\rFound %s : %s Size : (%s bytes)", entrytype, entry->d_name, entrysize);
                printf("%s",tempString);
                //uart_write_bytes(UART_PORT_NUM, (const char *) tempString, strlen(tempString));
            #endif
        }
        closedir(dir);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "SD Card not formatted");
        return ESP_FAIL;
    }

}

void app_main()
{

    decompressionBufferQueue = xQueueCreate(5, sizeof(DecompressionBuffer *));
    SD_Init();
    const char *config_file = MOUNT_POINT "/ConfigNew.txt";
    read_config_file(config_file);
    populateCorrectionArrays();
    // printCorrectionArrays();

    readShowsFile(MOUNT_POINT "/Shows.txt");
    configure_led_channels();
    // sdCard_read_dir(MOUNT_POINT"/");


    // write the loop to show the shows
    // for (int i = 0; i < globalShowCount; i++)
    // {
    //     printf("Show: %d FSEQ: %s repeat: %d next: %d state: %d\n", shows[i].showNumber, shows[i].fseqFileName, shows[i].repeat, shows[i].nextShowNumber, shows[i].state);
    // }

    // return;
    xTaskCreatePinnedToCore(read_file_task, "ReadFileTask", 4096 * 5, NULL, 5, &readFileTaskHandle, 1);
    // // while(1);
    // //  vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreatePinnedToCore(led_glow, "LedGlow", 4096 * 5, NULL, 7, &ledGlowTaskHandle, 0);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
