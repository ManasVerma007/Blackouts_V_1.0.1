idf_component_register(SRCS "app_main.c" "led_strip_encoder.c" "extractor.c" "models.c" "led_glow.c" "buffer_manager.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_timer fatfs vfs sdmmc console nvs_flash mbedtls spiffs)
