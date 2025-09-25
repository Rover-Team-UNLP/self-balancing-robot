#include "main.h"

static const char *TAG = "SELF_BALANCING_ROBOT";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Self-Balancing Robot Starting...");
    
    // Initialize system
    ESP_LOGI(TAG, "ESP32 Chip info:");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 with %d CPU cores, WiFi%s%s",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    
    ESP_LOGI(TAG, "Silicon revision %d", chip_info.revision);
    
    // Get flash size using modern API
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "%dMB %s flash", flash_size / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
    ESP_LOGI(TAG, "Free heap size: %d", esp_get_free_heap_size());
    
    // Main application loop
    while (1) {
        ESP_LOGI(TAG, "Self-Balancing Robot is running...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay 5 seconds
    }
}