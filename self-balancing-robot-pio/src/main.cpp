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
    
    // Initialize and enable motor driver
    ESP_LOGI(TAG, "Initializing motor driver...");
    esp_err_t ret = motor_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor driver: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = motor_driver_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable motor driver: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Starting motor tests in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Main application loop
    while (1) {
        ESP_LOGI(TAG, "=== Motor Test Cycle ===");
        
        // Test Motor A forward
        ESP_LOGI(TAG, "Motor A Forward");
        motor_set_speed(MOTOR_A, MOTOR_FORWARD, 100);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_stop(MOTOR_A);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Test Motor A backward  
        ESP_LOGI(TAG, "Motor A Backward");
        motor_set_speed(MOTOR_A, MOTOR_BACKWARD, 100);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_stop(MOTOR_A);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Test Motor B forward
        ESP_LOGI(TAG, "Motor B Forward");
        motor_set_speed(MOTOR_B, MOTOR_FORWARD, 100);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_stop(MOTOR_B);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Test Motor B backward
        ESP_LOGI(TAG, "Motor B Backward");
        motor_set_speed(MOTOR_B, MOTOR_BACKWARD, 100);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_stop(MOTOR_B);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Test both motors forward
        ESP_LOGI(TAG, "Both Motors Forward");
        motor_set_speed(MOTOR_A, MOTOR_FORWARD, 80);
        motor_set_speed(MOTOR_B, MOTOR_FORWARD, 80);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Test both motors backward
        ESP_LOGI(TAG, "Both Motors Backward");
        motor_set_speed(MOTOR_A, MOTOR_BACKWARD, 80);
        motor_set_speed(MOTOR_B, MOTOR_BACKWARD, 80);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_stop_all();
        
        ESP_LOGI(TAG, "Test cycle complete. Waiting 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}