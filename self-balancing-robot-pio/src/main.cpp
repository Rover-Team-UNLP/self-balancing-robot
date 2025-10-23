#include "main.h"


static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
    printf("\n========================================\n");
    printf("Self-Balancing Robot - Control System\n");
    printf("========================================\n\n");
    
    ESP_LOGI(TAG, "Initializing system...");
    
    // Inicializar sensores
    ESP_LOGI(TAG, "Initializing MPU6050...");
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050!");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing encoders...");
    if (encoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoders!");
        return;
    }
    
    // Calibrar MPU (mantener el robot quieto y nivelado)
    ESP_LOGI(TAG, "Calibrating MPU6050... Keep robot still!");
    vTaskDelay(pdMS_TO_TICKS(2000));
    mpu6050_calibrate(500);
    
    // Inicializar sistema de control
    ESP_LOGI(TAG, "Initializing balance control...");
    if (balance_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize balance control!");
        return;
    }
    
    // Esperar antes de iniciar
    ESP_LOGI(TAG, "Starting balance control in 3 seconds...");
    ESP_LOGI(TAG, "Place robot in upright position!");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Iniciar sistema de control
    if (balance_control_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start balance control!");
        return;
    }
    
    ESP_LOGI(TAG, "Balance control system running!");
    ESP_LOGI(TAG, "Robot should now try to maintain balance");
    
    // Mantener main task viva
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}