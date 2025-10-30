#include "main.h"
#include "web_interface.h"


static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
    printf("\n========================================\n");
    printf("Self-Balancing Robot - Control System\n");
    printf("========================================\n\n");
    
    ESP_LOGI(TAG, "Initializing system...");
    
    // Inicializar interfaz web primero
    ESP_LOGI(TAG, "Starting web interface...");
    if (web_interface_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize web interface!");
        // Continuar de todos modos
    }
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "📱 Web Interface Ready!");
    ESP_LOGI(TAG, "Connect to WiFi: ESP32-Balance-Robot");
    ESP_LOGI(TAG, "Password: robot123");
    ESP_LOGI(TAG, "Open browser: http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    
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
    
    // Calibrar giroscopio y acelerómetro del MPU
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== MPU CALIBRATION ==========");
    ESP_LOGI(TAG, "Place robot HORIZONTAL (lying down)");
    ESP_LOGI(TAG, "Keep it STILL for calibration...");
    ESP_LOGI(TAG, "====================================");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    if (mpu6050_calibrate(500) != ESP_OK) {
        ESP_LOGE(TAG, "MPU calibration failed!");
        return;
    }
    ESP_LOGI(TAG, "MPU calibration complete");
    
    // Inicializar sistema de control
    ESP_LOGI(TAG, "Initializing balance control...");
    if (balance_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize balance control!");
        return;
    }
    
    // Calibrar offset del ángulo (con robot VERTICAL)
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "====== ANGLE OFFSET CALIBRATION ======");
    ESP_LOGI(TAG, "NOW place robot in UPRIGHT position");
    ESP_LOGI(TAG, "This will set the zero angle reference");
    ESP_LOGI(TAG, "=====================================");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    if (balance_control_calibrate_offset() != ESP_OK) {
        ESP_LOGE(TAG, "Angle offset calibration failed!");
        return;
    }
    
    // Esperar antes de iniciar control activo
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== READY TO START ==========");
    ESP_LOGI(TAG, "Starting balance control in 2 seconds...");
    ESP_LOGI(TAG, "Keep robot upright!");
    ESP_LOGI(TAG, "====================================");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
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
