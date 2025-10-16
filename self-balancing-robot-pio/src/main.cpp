#include "main.h"

static const char *TAG = "MPU6050_TEST";

extern "C" void app_main(void)
{
    // Print immediately to verify serial is working
    printf("\n\n==================================\n");
    printf("MPU6050 Test Starting...\n");
    printf("==================================\n\n");
    
    ESP_LOGI(TAG, "MPU6050 Gyroscope Test Starting...");
    
    // Initialize system info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("ESP32-WROOM-32 with %d CPU cores\n", chip_info.cores);
    ESP_LOGI(TAG, "ESP32-WROOM-32 with %d CPU cores", chip_info.cores);
    
    // Initialize MPU6050
    printf("Initializing MPU6050...\n");
    ESP_LOGI(TAG, "Initializing MPU6050...");
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        printf("\n*** ERROR: Failed to initialize MPU6050: %s ***\n", esp_err_to_name(ret));
        printf("System halted. Check I2C connections:\n");
        printf("  SDA -> GPIO21\n");
        printf("  SCL -> GPIO22\n");
        printf("  VCC -> 3.3V\n");
        printf("  GND -> GND\n\n");
        
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted. Check I2C connections:");
        ESP_LOGE(TAG, "  SDA -> GPIO21");
        ESP_LOGE(TAG, "  SCL -> GPIO22");
        ESP_LOGE(TAG, "  VCC -> 3.3V");
        ESP_LOGE(TAG, "  GND -> GND");
        
        while(1) {
            printf("Waiting... (Press RESET to try again)\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    
    printf("MPU6050 initialized successfully!\n");
    ESP_LOGI(TAG, "MPU6050 initialized successfully!");
    
    // Optional: Calibrate sensor
    printf("\nStarting calibration in 3 seconds...\n");
    printf("Place the sensor on a flat, stable surface!\n");
    ESP_LOGI(TAG, "Starting calibration in 3 seconds...");
    ESP_LOGI(TAG, "Place the sensor on a flat, stable surface!");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ret = mpu6050_calibrate(500);
    if (ret != ESP_OK) {
        printf("WARNING: Calibration failed, continuing without calibration\n");
        ESP_LOGW(TAG, "Calibration failed, continuing without calibration");
    }
    
    printf("\nStarting continuous data reading...\n");
    printf("Data format: Accel(X,Y,Z) Gyro(X,Y,Z) Temp Pitch Roll\n\n");
    ESP_LOGI(TAG, "Starting continuous data reading...");
    ESP_LOGI(TAG, "Data format: Accel(X,Y,Z) Gyro(X,Y,Z) Temp Pitch Roll");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Main loop - read and display sensor data
    uint32_t sample_count = 0;
    while (1) {
        mpu6050_data_t data;
        float pitch, roll;
        
        // Read sensor data
        ret = mpu6050_read_data(&data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MPU6050 data");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Calculate angles
        ret = mpu6050_get_angles(&pitch, &roll);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calculate angles");
        }
        
        // Print data to serial (every sample)
        printf("[%lu] ", sample_count++);
        printf("Accel: X=%6.2f Y=%6.2f Z=%6.2f m/s² | ",
               data.accel_x, data.accel_y, data.accel_z);
        printf("Gyro: X=%7.2f Y=%7.2f Z=%7.2f °/s | ",
               data.gyro_x, data.gyro_y, data.gyro_z);
        printf("Temp: %5.1f°C | ", data.temp);
        printf("Pitch: %6.2f° Roll: %6.2f°\n", pitch, roll);
        
        // Also log with ESP_LOGI every 20 samples for cleaner output
        if (sample_count % 20 == 0) {
            ESP_LOGI(TAG, "Sample %lu - Pitch: %.2f° Roll: %.2f° GyroZ: %.2f°/s", 
                     sample_count, pitch, roll, data.gyro_z);
        }
        
        // Read at ~50Hz (20ms delay)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}