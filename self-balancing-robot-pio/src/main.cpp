#include "main.h"

static const char *TAG = "SENSOR_TEST";

extern "C" void app_main(void)
{
    // Print immediately to verify serial is working
    printf("\n\n========================================\n");
    printf("Self-Balancing Robot - Sensor Test\n");
    printf("========================================\n\n");
    
    ESP_LOGI(TAG, "Self-Balancing Robot Sensor Test Starting...");
    
    // Initialize system info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("ESP32-WROOM-32 with %d CPU cores\n", chip_info.cores);
    ESP_LOGI(TAG, "ESP32-WROOM-32 with %d CPU cores", chip_info.cores);
    
    // Initialize MPU6xxx
    printf("\n--- Initializing MPU6xxx (Gyro/Accel) ---\n");
    ESP_LOGI(TAG, "Initializing MPU6xxx...");
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        printf("\n*** ERROR: Failed to initialize MPU6xxx: %s ***\n", esp_err_to_name(ret));
        printf("System halted. Check I2C connections:\n");
        printf("  SDA -> GPIO21\n");
        printf("  SCL -> GPIO22\n");
        printf("  VCC -> 3.3V\n");
        printf("  GND -> GND\n\n");
        
        ESP_LOGE(TAG, "Failed to initialize MPU6xxx: %s", esp_err_to_name(ret));
        
        while(1) {
            printf("Waiting... (Press RESET to try again)\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    
    // Initialize Encoders
    printf("\n--- Initializing Encoders ---\n");
    ESP_LOGI(TAG, "Initializing Encoders...");
    ret = encoder_init();
    if (ret != ESP_OK) {
        printf("\n*** ERROR: Failed to initialize encoders: %s ***\n", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Failed to initialize encoders: %s", esp_err_to_name(ret));
        
        while(1) {
            printf("Waiting... (Press RESET to try again)\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    
    printf("\nAll sensors initialized successfully!\n");
    ESP_LOGI(TAG, "All sensors initialized successfully!");
    
    // Calibrate MPU (optional - skip for now to test faster)
    printf("\n--- Skipping MPU calibration for faster testing ---\n");
    printf("(You can calibrate later if needed)\n");
    
    printf("\n========================================\n");
    printf("Starting continuous sensor reading...\n");
    printf("Rotate the motors manually to see encoder changes\n");
    printf("========================================\n\n");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Main loop - read and display sensor data
    uint32_t sample_count = 0;
    while (1) {
        // Update encoder velocities
        encoder_update_all_velocities();
        
        // Read MPU data
        mpu6050_data_t mpu_data;
        float pitch, roll;
        
        ret = mpu6050_read_data(&mpu_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MPU data");
        }
        
        ret = mpu6050_get_angles(&pitch, &roll);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calculate angles");
        }
        
        // Read encoder data
        encoder_data_t enc_a_data, enc_b_data;
        encoder_get_data(ENCODER_MOTOR_A, &enc_a_data);
        encoder_get_data(ENCODER_MOTOR_B, &enc_b_data);
        
        // Print consolidated data
        printf("[%lu] ", sample_count++);
        printf("MPU: Pitch=%6.2f° Roll=%6.2f° GyroZ=%7.2f°/s | ",
               pitch, roll, mpu_data.gyro_z);
        printf("EncA: Cnt=%6ld Vel=%6.1fRPM Dir=%s | ",
               enc_a_data.count, enc_a_data.velocity_rpm,
               enc_a_data.direction_forward ? "FWD" : "REV");
        printf("EncB: Cnt=%6ld Vel=%6.1fRPM Dir=%s\n",
               enc_b_data.count, enc_b_data.velocity_rpm,
               enc_b_data.direction_forward ? "FWD" : "REV");
        
        // Also log summary every 20 samples
        if (sample_count % 20 == 0) {
            ESP_LOGI(TAG, "=== Sample %lu ===", sample_count);
            ESP_LOGI(TAG, "MPU: Pitch=%.2f° Roll=%.2f°", pitch, roll);
            ESP_LOGI(TAG, "Encoder A: Count=%ld, RPM=%.1f, Dir=%s", 
                     enc_a_data.count, enc_a_data.velocity_rpm,
                     enc_a_data.direction_forward ? "FWD" : "REV");
            ESP_LOGI(TAG, "Encoder B: Count=%ld, RPM=%.1f, Dir=%s",
                     enc_b_data.count, enc_b_data.velocity_rpm,
                     enc_b_data.direction_forward ? "FWD" : "REV");
        }
        
        // Read at ~20Hz (50ms delay)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}