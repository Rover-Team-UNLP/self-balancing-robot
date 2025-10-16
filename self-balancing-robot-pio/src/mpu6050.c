#include "mpu6050.h"

#include <math.h>
#include <string.h>

static const char *TAG = "MPU6050";
static bool initialized = false;
static mpu6050_gyro_range_t current_gyro_range = MPU6050_GYRO_RANGE_250DPS;
static mpu6050_accel_range_t current_accel_range = MPU6050_ACCEL_RANGE_2G;
static mpu6050_calibration_t calibration = {0};

static float gyro_scale = 131.0;
static float accel_scale = 16384.0;

static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(MPU6050_I2C_PORT, MPU6050_ADDR, write_buf, sizeof(write_buf),
                                      pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(MPU6050_I2C_PORT, MPU6050_ADDR, &reg, 1, data, len,
                                        pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read_byte(uint8_t reg, uint8_t *data) {
    return mpu6050_read_bytes(reg, data, 1);
}

static void i2c_scanner(void) {
    printf("\n");
    printf("Scanning I2C bus...\n");
    uint8_t devices_found = 0;

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("  Device found at address 0x%02X\n", addr);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        printf("  No I2C devices found!\n");
        printf("  Check connections:\n");
        printf("    - SDA to GPIO21\n");
        printf("    - SCL to GPIO22\n");
        printf("    - VCC to 3.3V\n");
        printf("    - GND to GND\n");
    } else {
        printf("Total devices found: %d\n", devices_found);
    }
    printf("\n");
}

esp_err_t mpu6050_init(void) {
    printf("Initializing MPU6xxx sensor...\n");
    ESP_LOGI(TAG, "Initializing MPU6xxx sensor (MPU6050/6500/9250)...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA_PIN,
        .scl_io_num = MPU6050_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(MPU6050_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return ret;
    }

    ret = i2c_driver_install(MPU6050_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_scanner();

    ret = mpu6050_test_connection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not found on I2C bus");
        return ret;
    }

    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;

    // Configure gyroscope range (±250°/s)
    ret = mpu6050_set_gyro_range(MPU6050_GYRO_RANGE_250DPS);
    if (ret != ESP_OK) return ret;

    // Configure accelerometer range (±2g)
    ret = mpu6050_set_accel_range(MPU6050_ACCEL_RANGE_2G);
    if (ret != ESP_OK) return ret;

    // Set DLPF to 42Hz
    ret = mpu6050_set_dlpf(MPU6050_DLPF_44HZ);
    if (ret != ESP_OK) return ret;

    // Set sample rate divider (1kHz / (1 + 4) = 200Hz)
    ret = mpu6050_write_byte(MPU6050_REG_SMPLRT_DIV, 4);
    if (ret != ESP_OK) return ret;

    initialized = true;
    printf("MPU6xxx sensor initialized successfully!\n");
    ESP_LOGI(TAG, "MPU6xxx sensor initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_test_connection(void) {
    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_byte(MPU6050_REG_WHO_AM_I, &who_am_i);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    printf("WHO_AM_I register: 0x%02X\n", who_am_i);
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);

    // Check for valid MPU chip IDs
    // MPU6050: 0x68
    // MPU6500: 0x70
    // MPU9250: 0x71 or 0x73
    if (who_am_i == 0x68) {
        printf("Detected: MPU6050\n");
        ESP_LOGI(TAG, "Detected: MPU6050");
    } else if (who_am_i == 0x70) {
        printf("Detected: MPU6500\n");
        ESP_LOGI(TAG, "Detected: MPU6500");
    } else if (who_am_i == 0x71 || who_am_i == 0x73) {
        printf("Detected: MPU9250\n");
        ESP_LOGI(TAG, "Detected: MPU9250");
    } else {
        printf("Unknown device with WHO_AM_I: 0x%02X\n", who_am_i);
        ESP_LOGE(TAG, "Unknown device with WHO_AM_I: 0x%02X", who_am_i);
        ESP_LOGE(TAG, "Expected: 0x68 (MPU6050), 0x70 (MPU6500), 0x71/0x73 (MPU9250)");
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_range(mpu6050_gyro_range_t range) {
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_GYRO_CONFIG, range << 3);
    if (ret != ESP_OK) return ret;

    current_gyro_range = range;

    // Update scale factor
    switch (range) {
        case MPU6050_GYRO_RANGE_250DPS:
            gyro_scale = 131.0;
            break;
        case MPU6050_GYRO_RANGE_500DPS:
            gyro_scale = 65.5;
            break;
        case MPU6050_GYRO_RANGE_1000DPS:
            gyro_scale = 32.8;
            break;
        case MPU6050_GYRO_RANGE_2000DPS:
            gyro_scale = 16.4;
            break;
    }

    ESP_LOGI(TAG, "Gyro range set to ±%d°/s", 250 * (1 << range));
    return ESP_OK;
}

esp_err_t mpu6050_set_accel_range(mpu6050_accel_range_t range) {
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, range << 3);
    if (ret != ESP_OK) return ret;

    current_accel_range = range;

    // Update scale factor
    switch (range) {
        case MPU6050_ACCEL_RANGE_2G:
            accel_scale = 16384.0;
            break;
        case MPU6050_ACCEL_RANGE_4G:
            accel_scale = 8192.0;
            break;
        case MPU6050_ACCEL_RANGE_8G:
            accel_scale = 4096.0;
            break;
        case MPU6050_ACCEL_RANGE_16G:
            accel_scale = 2048.0;
            break;
    }

    ESP_LOGI(TAG, "Accel range set to ±%dg", 2 * (1 << range));
    return ESP_OK;
}

esp_err_t mpu6050_set_dlpf(mpu6050_dlpf_t dlpf) {
    return mpu6050_write_byte(MPU6050_REG_CONFIG, dlpf);
}

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data) {
    if (!initialized) {
        ESP_LOGE(TAG, "MPU6050 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Combine high and low bytes
    data->accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}

esp_err_t mpu6050_read_data(mpu6050_data_t *data) {
    mpu6050_raw_data_t raw_data;
    esp_err_t ret = mpu6050_read_raw(&raw_data);

    if (ret != ESP_OK) {
        return ret;
    }

    // Convert to physical units and apply calibration
    data->accel_x = (raw_data.accel_x_raw / accel_scale) * 9.81 - calibration.accel_x_offset;
    data->accel_y = (raw_data.accel_y_raw / accel_scale) * 9.81 - calibration.accel_y_offset;
    data->accel_z = (raw_data.accel_z_raw / accel_scale) * 9.81 - calibration.accel_z_offset;

    data->gyro_x = (raw_data.gyro_x_raw / gyro_scale) - calibration.gyro_x_offset;
    data->gyro_y = (raw_data.gyro_y_raw / gyro_scale) - calibration.gyro_y_offset;
    data->gyro_z = (raw_data.gyro_z_raw / gyro_scale) - calibration.gyro_z_offset;

    // Temperature in Celsius
    data->temp = (raw_data.temp_raw / 340.0) + 36.53;

    return ESP_OK;
}

esp_err_t mpu6050_calibrate(uint16_t samples) {
    ESP_LOGI(TAG, "Starting calibration with %d samples...", samples);
    ESP_LOGI(TAG, "Keep the sensor still and level!");

    vTaskDelay(pdMS_TO_TICKS(2000));  // Give user time to position sensor

    float accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

    for (uint16_t i = 0; i < samples; i++) {
        mpu6050_raw_data_t raw_data;
        esp_err_t ret = mpu6050_read_raw(&raw_data);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration failed at sample %d", i);
            return ret;
        }

        accel_x_sum += raw_data.accel_x_raw / accel_scale * 9.81;
        accel_y_sum += raw_data.accel_y_raw / accel_scale * 9.81;
        accel_z_sum += raw_data.accel_z_raw / accel_scale * 9.81;

        gyro_x_sum += raw_data.gyro_x_raw / gyro_scale;
        gyro_y_sum += raw_data.gyro_y_raw / gyro_scale;
        gyro_z_sum += raw_data.gyro_z_raw / gyro_scale;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    calibration.accel_x_offset = accel_x_sum / samples;
    calibration.accel_y_offset = accel_y_sum / samples;
    calibration.accel_z_offset = (accel_z_sum / samples) - 9.81;  // Subtract gravity

    calibration.gyro_x_offset = gyro_x_sum / samples;
    calibration.gyro_y_offset = gyro_y_sum / samples;
    calibration.gyro_z_offset = gyro_z_sum / samples;

    ESP_LOGI(TAG, "Calibration complete!");
    ESP_LOGI(TAG, "Accel offsets: X=%.3f Y=%.3f Z=%.3f m/s²", calibration.accel_x_offset,
             calibration.accel_y_offset, calibration.accel_z_offset);
    ESP_LOGI(TAG, "Gyro offsets: X=%.3f Y=%.3f Z=%.3f °/s", calibration.gyro_x_offset,
             calibration.gyro_y_offset, calibration.gyro_z_offset);

    return ESP_OK;
}

esp_err_t mpu6050_get_angles(float *pitch, float *roll) {
    mpu6050_data_t data;
    esp_err_t ret = mpu6050_read_data(&data);

    if (ret != ESP_OK) {
        return ret;
    }

    // Calculate pitch and roll from accelerometer
    *pitch = atan2(data.accel_y, sqrt(data.accel_x * data.accel_x + data.accel_z * data.accel_z)) *
             180.0 / M_PI;
    *roll = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) *
            180.0 / M_PI;

    return ESP_OK;
}
