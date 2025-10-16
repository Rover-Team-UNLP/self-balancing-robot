#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C Configuration
#define MPU6050_I2C_PORT I2C_NUM_0
#define MPU6050_I2C_SDA_PIN GPIO_NUM_21
#define MPU6050_I2C_SCL_PIN GPIO_NUM_22
#define MPU6050_I2C_FREQ_HZ 400000  // 400kHz

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68      // AD0 pin LOW
#define MPU6050_ADDR_ALT 0x69  // AD0 pin HIGH

// MPU6050 Registers
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_SMPLRT_DIV 0x19

// Data Registers
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40

#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_TEMP_OUT_L 0x42

#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_GYRO_XOUT_L 0x44
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_YOUT_L 0x46
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_GYRO_ZOUT_L 0x48

// Scale Ranges
typedef enum {
    MPU6050_GYRO_RANGE_250DPS = 0,   // ±250 deg/s
    MPU6050_GYRO_RANGE_500DPS = 1,   // ±500 deg/s
    MPU6050_GYRO_RANGE_1000DPS = 2,  // ±1000 deg/s
    MPU6050_GYRO_RANGE_2000DPS = 3   // ±2000 deg/s
} mpu6050_gyro_range_t;

typedef enum {
    MPU6050_ACCEL_RANGE_2G = 0,  // ±2g
    MPU6050_ACCEL_RANGE_4G = 1,  // ±4g
    MPU6050_ACCEL_RANGE_8G = 2,  // ±8g
    MPU6050_ACCEL_RANGE_16G = 3  // ±16g
} mpu6050_accel_range_t;

// Digital Low Pass Filter
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ = 2,
    MPU6050_DLPF_44HZ = 3,
    MPU6050_DLPF_21HZ = 4,
    MPU6050_DLPF_10HZ = 5,
    MPU6050_DLPF_5HZ = 6
} mpu6050_dlpf_t;

// Data structures
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;
} mpu6050_raw_data_t;

typedef struct {
    float accel_x;  // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;  // deg/s
    float gyro_y;
    float gyro_z;
    float temp;  // °C
} mpu6050_data_t;

typedef struct {
    float accel_x_offset;
    float accel_y_offset;
    float accel_z_offset;
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} mpu6050_calibration_t;

// Function declarations
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_test_connection(void);
esp_err_t mpu6050_set_gyro_range(mpu6050_gyro_range_t range);
esp_err_t mpu6050_set_accel_range(mpu6050_accel_range_t range);
esp_err_t mpu6050_set_dlpf(mpu6050_dlpf_t dlpf);
esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data);
esp_err_t mpu6050_read_data(mpu6050_data_t *data);
esp_err_t mpu6050_calibrate(uint16_t samples);
esp_err_t mpu6050_get_angles(float *pitch, float *roll);

#ifdef __cplusplus
}
#endif

#endif  // MPU6050_H
