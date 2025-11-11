#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <math.h>

#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_driver.h"
#include "mpu6050.h"
#include "pid_controller.h"

#define CONTROL_FREQ_HZ 100
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQ_HZ)
#define MOTOR_B_CORRECTION 0.82f  // Factor de correccion para el motor b
#define KP 18.0f
#define KI 0.0f
#define KD 0.0f

#define ALPHA (0.98f)  // Le creemos un 98% al giroscopio y un 2% al acelerometro

#define ANGLE_DEADBAND_ENTER 0.5f
#define ANGLE_DEADBAND_EXIT 0.7f

// Datos de sensores
typedef struct {
    float pitch;         // Ángulo de pitch del MPU (rad)
    float gyro_x;        // Velocidad angular Y (rad/s)
    float position_a;    // Posición encoder A (rev)
    float velocity_a;    // Velocidad encoder A (rad/s)
    float position_b;    // Posición encoder B (rev)
    float velocity_b;    // Velocidad encoder B (rad/s)
    uint32_t timestamp;  // Timestamp en ms
} sensor_data_t;

// Salida de control
typedef struct {
    int16_t motor_a_pwm;  // PWM para motor A (-255 a 255)
    int16_t motor_b_pwm;  // PWM para motor B (-255 a 255)
} control_output_t;

static pid_controller_t pid_angle;
static pid_controller_t pid_position;

// Inicializa el sistema de control (crea tareas FreeRTOS)
esp_err_t balance_control_init(void);

// Inicia el sistema de control
esp_err_t balance_control_start(void);

// Detiene el sistema de control
esp_err_t balance_control_stop(void);

// Calibra el offset del ángulo (llamar con robot vertical)
float balance_control_calibrate_offset(void);

// Configura las ganancias PID del ángulo
esp_err_t balance_control_set_angle_pid(float kp, float ki, float kd);

// Configura las ganancias PID de la posición
esp_err_t balance_control_set_position_pid(float kp, float ki, float kd);

#endif  // BALANCE_CONTROL_H