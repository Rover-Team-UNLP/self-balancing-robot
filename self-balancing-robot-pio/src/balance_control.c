/**
 * balance_control.c - El "Cerebro" del Robot
 * * Lee los sensores filtrados y calcula la salida
 * de los motores usando control PID para mantener el equilibrio.
 */

#include "balance_control.h"  // El .h con las funciones públicas

// --- Librerías de los módulos "rescatados" ---
#include "encoder.h"
#include "motor_driver.h"
#include "mpu6050.h"
#include "pid_controller.h"

// --- Constantes del Módulo ---
static const char* TAG = "BALANCE_CONTROL";

// Frecuencia del bucle de control (100Hz)
#define CONTROL_FREQ_HZ 100
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQ_HZ)

// Parámetros Físicos y de Seguridad
#define COMPLEMENTARY_ALPHA 0.98f  // 98% giroscopio, 2% acelerómetro
#define ANGLE_DEADBAND 0.5f        // ±0.5° zona muerta para no vibrar
#define ANGLE_SAFETY_LIMIT 45.0f   // ±45° límite para apagar motores
#define MAX_PWM_OUTPUT 200         // Límite de PWM (de 255) para no ir tan rápido

// --- Variables Estáticas del Módulo ---

// Controladores PID (usando tu módulo pid_controller.c)
static pid_controller_t pid_angle;
static pid_controller_t pid_position;

// Handles de las Tareas RTOS
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t control_task_handle = NULL;

// Mutex para comunicación entre tareas
// static SemaphoreHandle_t xMutex = NULL;
static SemaphoreHandle_t sensor_data_ready_sem = NULL;

static volatile float angle_current = 0.0f;  // El ángulo filtrado actual
static volatile float position_avg = 0.0f;   // La posición promedio de las ruedas
static volatile float velocity_avg = 0.0f;   // La velocidad promedio de las ruedas

// Variables de estado (internas)
static float angle_offset = 0.0f;              // Offset de calibración
static float prev_angle_complementary = 0.0f;  // Memoria del filtro
static bool system_running = false;            // Flag de esta
static bool in_deadband = false;

static void sensor_task(void* pvParameters);

static void control_task(void* pvParameters);

esp_err_t balance_control_init(void) {
    // xMutex = xSemaphoreCreateMutex();
    sensor_data_ready_sem = xSemaphoreCreateBinary();
    if (sensor_data_ready_sem == NULL) {
        ESP_LOGE(TAG, "Fallo en la creacion del semaforo");
        return ESP_FAIL;
    }

    float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    in_deadband = false;

    pid_init(&pid_angle, KP, KI, KD, dt);
    pid_init(&pid_position, 0, 0, 0, dt);

    pid_set_limits(&pid_angle, -MAX_PWM_OUTPUT, MAX_PWM_OUTPUT);
    pid_set_setpoint(&pid_angle, 0.0f);

    return ESP_OK;
}

esp_err_t balance_control_start(void) {
    if (motor_driver_init() != ESP_OK) {
        return ESP_FAIL;
    }
    if (motor_driver_enable() != ESP_OK) {
        return ESP_FAIL;
    }

    system_running = true;

    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL, 2, &sensor_task_handle, 0);
    xTaskCreatePinnedToCore(control_task, "ControlTask", 4096, NULL, 1, &control_task_handle, 1);

    return ESP_OK;
}

esp_err_t balance_control_stop(void) {
    if (!system_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping balance control...");

    system_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));

    motor_stop_all();
    motor_driver_disable();

    ESP_LOGI(TAG, "Balance control stopped");
    return ESP_OK;
}

esp_err_t balance_control_calibrate_offset(void) {
    float pitch;
    float roll;
    esp_err_t return_value = mpu6050_get_angles(&pitch, &roll);
    if (return_value != ESP_OK) {
        return return_value;
    }
    angle_offset = pitch;
    return return_value;
}

esp_err_t balance_control_set_angle_pid(float kp, float ki, float kd) {
    pid_angle.kp = kp;
    pid_angle.ki = ki;
    pid_angle.kd = kd;

    ESP_LOGI(TAG, "Angle PID updated: Kp=%.1f Ki=%.1f Kd=%.1f", kp, ki, kd);
    return ESP_OK;
}

esp_err_t balance_control_set_position_pid(float kp, float ki, float kd) {
    pid_position.kp = kp;
    pid_position.ki = ki;
    pid_position.kd = kd;

    ESP_LOGI(TAG, "Position PID updated: Kp=%.1f Ki=%.1f Kd=%.1f", kp, ki, kd);
    return ESP_OK;
}

// --- Implementación de Funciones Internas (Privadas) ---

static void sensor_task(void* pvParameters) {
    // Variables de temporizacion de la tarea
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t current_time_ms;
    float dt;

    // Variables de lectura
    mpu6050_data_t new_mpu_data;
    float encoder_vel_avg, encoder_pos_avg;
    float pitch, roll;
    float accel_angle, gyro;
    sensor_data_t new_sensor_data;
    float filtered_angle;
    encoder_data_t encoder_a, encoder_b;

    while (system_running) {
        // Actualizo el diferencial de tiempo
        current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        dt = (current_time_ms - last_time_ms) / 1000.0f;
        last_time_ms = current_time_ms;

        // Lectura desde el IMU con chequeo de error
        if (mpu6050_read_data(&new_mpu_data) == ESP_OK &&
            mpu6050_get_angles(&pitch, &roll) == ESP_OK) {
            accel_angle = pitch - angle_offset;
            gyro = new_mpu_data.gyro_x;
        } else {
            ESP_LOGE("SENSOR_TASK", "Error al leer IMU");
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        filtered_angle = (gyro * dt + prev_angle_complementary) * ALPHA + accel_angle * (1 - ALPHA);
        prev_angle_complementary = filtered_angle;

        // Lectura desde los encoders
        encoder_update_all_velocities();

        encoder_get_data(ENCODER_MOTOR_A, &encoder_a);
        encoder_get_data(ENCODER_MOTOR_B, &encoder_b);

        encoder_vel_avg = (encoder_a.velocity_rad_s + encoder_b.velocity_rad_s) / 2.0f;
        encoder_pos_avg = (encoder_a.position_revs + encoder_b.position_revs) / 2.0f;

        // Actualizo las variables compartidas cuando me cedan el mute
        // if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        angle_current = filtered_angle;
        position_avg = encoder_pos_avg;
        velocity_avg = encoder_vel_avg;

        //     xSemaphoreGive(xMutex);
        // }
        xSemaphoreGive(sensor_data_ready_sem);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

static void control_task(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    float angle = 0;
    float position = 0;
    while (system_running) {
        if (xSemaphoreTake(sensor_data_ready_sem, pdMS_TO_TICKS(5)) == true) {
            angle = angle_current;
            position = position_avg;
        } else {
            continue;
        }

        float angle_abs = fabs(angle);
        if (in_deadband) {
            if (angle_abs > ANGLE_DEADBAND_EXIT) in_deadband = false;
        } else {
            if (angle_abs < ANGLE_DEADBAND_ENTER) in_deadband = true;
        }

        float angle_output = pid_compute(&pid_angle, angle);
        float position_output = pid_compute(&pid_position, position);

        float total_output = -(angle_output + position_output * 0.1f);
        if (total_output > MAX_PWM_OUTPUT) {
            total_output = MAX_PWM_OUTPUT;
        } else if (total_output < -MAX_PWM_OUTPUT) {
            total_output = -MAX_PWM_OUTPUT;
        }

        int16_t pwm_signal = (int16_t)total_output;
        uint8_t pwm_magnitud = (uint8_t)abs(total_output);

        if (pwm_magnitud > 0 && pwm_magnitud < MIN_PWM) {
            pwm_magnitud = MIN_PWM;
        }

        if (pwm_signal > 0) {
            motor_set_speed(MOTOR_A, MOTOR_FORWARD, pwm_magnitud);
            motor_set_speed(MOTOR_B, MOTOR_FORWARD, pwm_magnitud * MOTOR_B_CORRECTION);
        } else if (pwm_signal < 0) {
            motor_set_speed(MOTOR_A, MOTOR_BACKWARD, pwm_magnitud);
            motor_set_speed(MOTOR_B, MOTOR_BACKWARD, pwm_magnitud * MOTOR_B_CORRECTION);
        } else {
            motor_stop_all();
        }
    }
}
