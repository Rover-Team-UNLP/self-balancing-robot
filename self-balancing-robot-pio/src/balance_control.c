#include "balance_control.h"

static const char* TAG = "BALANCE_V2";

// Handles de tareas
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t control_task_handle = NULL;

// Mutex para proteger variables compartidas
static SemaphoreHandle_t xMutex = NULL;

// Variables compartidas
static volatile float angle_current = 0.0f;
static volatile float position_avg = 0.0f;
static volatile float velocity_avg = 0.0f;

// Estado del sistema
static bool system_running = false;

// Controlador PID
static pid_controller_t pid_angle;
static pid_controller_t pid_position;

// Conversiones
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define RPM_TO_RAD_S (2.0f * M_PI / 60.0f)

// Parámetros del filtro complementario
#define COMPLEMENTARY_ALPHA 0.98f  // Peso del giroscopio (0.96-0.98 típico)
static float prev_angle_complementary = 0.0f;
static float angle_offset = 0.0f;

// Constante de corrección para ángulos negativos (ajustar según hardware)
#define NEG_CORRECT 1.07f

// Límites de control
#define ANGLE_DEADBAND 0.5f       // ±0.5° zona muerta
#define ANGLE_SAFETY_LIMIT 45.0f  // ±45° límite de seguridad
#define MAX_PWM_OUTPUT 200        // Límite máximo de PWM

/**
 * Filtro complementario para fusionar acelerómetro y giroscopio
 */
float complementary_filter(float accel_angle, float gyro_rate, float prev_angle, float dt) {
    // Integrar giroscopio
    float gyro_angle = prev_angle + gyro_rate * dt;

    // Fusionar con acelerómetro usando filtro complementario
    float filtered_angle =
        COMPLEMENTARY_ALPHA * gyro_angle + (1.0f - COMPLEMENTARY_ALPHA) * accel_angle;

    return filtered_angle;
}

/**
 * Tarea para leer sensores (MPU6050 + Encoders)
 */
static void sensor_task(void* pvParameters) {
    ESP_LOGI(TAG, "Sensor task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (system_running) {
        uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (current_time_ms - last_time_ms) / 1000.0f;  // En segundos
        last_time_ms = current_time_ms;

        // Leer MPU6050
        mpu6050_data_t mpu_data;
        float pitch_accel, roll;

        if (mpu6050_read_data(&mpu_data) == ESP_OK &&
            mpu6050_get_angles(&pitch_accel, &roll) == ESP_OK) {
            // Aplicar offset de calibración
            float accel_angle = pitch_accel - angle_offset;

            // Corregir ángulos negativos (por imperfecciones del montaje)
            if (accel_angle < 0) {
                accel_angle *= NEG_CORRECT;
            }

            // Aplicar filtro complementario
            float angle_filt = complementary_filter(accel_angle,
                                                    mpu_data.gyro_y,  // Velocidad angular en °/s
                                                    prev_angle_complementary, dt);

            prev_angle_complementary = angle_filt;

            // Actualizar variable compartida
            if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                angle_current = angle_filt;
                xSemaphoreGive(xMutex);
            }
        }

        // Leer encoders para control de posición
        encoder_update_all_velocities();

        encoder_data_t enc_a, enc_b;
        encoder_get_data(ENCODER_MOTOR_A, &enc_a);
        encoder_get_data(ENCODER_MOTOR_B, &enc_b);

        float pos_avg = (enc_a.position_revs + enc_b.position_revs) / 2.0f;
        float vel_avg = (enc_a.velocity_rpm + enc_b.velocity_rpm) / 2.0f;

        // Actualizar variables compartidas
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            position_avg = pos_avg;
            velocity_avg = vel_avg * RPM_TO_RAD_S;  // Convertir a rad/s
            xSemaphoreGive(xMutex);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

/**
 * Tarea de control PID
 */
static void control_task(void* pvParameters) {
    ESP_LOGI(TAG, "Control task started");

    TickType_t last_wake_time = xTaskGetTickCount();

    while (system_running) {
        // Leer variables compartidas
        float angle, position, velocity;

        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            angle = angle_current;
            position = position_avg;
            velocity = velocity_avg;
            xSemaphoreGive(xMutex);
        } else {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        int16_t motor_a_pwm = 0;
        int16_t motor_b_pwm = 0;

        // ==========================================
        // LÓGICA DE CONTROL
        // ==========================================

        float angle_abs = fabsf(angle);

        // CASO 1: Zona muerta - robot casi vertical
        if (angle_abs < ANGLE_DEADBAND) {
            motor_stop_all();
            pid_reset(&pid_angle);
            pid_reset(&pid_position);
        }
        // CASO 2: Ángulo excede límite de seguridad - robot caído
        else if (angle_abs > ANGLE_SAFETY_LIMIT) {
            motor_stop_all();
            pid_reset(&pid_angle);
            pid_reset(&pid_position);
            ESP_LOGW(TAG, "Safety stop: angle %.1f° exceeds limit", angle);
        }
        // CASO 3: Control activo
        else {
            // PID de ángulo (control principal)
            float angle_output = pid_compute(&pid_angle, angle);

            // PID de posición (control secundario para evitar deriva)
            float position_output = pid_compute(&pid_position, position);

            // Combinar salidas (ángulo es prioritario)
            float total_output = angle_output + position_output * 0.1f;

            // Limitar salida
            if (total_output > MAX_PWM_OUTPUT) total_output = MAX_PWM_OUTPUT;
            if (total_output < -MAX_PWM_OUTPUT) total_output = -MAX_PWM_OUTPUT;

            // Normalizar a rango 0-1 para la función drive
            float normalized_output = total_output / (float)MAX_PWM_OUTPUT;

            // Convertir a PWM de motores (0-255)
            motor_a_pwm = (int16_t)(normalized_output * 255.0f);
            motor_b_pwm = motor_a_pwm;  // Ambos motores igual velocidad

            // Aplicar a motores
            if (motor_a_pwm > 0) {
                motor_set_speed(MOTOR_A, MOTOR_FORWARD, (uint8_t)motor_a_pwm);
                motor_set_speed(MOTOR_B, MOTOR_FORWARD, (uint8_t)motor_b_pwm);
            } else if (motor_a_pwm < 0) {
                motor_set_speed(MOTOR_A, MOTOR_BACKWARD, (uint8_t)(-motor_a_pwm));
                motor_set_speed(MOTOR_B, MOTOR_BACKWARD, (uint8_t)(-motor_b_pwm));
            } else {
                motor_stop_all();
            }
        }

        // Log cada segundo
        static uint32_t log_counter = 0;
        if (++log_counter >= (CONTROL_FREQ_HZ / 10)) {
            log_counter = 0;

            // Mostrar componentes del PID por separado
            float p_term = pid_angle.kp * angle;
            float i_term = pid_angle.ki * pid_angle.integral;
            float d_term = pid_angle.kd * (angle - pid_angle.prev_error) / pid_angle.dt;

            ESP_LOGI(TAG, "Ang=%.2f° P=%.1f I=%.1f D=%.1f Total=%.1f PWM=%d", angle, p_term, i_term,
                     d_term, (p_term + i_term + d_term), motor_a_pwm);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

/**
 * Calibrar offset del MPU6050 en posición vertical
 */
esp_err_t balance_control_calibrate_offset(void) {
    ESP_LOGI(TAG, "Calibrating angle offset...");
    ESP_LOGI(TAG, "Keep robot in UPRIGHT position!");

    vTaskDelay(pdMS_TO_TICKS(1000));

    float pitch, roll;
    if (mpu6050_get_angles(&pitch, &roll) == ESP_OK) {
        angle_offset = pitch;
        ESP_LOGI(TAG, "Angle offset calibrated: %.2f°", angle_offset);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to calibrate offset");
    return ESP_FAIL;
}

esp_err_t balance_control_init(void) {
    ESP_LOGI(TAG, "Initializing balance control system (v2)...");

    float dt = CONTROL_PERIOD_MS / 1000.0f;

    // Inicializar PID de ángulo (valores del código que funcionaba)
    if (pid_init(&pid_angle, 20.0f, 0.0f, 0.0f, dt) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize angle PID");
        return ESP_FAIL;
    }
    pid_set_limits(&pid_angle, -MAX_PWM_OUTPUT, MAX_PWM_OUTPUT);
    pid_set_setpoint(&pid_angle, 0.0f);

    // Inicializar PID de posición (más suave)
    if (pid_init(&pid_position, 0.0f, 0.0f, 0.0f, dt) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize position PID");
        return ESP_FAIL;
    }
    pid_set_limits(&pid_position, -50.0f, 50.0f);
    pid_set_setpoint(&pid_position, 0.0f);

    // Crear mutex
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Balance control initialized");
    ESP_LOGI(TAG, "=== TUNING MODE: P-Only ===");
    ESP_LOGI(TAG, "PID Angle: Kp=5.0 Ki=0.0 Kd=0.0");

    return ESP_OK;
}

esp_err_t balance_control_start(void) {
    if (system_running) {
        ESP_LOGW(TAG, "System already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting balance control...");

    // Inicializar hardware
    if (motor_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor driver");
        return ESP_FAIL;
    }

    if (motor_driver_enable() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable motor driver");
        return ESP_FAIL;
    }

    system_running = true;

    // Crear tareas
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL,
                            2,  // Prioridad alta
                            &sensor_task_handle,
                            0  // Core 0
    );

    xTaskCreatePinnedToCore(control_task, "ControlTask", 4096, NULL,
                            1,  // Prioridad media
                            &control_task_handle,
                            1  // Core 1
    );

    ESP_LOGI(TAG, "Balance control started at %d Hz", CONTROL_FREQ_HZ);
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
