#include "balance_control.h"

static const char* TAG = "BALANCE_CTRL";

// Handles de tareas
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t control_task_handle = NULL;
static TaskHandle_t motor_task_handle = NULL;

// Colas de comunicación entre tareas
static QueueHandle_t sensor_queue = NULL;
static QueueHandle_t control_queue = NULL;

// Filtros y controladores
static kalman_filter_t kalman;
static pid_controller_t pid_angle;
static pid_controller_t pid_position;

// Estado del sistema
static bool system_running = false;

// Conversiones
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define RPM_TO_RAD_S (2.0f * M_PI / 60.0f)

// Tarea de lectura de sensores
static void sensor_task(void* pvParameters) {
    ESP_LOGI(TAG, "Sensor task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    sensor_data_t sensor_data;

    while (system_running) {
        // Leer MPU6050
        mpu6050_data_t mpu_data;
        float pitch, roll;

        if (mpu6050_read_data(&mpu_data) == ESP_OK && mpu6050_get_angles(&pitch, &roll) == ESP_OK) {
            sensor_data.pitch = pitch * DEG_TO_RAD;  // Convertir a radianes
            sensor_data.gyro_y = mpu_data.gyro_y * DEG_TO_RAD;
        }

        // Leer encoders
        encoder_update_all_velocities();

        encoder_data_t enc_a, enc_b;
        encoder_get_data(ENCODER_MOTOR_A, &enc_a);
        encoder_get_data(ENCODER_MOTOR_B, &enc_b);

        sensor_data.position_a = enc_a.position_revs;
        sensor_data.velocity_a = enc_a.velocity_rpm * RPM_TO_RAD_S;
        sensor_data.position_b = enc_b.position_revs;
        sensor_data.velocity_b = enc_b.velocity_rpm * RPM_TO_RAD_S;

        sensor_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Enviar datos a la tarea de control
        xQueueSend(sensor_queue, &sensor_data, 0);

        // Esperar hasta el próximo ciclo
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

// Tarea de control (Kalman + PID)
static void control_task(void* pvParameters) {
    ESP_LOGI(TAG, "Control task started");

    sensor_data_t sensor_data;
    control_output_t control_output;

    while (system_running) {
        // Esperar datos de sensores
        if (xQueueReceive(sensor_queue, &sensor_data, pdMS_TO_TICKS(20)) == pdTRUE) {
            // Promediar posición y velocidad de ambos encoders
            float avg_position = (sensor_data.position_a + sensor_data.position_b) / 2.0f;
            float avg_velocity = (sensor_data.velocity_a + sensor_data.velocity_b) / 2.0f;

            // Actualizar filtro de Kalman
            kalman_update(&kalman, sensor_data.pitch, sensor_data.gyro_y, avg_position,
                          avg_velocity);

            // Obtener estado estimado
            kalman_state_t state = kalman_get_state(&kalman);

            // Control en cascada: primero controlar ángulo, luego posición

            // PID de ángulo (control primario)
            float angle_control = pid_compute(&pid_angle, state.angle);

            // PID de posición (control secundario)
            float position_control = pid_compute(&pid_position, state.position);

            // Combinar salidas (el ángulo es prioritario)
            float total_control = angle_control + position_control * 0.3f;

            // Convertir a PWM para motores
            control_output.motor_a_pwm = (int16_t)total_control;
            control_output.motor_b_pwm = (int16_t)total_control;

            // Limitar a rango válido
            if (control_output.motor_a_pwm > 255) control_output.motor_a_pwm = 255;
            if (control_output.motor_a_pwm < -255) control_output.motor_a_pwm = -255;
            if (control_output.motor_b_pwm > 255) control_output.motor_b_pwm = 255;
            if (control_output.motor_b_pwm < -255) control_output.motor_b_pwm = -255;

            // Enviar a tarea de motores
            xQueueSend(control_queue, &control_output, 0);

            // Log cada segundo
            static uint32_t log_counter = 0;
            if (++log_counter >= CONTROL_FREQ_HZ) {
                log_counter = 0;
                ESP_LOGI(TAG, "Angle=%.2f° Pos=%.2f PWM=%d", state.angle * RAD_TO_DEG,
                         state.position, control_output.motor_a_pwm);
            }
        }
    }

    vTaskDelete(NULL);
}

// Tarea de actuación de motores
static void motor_task(void* pvParameters) {
    ESP_LOGI(TAG, "Motor task started");

    control_output_t control_output;

    while (system_running) {
        // Esperar comandos de control
        if (xQueueReceive(control_queue, &control_output, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Aplicar a motor A
            if (control_output.motor_a_pwm > 0) {
                motor_set_speed(MOTOR_A, MOTOR_FORWARD, (uint8_t)control_output.motor_a_pwm);
            } else if (control_output.motor_a_pwm < 0) {
                motor_set_speed(MOTOR_A, MOTOR_BACKWARD, (uint8_t)(-control_output.motor_a_pwm));
            } else {
                motor_stop(MOTOR_A);
            }

            // Aplicar a motor B
            if (control_output.motor_b_pwm > 0) {
                motor_set_speed(MOTOR_B, MOTOR_FORWARD, (uint8_t)control_output.motor_b_pwm);
            } else if (control_output.motor_b_pwm < 0) {
                motor_set_speed(MOTOR_B, MOTOR_BACKWARD, (uint8_t)(-control_output.motor_b_pwm));
            } else {
                motor_stop(MOTOR_B);
            }
        } else {
            // Timeout - detener motores por seguridad
            motor_stop_all();
        }
    }

    // Detener motores al finalizar
    motor_stop_all();
    vTaskDelete(NULL);
}

esp_err_t balance_control_init(void) {
    ESP_LOGI(TAG, "Initializing balance control system...");

    // Inicializar filtro de Kalman
    float dt = CONTROL_PERIOD_MS / 1000.0f;
    if (kalman_init(&kalman, dt) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Kalman filter");
        return ESP_FAIL;
    }

    // Inicializar PIDs (valores iniciales, ajustar experimentalmente)
    if (pid_init(&pid_angle, 40.0f, 0.5f, 2.0f, dt) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize angle PID");
        return ESP_FAIL;
    }

    if (pid_init(&pid_position, 5.0f, 0.1f, 0.5f, dt) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize position PID");
        return ESP_FAIL;
    }

    // Configurar límites de PID
    pid_set_limits(&pid_angle, -255.0f, 255.0f);
    pid_set_limits(&pid_position, -50.0f, 50.0f);

    // Crear colas
    sensor_queue = xQueueCreate(5, sizeof(sensor_data_t));
    control_queue = xQueueCreate(5, sizeof(control_output_t));

    if (sensor_queue == NULL || control_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Balance control system initialized");
    return ESP_OK;
}

esp_err_t balance_control_start(void) {
    if (system_running) {
        ESP_LOGW(TAG, "System already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting balance control system...");

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

    // Crear tareas (prioridades: Sensores=5, Control=6, Motores=4)
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 5, &sensor_task_handle, 0);
    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 6, &control_task_handle, 1);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 4, &motor_task_handle, 1);

    ESP_LOGI(TAG, "Balance control system started");
    return ESP_OK;
}

esp_err_t balance_control_stop(void) {
    if (!system_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping balance control system...");

    system_running = false;

    // Esperar a que las tareas terminen
    vTaskDelay(pdMS_TO_TICKS(100));

    // Detener motores
    motor_stop_all();
    motor_driver_disable();

    ESP_LOGI(TAG, "Balance control system stopped");
    return ESP_OK;
}

esp_err_t balance_control_set_pid_gains(float kp_angle, float ki_angle, float kd_angle,
                                        float kp_position, float ki_position, float kd_position) {
    pid_angle.kp = kp_angle;
    pid_angle.ki = ki_angle;
    pid_angle.kd = kd_angle;

    pid_position.kp = kp_position;
    pid_position.ki = ki_position;
    pid_position.kd = kd_position;

    ESP_LOGI(TAG, "PID gains updated");
    return ESP_OK;
}