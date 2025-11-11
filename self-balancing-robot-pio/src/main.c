#include "main.h"
#include "web_interface.h"
#include "driver/gpio.h"
// Añade esto para abs()
#include <math.h>
// Añade esto para printf
#include <stdio.h>

static const char *TAG = "MAIN";

// ... (tu código de funciones LED va aquí) ...

// --- DEFINES ---
// (Moví los defines aquí fuera de app_main, es una mejor práctica)
#define COMPLEMENTARY_ALPHA 0.98f // 98% giroscopio, 2% acelerómetro
#define ANGLE_DEADBAND 0.5f       // ±0.5° zona muerta para no vibrar
#define ANGLE_SAFETY_LIMIT 45.0f  // ±45° límite para apagar motores
#define MAX_PWM_OUTPUT 200
#define MIN_PWM 20 // Suposición: define un arranque mínimo para motores

// Frecuencia del bucle de control (100Hz)
#define CONTROL_FREQ_HZ 100
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQ_HZ)

// Frecuencia del Logger (20Hz)
#define LOG_FREQ_HZ 20
#define LOG_PERIOD_MS (1000 / LOG_FREQ_HZ)

// Asumimos que estas structs y funciones existen en otros archivos
// --- INICIO DE SUPOSICIONES ---
#include "mpu6050.h"         // Asumo que mpu6050_data_t está aquí
#include "encoder.h"         // Asumo que encoder_data_t está aquí
#include "pid_controller.h"             // Asumo que pid_controller_t está aquí
#include "motor_driver.h"           // Asumo que motor_set_speed está aquí
#include "balance_control.h" // Asumo que balance_control_init/start están aquí

// Definiciones de PID (asumo que se inicializan en balance_control_init)
extern pid_controller_t pid_angle;
extern pid_controller_t pid_position;
// --- FIN DE SUPOSICIONES ---

void app_main(void)
{
    // ... (variables estáticas como angle_current, etc. van aquí) ...
    static volatile float angle_current = 0.0f; // El ángulo filtrado actual
    static volatile float position_avg = 0.0f;  // La posición promedio de las ruedas
    static volatile float velocity_avg = 0.0f;
    static float angle_offset = 0.0f; // Offset de calibración
    static float prev_angle_complementary = 0.0f;
    static bool in_deadband = false;

    printf("\n========================================\n");
    printf("Self-Balancing Robot - Control System\n");
    printf("========================================\n\n");

    ESP_LOGI(TAG, "Initializing system...");

    // ... (Tu código de inicialización de LED, Web, MPU, Encoders) ...
    // ... (Tu código de calibración MPU y Offset) ...
    // ... (Tu código de inicio de balance_control) ...

    ESP_LOGI(TAG, "Balance control system running!");

    // --- Variables del Bucle de Control ---
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    float dt = (float)CONTROL_PERIOD_MS / 1000.0f; // dt ahora es constante y correcto

    // --- Variables del Logger ---
    TickType_t last_log_time = xTaskGetTickCount();
    const TickType_t xLogFrequency = pdMS_TO_TICKS(LOG_PERIOD_MS);

    // Variables de lectura
    mpu6050_data_t new_mpu_data;
    float encoder_vel_avg, encoder_pos_avg;
    float pitch, roll;
    float accel_angle, gyro;
    float filtered_angle;
    encoder_data_t encoder_a, encoder_b;
    float angle_output, position_output, total_output;

    // ==========================================================
    // IMPRIMIR CABECERA CSV PARA EL SCRIPT DE PYTHON
    // ==========================================================
    // Usamos printf normal, que sale por el UART por defecto
    printf("timestamp_ms,angle,position,velocity,pid_angle_out,pid_pos_out,total_output\n");

    while (1)
    {
        // ==========================================================
        // 1. GESTIÓN DEL TIEMPO (LA PARTE MÁS IMPORTANTE)
        // ==========================================================
        // Esto bloquea la tarea hasta que sea el momento del
        // próximo ciclo de control (10ms / 100Hz).
        vTaskDelayUntil(&last_wake_time, xFrequency);

        // ==========================================================
        // 2. LECTURA DE SENSORES
        // ==========================================================

        // Lectura desde el IMU con chequeo de error
        if (mpu6050_read_data(&new_mpu_data) == ESP_OK &&
            mpu6050_get_angles(&pitch, &roll) == ESP_OK)
        {
            accel_angle = pitch - angle_offset;
            gyro = new_mpu_data.gyro_x;
        }
        else
        {
            ESP_LOGE("CONTROL_LOOP", "Error al leer IMU");
            continue; // Saltar este ciclo
        }

        // Filtro Complementario
        // (CORREGIDO: Usando COMPLEMENTARY_ALPHA)
        filtered_angle = (gyro * dt + prev_angle_complementary) * COMPLEMENTARY_ALPHA +
                         accel_angle * (1.0f - COMPLEMENTARY_ALPHA);
        prev_angle_complementary = filtered_angle;

        // Lectura desde los encoders
        encoder_update_all_velocities();
        encoder_get_data(ENCODER_MOTOR_A, &encoder_a);
        encoder_get_data(ENCODER_MOTOR_B, &encoder_b);

        encoder_vel_avg = (encoder_a.velocity_rad_s + encoder_b.velocity_rad_s) / 2.0f;
        encoder_pos_avg = (encoder_a.position_revs + encoder_b.position_revs) / 2.0f;

        // Actualizar variables "globales"
        angle_current = filtered_angle;
        position_avg = encoder_pos_avg;
        velocity_avg = encoder_vel_avg;

        // ==========================================================
        // 3. CÁLCULO DE CONTROL (PID)
        // ==========================================================

        float angle_abs = fabs(angle_current);
        // (CORREGIDO: Usando ANGLE_DEADBAND para ambas condiciones)
        if (in_deadband)
        {
            if (angle_abs > ANGLE_DEADBAND) // Salir de la zona muerta
                in_deadband = false;
        }
        else
        {
            if (angle_abs < ANGLE_DEADBAND) // Entrar en la zona muerta
                in_deadband = true;
        }

        angle_output = pid_compute(&pid_angle, angle_current);
        position_output = pid_compute(&pid_position, position_avg);

        if (in_deadband)
        {
            angle_output = 0; // No corregir ángulo si está "quieto"
        }

        total_output = -(angle_output + position_output * 0.1f); // Factor de posición (ajustable)

        // Saturación (Clamping) de la salida
        if (total_output > MAX_PWM_OUTPUT)
            total_output = MAX_PWM_OUTPUT;
        else if (total_output < -MAX_PWM_OUTPUT)
            total_output = -MAX_PWM_OUTPUT;

        // ==========================================================
        // 4. ACTUACIÓN (MOTORES)
        // ==========================================================

        int16_t pwm_signal = (int16_t)total_output;
        uint8_t pwm_magnitud = (uint8_t)fabs(total_output);

        // Aplicar PWM mínimo para vencer la inercia
        if (pwm_magnitud > 0 && pwm_magnitud < MIN_PWM)
        {
            pwm_magnitud = MIN_PWM;
        }

        // Corte de seguridad si el robot se cae
        if (angle_abs > ANGLE_SAFETY_LIMIT)
        {
            motor_stop_all();
        }
        else if (pwm_signal > 0)
        {
            motor_set_speed(MOTOR_A, MOTOR_FORWARD, pwm_magnitud);
            motor_set_speed(MOTOR_B, MOTOR_FORWARD, pwm_magnitud);
        }
        else if (pwm_signal < 0)
        {
            motor_set_speed(MOTOR_A, MOTOR_BACKWARD, pwm_magnitud);
            motor_set_speed(MOTOR_B, MOTOR_BACKWARD, pwm_magnitud);
        }
        else
        {
            motor_stop_all();
        }

        // ==========================================================
        // 5. LOGGER (Se ejecuta a 20Hz)
        // ==========================================================
        if (xTaskGetTickCount() - last_log_time >= xLogFrequency)
        {
            last_log_time = xTaskGetTickCount(); // Resetear timer de log

            // Imprimimos la fila de datos CSV
            // Usamos printf simple para salida por UART
            printf("%lu,", last_log_time * portTICK_PERIOD_MS); // Timestamp en ms
            printf("%.4f,", angle_current);
            printf("%.4f,", position_avg);
            printf("%.4f,", velocity_avg);
            printf("%.4f,", angle_output);
            printf("%.4f,", position_output);
            printf("%.4f\n", total_output); // \n para nueva línea
        }
    }
}