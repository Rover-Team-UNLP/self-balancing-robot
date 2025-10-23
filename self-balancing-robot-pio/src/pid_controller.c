#include "pid_controller.h"

#include <string.h>

#include "esp_log.h"

static const char* TAG = "PID";

esp_err_t pid_init(pid_controller_t* pid, float kp, float ki, float kd, float dt) {
    if (pid == NULL || dt <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(pid, 0, sizeof(pid_controller_t));

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;

    pid->output_min = -255.0f;  // PWM mínimo
    pid->output_max = 255.0f;   // PWM máximo

    pid->setpoint = 0.0f;  // Equilibrio en 0 grados

    ESP_LOGI(TAG, "PID initialized: Kp=%.3f, Ki=%.3f, Kd=%.3f, dt=%.3f", kp, ki, kd, dt);
    return ESP_OK;
}

esp_err_t pid_set_limits(pid_controller_t* pid, float min, float max) {
    if (pid == NULL || min >= max) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->output_min = min;
    pid->output_max = max;

    return ESP_OK;
}

esp_err_t pid_set_setpoint(pid_controller_t* pid, float setpoint) {
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->setpoint = setpoint;
    return ESP_OK;
}

float pid_compute(pid_controller_t* pid, float measured_value) {
    if (pid == NULL) {
        return 0.0f;
    }

    // Calcular error
    float error = pid->setpoint - measured_value;

    // Término proporcional
    float p_term = pid->kp * error;

    // Término integral (con anti-windup)
    pid->integral += error * pid->dt;

    // Anti-windup: limitar integral
    float max_integral = pid->output_max / (pid->ki + 0.001f);
    if (pid->integral > max_integral) {
        pid->integral = max_integral;
    } else if (pid->integral < -max_integral) {
        pid->integral = -max_integral;
    }

    float i_term = pid->ki * pid->integral;

    // Término derivativo
    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term = pid->kd * derivative;

    // Calcular salida total
    float output = p_term + i_term + d_term;

    // Aplicar límites de saturación
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    // Guardar error para próxima iteración
    pid->prev_error = error;

    return output;
}

esp_err_t pid_reset(pid_controller_t* pid) {
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    ESP_LOGI(TAG, "PID reset");
    return ESP_OK;
}