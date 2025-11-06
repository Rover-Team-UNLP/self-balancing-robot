#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

#include "esp_err.h"



typedef struct {
    float kp;  // Ganancia proporcional
    float ki;  // Ganancia integral
    float kd;  // Ganancia derivativa

    float setpoint;    // Valor deseado (0 para equilibrio)
    float integral;    // Acumulador integral
    float prev_error;  // Error anterior para derivada

    float output_min;  // Límite mínimo de salida
    float output_max;  // Límite máximo de salida

    float dt;  // Período de muestreo
} pid_controller_t;

// Inicializa el controlador PID
esp_err_t pid_init(pid_controller_t* pid, float kp, float ki, float kd, float dt);

// Configura los límites de salida del PID
esp_err_t pid_set_limits(pid_controller_t* pid, float min, float max);

// Configura el setpoint (punto de referencia)
esp_err_t pid_set_setpoint(pid_controller_t* pid, float setpoint);

// Calcula la salida del PID dado el valor medido actual
float pid_compute(pid_controller_t* pid, float measured_value);

// Reinicia el controlador PID (limpia integral y error previo)
esp_err_t pid_reset(pid_controller_t* pid);



#endif  // PID_CONTROLLER_H