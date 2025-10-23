#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Estado del sistema para el robot equilibrista
typedef struct {
    float angle;       // Ángulo de inclinación (rad)
    float angle_rate;  // Velocidad angular (rad/s)
    float position;    // Posición lineal (m)
    float velocity;    // Velocidad lineal (m/s)
} kalman_state_t;

// Estructura del filtro de Kalman
typedef struct {
    kalman_state_t state;  // Estado estimado
    float P[4][4];         // Matriz de covarianza del error
    float Q[4][4];         // Ruido del proceso
    float R[2][2];         // Ruido de medición
    float dt;              // Período de muestreo (s)
} kalman_filter_t;

// Inicializa el filtro de Kalman con parámetros de ruido
esp_err_t kalman_init(kalman_filter_t* kf, float dt);

// Actualiza el filtro con nuevas mediciones (ángulo del MPU, posición de encoders)
esp_err_t kalman_update(kalman_filter_t* kf, float measured_angle, float measured_gyro,
                        float measured_position, float measured_velocity);

// Obtiene el estado estimado actual
kalman_state_t kalman_get_state(kalman_filter_t* kf);

// Reinicia el filtro de Kalman
esp_err_t kalman_reset(kalman_filter_t* kf);

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_FILTER_H