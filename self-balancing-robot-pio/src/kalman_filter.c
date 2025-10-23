#include "kalman_filter.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

static const char* TAG = "KALMAN";

esp_err_t kalman_init(kalman_filter_t* kf, float dt) {
    if (kf == NULL || dt <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(kf, 0, sizeof(kalman_filter_t));
    kf->dt = dt;

    // Inicializar matriz de covarianza del error (P)
    for (int i = 0; i < 4; i++) {
        kf->P[i][i] = 1.0f;
    }

    // Matriz de ruido del proceso (Q) - ajustar según necesidad
    kf->Q[0][0] = 0.001f;  // Ángulo
    kf->Q[1][1] = 0.003f;  // Velocidad angular
    kf->Q[2][2] = 0.001f;  // Posición
    kf->Q[3][3] = 0.003f;  // Velocidad

    // Matriz de ruido de medición (R) - ajustar según sensores
    kf->R[0][0] = 0.03f;  // Ruido del acelerómetro (ángulo)
    kf->R[1][1] = 0.01f;  // Ruido del encoder (posición)

    ESP_LOGI(TAG, "Kalman filter initialized with dt=%.3f", dt);
    return ESP_OK;
}

esp_err_t kalman_update(kalman_filter_t* kf, float measured_angle, float measured_gyro,
                        float measured_position, float measured_velocity) {
    if (kf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // PREDICCIÓN
    // x_k = F * x_{k-1}
    float dt = kf->dt;
    kalman_state_t predicted;

    predicted.angle = kf->state.angle + kf->state.angle_rate * dt;
    predicted.angle_rate = kf->state.angle_rate + measured_gyro * dt;
    predicted.position = kf->state.position + kf->state.velocity * dt;
    predicted.velocity = kf->state.velocity;

    // P_k = F * P_{k-1} * F^T + Q (simplificado)
    float P_temp[4][4];
    memcpy(P_temp, kf->P, sizeof(P_temp));

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = P_temp[i][j] + kf->Q[i][j];
        }
    }

    // CORRECCIÓN
    // Innovación (diferencia entre medición y predicción)
    float y_angle = measured_angle - predicted.angle;
    float y_position = measured_position - predicted.position;

    // Ganancia de Kalman simplificada
    float K_angle = kf->P[0][0] / (kf->P[0][0] + kf->R[0][0]);
    float K_position = kf->P[2][2] / (kf->P[2][2] + kf->R[1][1]);

    // Actualizar estado
    kf->state.angle = predicted.angle + K_angle * y_angle;
    kf->state.angle_rate = predicted.angle_rate + K_angle * measured_gyro;
    kf->state.position = predicted.position + K_position * y_position;
    kf->state.velocity = measured_velocity;  // Usar medición directa

    // Actualizar covarianza
    kf->P[0][0] = (1.0f - K_angle) * kf->P[0][0];
    kf->P[2][2] = (1.0f - K_position) * kf->P[2][2];

    return ESP_OK;
}

kalman_state_t kalman_get_state(kalman_filter_t* kf) { return kf->state; }

esp_err_t kalman_reset(kalman_filter_t* kf) {
    if (kf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float dt = kf->dt;
    kalman_init(kf, dt);

    ESP_LOGI(TAG, "Kalman filter reset");
    return ESP_OK;
}