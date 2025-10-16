#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_A_PIN_A GPIO_NUM_32
#define ENCODER_A_PIN_B GPIO_NUM_35

#define ENCODER_B_PIN_A GPIO_NUM_34
#define ENCODER_B_PIN_B GPIO_NUM_39

#define ENCODER_PPR 11
#define ENCODER_GEAR_RATIO 90
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * ENCODER_GEAR_RATIO * 2)

#define ENCODER_VELOCITY_UPDATE_US 50000

typedef struct {
    volatile int32_t count;
    volatile int32_t last_count;
    volatile int64_t last_time_us;
    float position_revs;
    float velocity_rpm;
    float velocity_rad_s;
    volatile bool direction_forward;
} encoder_data_t;

typedef enum {
    ENCODER_MOTOR_A = 0,
    ENCODER_MOTOR_B = 1
} encoder_id_t;

esp_err_t encoder_init(void);
esp_err_t encoder_reset(encoder_id_t encoder);
esp_err_t encoder_reset_all(void);

int32_t encoder_get_count(encoder_id_t encoder);
float encoder_get_position_revs(encoder_id_t encoder);
float encoder_get_velocity_rpm(encoder_id_t encoder);
float encoder_get_velocity_rad_s(encoder_id_t encoder);
bool encoder_get_direction(encoder_id_t encoder);

esp_err_t encoder_update_velocity(encoder_id_t encoder);
esp_err_t encoder_update_all_velocities(void);

void encoder_get_data(encoder_id_t encoder, encoder_data_t *data);

#ifdef __cplusplus
}
#endif

#endif  // ENCODER_H
