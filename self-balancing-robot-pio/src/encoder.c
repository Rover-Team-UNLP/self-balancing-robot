#include "encoder.h"

#include <math.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ENCODER";

static encoder_data_t encoder_a_data = {0};
static encoder_data_t encoder_b_data = {0};

static volatile uint8_t encoder_a_state = 0;
static volatile uint8_t encoder_b_state = 0;

static const int8_t quadrature_table[4][4] = {
    {0, -1, 1, 0}, {1, 0, 0, -1}, {-1, 0, 0, 1}, {0, 1, -1, 0}};

static void IRAM_ATTR encoder_a_isr_handler(void* arg) {
    uint8_t a = gpio_get_level(ENCODER_A_PIN_A);
    uint8_t b = gpio_get_level(ENCODER_A_PIN_B);
    uint8_t new_state = (a << 1) | b;

    int8_t increment = quadrature_table[encoder_a_state][new_state];
    encoder_a_data.count += increment;

    if (increment > 0) {
        encoder_a_data.direction_forward = true;
    } else if (increment < 0) {
        encoder_a_data.direction_forward = false;
    }

    encoder_a_state = new_state;
}

static void IRAM_ATTR encoder_b_isr_handler(void* arg) {
    uint8_t a = gpio_get_level(ENCODER_B_PIN_A);
    uint8_t b = gpio_get_level(ENCODER_B_PIN_B);
    uint8_t new_state = (a << 1) | b;

    int8_t increment = quadrature_table[encoder_b_state][new_state];
    encoder_b_data.count += increment;

    if (increment > 0) {
        encoder_b_data.direction_forward = true;
    } else if (increment < 0) {
        encoder_b_data.direction_forward = false;
    }

    encoder_b_state = new_state;
}

esp_err_t encoder_init(void) {
    ESP_LOGI(TAG, "Initializing encoders...");
    printf("Initializing encoders with quadrature decoding...\n");

    gpio_config_t io_conf_a = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_A_PIN_A) | (1ULL << ENCODER_A_PIN_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE};

    esp_err_t ret = gpio_config(&io_conf_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Motor A encoder pins");
        return ret;
    }

    gpio_config_t io_conf_b = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_B_PIN_A) | (1ULL << ENCODER_B_PIN_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};

    ret = gpio_config(&io_conf_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Motor B encoder pins");
        return ret;
    }

    printf("Encoder pins configured:\n");
    printf("  Motor A: A=GPIO%d, B=GPIO%d\n", ENCODER_A_PIN_A, ENCODER_A_PIN_B);
    printf("  Motor B: A=GPIO%d, B=GPIO%d\n", ENCODER_B_PIN_A, ENCODER_B_PIN_B);

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }

    ret = gpio_isr_handler_add(ENCODER_A_PIN_A, encoder_a_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for Motor A channel A");
        return ret;
    }

    ret = gpio_isr_handler_add(ENCODER_A_PIN_B, encoder_a_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for Motor A channel B");
        return ret;
    }

    ret = gpio_isr_handler_add(ENCODER_B_PIN_A, encoder_b_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for Motor B channel A");
        return ret;
    }

    ret = gpio_isr_handler_add(ENCODER_B_PIN_B, encoder_b_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for Motor B channel B");
        return ret;
    }

    encoder_a_state = (gpio_get_level(ENCODER_A_PIN_A) << 1) | gpio_get_level(ENCODER_A_PIN_B);
    encoder_b_state = (gpio_get_level(ENCODER_B_PIN_A) << 1) | gpio_get_level(ENCODER_B_PIN_B);

    encoder_a_data.last_time_us = esp_timer_get_time();
    encoder_b_data.last_time_us = esp_timer_get_time();

    printf("Encoders initialized successfully!\n");
    printf("Specifications:\n");
    printf("  PPR: %d pulses/rev (motor shaft)\n", ENCODER_PPR);
    printf("  Gear ratio: %d:1\n", ENCODER_GEAR_RATIO);
    printf("  Counts per wheel rev: %d\n", ENCODER_COUNTS_PER_REV);

    ESP_LOGI(TAG, "Encoders initialized successfully");
    return ESP_OK;
}

esp_err_t encoder_reset(encoder_id_t encoder) {
    if (encoder == ENCODER_MOTOR_A) {
        encoder_a_data.count = 0;
        encoder_a_data.last_count = 0;
        encoder_a_data.position_revs = 0;
        encoder_a_data.velocity_rpm = 0;
        encoder_a_data.velocity_rad_s = 0;
        encoder_a_data.last_time_us = esp_timer_get_time();
        ESP_LOGI(TAG, "Motor A encoder reset");
    } else {
        encoder_b_data.count = 0;
        encoder_b_data.last_count = 0;
        encoder_b_data.position_revs = 0;
        encoder_b_data.velocity_rpm = 0;
        encoder_b_data.velocity_rad_s = 0;
        encoder_b_data.last_time_us = esp_timer_get_time();
        ESP_LOGI(TAG, "Motor B encoder reset");
    }
    return ESP_OK;
}

esp_err_t encoder_reset_all(void) {
    encoder_reset(ENCODER_MOTOR_A);
    encoder_reset(ENCODER_MOTOR_B);
    return ESP_OK;
}

int32_t encoder_get_count(encoder_id_t encoder) {
    return (encoder == ENCODER_MOTOR_A) ? encoder_a_data.count : encoder_b_data.count;
}

float encoder_get_position_revs(encoder_id_t encoder) {
    encoder_data_t* data = (encoder == ENCODER_MOTOR_A) ? &encoder_a_data : &encoder_b_data;
    data->position_revs = (float)data->count / ENCODER_COUNTS_PER_REV;
    return data->position_revs;
}

float encoder_get_velocity_rpm(encoder_id_t encoder) {
    return (encoder == ENCODER_MOTOR_A) ? encoder_a_data.velocity_rpm : encoder_b_data.velocity_rpm;
}

float encoder_get_velocity_rad_s(encoder_id_t encoder) {
    return (encoder == ENCODER_MOTOR_A) ? encoder_a_data.velocity_rad_s
                                        : encoder_b_data.velocity_rad_s;
}

bool encoder_get_direction(encoder_id_t encoder) {
    return (encoder == ENCODER_MOTOR_A) ? encoder_a_data.direction_forward
                                        : encoder_b_data.direction_forward;
}

esp_err_t encoder_update_velocity(encoder_id_t encoder) {
    encoder_data_t* data = (encoder == ENCODER_MOTOR_A) ? &encoder_a_data : &encoder_b_data;

    int64_t current_time_us = esp_timer_get_time();
    int64_t delta_time_us = current_time_us - data->last_time_us;

    if (delta_time_us < ENCODER_VELOCITY_UPDATE_US) {
        return ESP_OK;
    }

    int32_t delta_count = data->count - data->last_count;
    float delta_revs = (float)delta_count / ENCODER_COUNTS_PER_REV;
    float delta_time_min = (float)delta_time_us / 60000000.0f;

    if (delta_time_min > 0) {
        data->velocity_rpm = delta_revs / delta_time_min;
        data->velocity_rad_s = data->velocity_rpm * (2.0f * M_PI / 60.0f);
    } else {
        data->velocity_rpm = 0;
        data->velocity_rad_s = 0;
    }

    data->last_count = data->count;
    data->last_time_us = current_time_us;

    return ESP_OK;
}

esp_err_t encoder_update_all_velocities(void) {
    encoder_update_velocity(ENCODER_MOTOR_A);
    encoder_update_velocity(ENCODER_MOTOR_B);
    return ESP_OK;
}

void encoder_get_data(encoder_id_t encoder, encoder_data_t* data) {
    if (encoder == ENCODER_MOTOR_A) {
        *data = encoder_a_data;
    } else {
        *data = encoder_b_data;
    }

    data->position_revs = (float)data->count / ENCODER_COUNTS_PER_REV;
}
