#include "motor_driver.h"

static const char* TAG = "MOTOR_DRIVER";
static bool initialized = false;

esp_err_t motor_driver_init(void) {
    ESP_LOGI(TAG, "Initializing motor driver...");

    // Configure GPIO pins
    gpio_config_t io_conf = {};

    // Configure control pins as outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_STANDBY_PIN) | (1ULL << MOTOR_INA1_PIN) |
                           (1ULL << MOTOR_INA2_PIN) | (1ULL << MOTOR_INB1_PIN) |
                           (1ULL << MOTOR_INB2_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins");
        return ret;
    }

    // Initialize all control pins to safe state
    gpio_set_level(MOTOR_STANDBY_PIN, 0);  // Disable motors initially
    gpio_set_level(MOTOR_INA1_PIN, 0);
    gpio_set_level(MOTOR_INA2_PIN, 0);
    gpio_set_level(MOTOR_INB1_PIN, 0);
    gpio_set_level(MOTOR_INB2_PIN, 0);

    // Configure PWM timer for Motor A
    ledc_timer_config_t ledc_timer_a = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .timer_num = LEDC_TIMER_0,
                                        .duty_resolution = MOTOR_PWM_RESOLUTION,
                                        .freq_hz = MOTOR_PWM_FREQUENCY,
                                        .clk_cfg = LEDC_AUTO_CLK};
    ret = ledc_timer_config(&ledc_timer_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer A");
        return ret;
    }

    // Configure PWM channel for Motor A
    ledc_channel_config_t ledc_channel_a = {.gpio_num = MOTOR_PWMA_PIN,
                                            .speed_mode = LEDC_LOW_SPEED_MODE,
                                            .channel = LEDC_CHANNEL_0,
                                            .timer_sel = LEDC_TIMER_0,
                                            .duty = 0,
                                            .hpoint = 0};
    ret = ledc_channel_config(&ledc_channel_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel A");
        return ret;
    }

    // Configure PWM timer for Motor B
    ledc_timer_config_t ledc_timer_b = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .timer_num = LEDC_TIMER_1,
                                        .duty_resolution = MOTOR_PWM_RESOLUTION,
                                        .freq_hz = MOTOR_PWM_FREQUENCY,
                                        .clk_cfg = LEDC_AUTO_CLK};
    ret = ledc_timer_config(&ledc_timer_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer B");
        return ret;
    }

    // Configure PWM channel for Motor B
    ledc_channel_config_t ledc_channel_b = {.gpio_num = MOTOR_PWMB_PIN,
                                            .speed_mode = LEDC_LOW_SPEED_MODE,
                                            .channel = LEDC_CHANNEL_1,
                                            .timer_sel = LEDC_TIMER_1,
                                            .duty = 0,
                                            .hpoint = 0};
    ret = ledc_channel_config(&ledc_channel_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel B");
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG, "Motor driver initialized successfully");
    return ESP_OK;
}

esp_err_t motor_driver_enable(void) {
    if (!initialized) {
        ESP_LOGE(TAG, "Motor driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(MOTOR_STANDBY_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled");
    return ESP_OK;
}

esp_err_t motor_driver_disable(void) {
    if (!initialized) {
        ESP_LOGE(TAG, "Motor driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(MOTOR_STANDBY_PIN, 0);
    ESP_LOGI(TAG, "Motor driver disabled");
    return ESP_OK;
}

esp_err_t motor_set_speed(motor_id_t motor, motor_direction_t direction, uint8_t speed) {
    if (!initialized) {
        ESP_LOGE(TAG, "Motor driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (speed > MOTOR_PWM_MAX_DUTY) {
        speed = MOTOR_PWM_MAX_DUTY;
    }

    esp_err_t ret = ESP_OK;

    if (motor == MOTOR_A) {
        // Set direction for Motor A
        switch (direction) {
            case MOTOR_FORWARD:
                gpio_set_level(MOTOR_INA1_PIN, 1);
                gpio_set_level(MOTOR_INA2_PIN, 0);
                break;
            case MOTOR_BACKWARD:
                gpio_set_level(MOTOR_INA1_PIN, 0);
                gpio_set_level(MOTOR_INA2_PIN, 1);
                break;
            case MOTOR_STOP:
            default:
                gpio_set_level(MOTOR_INA1_PIN, 0);
                gpio_set_level(MOTOR_INA2_PIN, 0);
                speed = 0;
                break;
        }

        // Set PWM duty cycle for Motor A
        ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
        if (ret == ESP_OK) {
            ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }

        ESP_LOGI(TAG, "Motor A: Direction=%d, Speed=%d", direction, speed);

    } else if (motor == MOTOR_B) {
        // Set direction for Motor B
        switch (direction) {
            case MOTOR_FORWARD:
                gpio_set_level(MOTOR_INB1_PIN, 1);
                gpio_set_level(MOTOR_INB2_PIN, 0);
                break;
            case MOTOR_BACKWARD:
                gpio_set_level(MOTOR_INB1_PIN, 0);
                gpio_set_level(MOTOR_INB2_PIN, 1);
                break;
            case MOTOR_STOP:
            default:
                gpio_set_level(MOTOR_INB1_PIN, 0);
                gpio_set_level(MOTOR_INB2_PIN, 0);
                speed = 0;
                break;
        }

        // Set PWM duty cycle for Motor B
        ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
        if (ret == ESP_OK) {
            ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        }

        ESP_LOGI(TAG, "Motor B: Direction=%d, Speed=%d", direction, speed);
    }

    return ret;
}

esp_err_t motor_stop(motor_id_t motor) { return motor_set_speed(motor, MOTOR_STOP, 0); }

esp_err_t motor_stop_all(void) {
    esp_err_t ret1 = motor_stop(MOTOR_A);
    esp_err_t ret2 = motor_stop(MOTOR_B);

    ESP_LOGI(TAG, "All motors stopped");
    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t motor_emergency_stop(void) {
    esp_err_t ret = motor_stop_all();
    motor_driver_disable();
    ESP_LOGW(TAG, "EMERGENCY STOP executed");
    return ret;
}