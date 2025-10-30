#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin definitions based on your wiring
#define MOTOR_STANDBY_PIN GPIO_NUM_27
#define MOTOR_PWMA_PIN GPIO_NUM_33
#define MOTOR_INA1_PIN GPIO_NUM_26
#define MOTOR_INA2_PIN GPIO_NUM_25
#define MOTOR_PWMB_PIN GPIO_NUM_13
#define MOTOR_INB1_PIN GPIO_NUM_14
#define MOTOR_INB2_PIN GPIO_NUM_12

// PWM configuration
#define MOTOR_PWM_FREQUENCY 1000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_PWM_MAX_DUTY 255
#define MIN_PWM 40

typedef enum { MOTOR_A = 0, MOTOR_B = 1 } motor_id_t;

typedef enum { MOTOR_STOP = 0, MOTOR_FORWARD = 1, MOTOR_BACKWARD = 2 } motor_direction_t;

// Inicializa el driver de motores configurando GPIOs y canales PWM
esp_err_t motor_driver_init(void);

// Habilita el driver de motores (pin STANDBY en alto)
esp_err_t motor_driver_enable(void);

// Deshabilita el driver de motores (pin STANDBY en bajo)
esp_err_t motor_driver_disable(void);

// Configura velocidad y dirección de un motor específico (velocidad: 0-255)
esp_err_t motor_set_speed(motor_id_t motor, motor_direction_t direction, uint8_t speed);

// Detiene un motor específico
esp_err_t motor_stop(motor_id_t motor);

// Detiene ambos motores
esp_err_t motor_stop_all(void);

// Parada de emergencia: detiene y deshabilita ambos motores
esp_err_t motor_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_DRIVER_H