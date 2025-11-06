#include "main.h"
#include "web_interface.h"
#include "driver/gpio.h"

static const char *TAG = "MAIN";

// LED integrado de la ESP32
#define LED_PIN GPIO_NUM_2

// Función para controlar el LED
static void led_init(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0); // Apagar inicialmente
}

static void led_on(void)
{
    gpio_set_level(LED_PIN, 1);
}

static void led_off(void)
{
    gpio_set_level(LED_PIN, 0);
}

static void led_blink(int times, int total_period_ms)
{
    int half_period = total_period_ms / (times * 2);
    for (int i = 0; i < times; i++)
    {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(half_period));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(half_period));
    }
}

void app_main(void)
{
    printf("\n========================================\n");
    printf("Self-Balancing Robot - Control System\n");
    printf("========================================\n\n");

    ESP_LOGI(TAG, "Initializing system...");

    // Inicializar LED
    led_init();

    // Inicializar interfaz web primero
    ESP_LOGI(TAG, "Starting web interface...");
    if (web_interface_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize web interface!");
        // Continuar de todos modos
    }
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "📱 Web Interface Ready!");
    ESP_LOGI(TAG, "Connect to WiFi: ESP32-Balance-Robot");
    ESP_LOGI(TAG, "Password: robot123");
    ESP_LOGI(TAG, "Open browser: http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    // Inicializar sensores
    ESP_LOGI(TAG, "Initializing MPU6050...");
    if (mpu6050_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6050!");
        return;
    }

    ESP_LOGI(TAG, "Initializing encoders...");
    if (encoder_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize encoders!");
        return;
    }

    // Calibrar giroscopio y acelerómetro del MPU
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== MPU CALIBRATION ==========");
    ESP_LOGI(TAG, "Place robot HORIZONTAL (lying down)");
    ESP_LOGI(TAG, "Keep it STILL for calibration...");
    ESP_LOGI(TAG, "LED will turn ON during calibration");
    ESP_LOGI(TAG, "====================================");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Encender LED durante calibración MPU
    led_on();
    ESP_LOGI(TAG, "Calibrating MPU... (LED ON)");

    if (mpu6050_calibrate(500) != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU calibration failed!");
        led_off();
        return;
    }
    ESP_LOGI(TAG, "MPU calibration complete");

    // Inicializar sistema de control
    ESP_LOGI(TAG, "Initializing balance control...");
    if (balance_control_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize balance control!");
        led_off();
        return;
    }

    // Parpadear LED 3 veces antes de calibración de offset
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "====== ANGLE OFFSET CALIBRATION ======");
    ESP_LOGI(TAG, "NOW place robot in UPRIGHT position");
    ESP_LOGI(TAG, "LED will BLINK 3 times (5 seconds)");
    ESP_LOGI(TAG, "Then calibration will start");
    ESP_LOGI(TAG, "=====================================");

    // Parpadear 3 veces en 5 segundos
    led_blink(3, 5000);

    ESP_LOGI(TAG, "Starting angle offset calibration... (LED ON)");
    led_on();

    if (balance_control_calibrate_offset() != ESP_OK)
    {
        ESP_LOGE(TAG, "Angle offset calibration failed!");
        led_off();
        return;
    }

    // Apagar LED al finalizar calibraciones
    led_off();
    ESP_LOGI(TAG, "All calibrations complete! (LED OFF)");

    // Esperar antes de iniciar control activo
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== READY TO START ==========");
    ESP_LOGI(TAG, "Starting balance control in 2 seconds...");
    ESP_LOGI(TAG, "Keep robot upright!");
    ESP_LOGI(TAG, "====================================");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Iniciar sistema de control
    if (balance_control_start() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start balance control!");
        return;
    }

    ESP_LOGI(TAG, "Balance control system running!");
    ESP_LOGI(TAG, "Robot should now try to maintain balance");

    // Mantener main task viva
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}