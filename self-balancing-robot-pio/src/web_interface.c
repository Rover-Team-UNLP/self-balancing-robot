#include "web_interface.h"

#include <stdlib.h>
#include <string.h>

#include "balance_control.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char* TAG = "WEB_INTERFACE";

// Configuración WiFi - CAMBIAR ESTOS VALORES
#define WIFI_SSID "ESP32-Balance-Robot"
#define WIFI_PASS "robot123"

static httpd_handle_t server = NULL;

// HTML minimalista
static const char* html_page =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Robot PID Tuning</title>"
    "<style>"
    "body{font-family:Arial;max-width:400px;margin:20px "
    "auto;padding:20px;background:#1a1a1a;color:#fff}"
    "h1{text-align:center;color:#4CAF50;font-size:24px;margin-bottom:30px}"
    ".section{background:#2a2a2a;padding:15px;margin:10px 0;border-radius:8px}"
    ".label{color:#aaa;font-size:12px;margin-bottom:5px}"
    "input{width:100%;padding:8px;margin:5px 0;box-sizing:border-box;background:#333;border:1px "
    "solid #555;color:#fff;border-radius:4px;font-size:16px}"
    "button{width:100%;padding:12px;margin:8px "
    "0;background:#4CAF50;border:none;color:#fff;border-radius:6px;font-size:16px;cursor:pointer;"
    "font-weight:bold}"
    "button:active{background:#45a049}"
    ".cal-btn{background:#2196F3}"
    ".cal-btn:active{background:#0b7dda}"
    ".stop-btn{background:#f44336}"
    ".stop-btn:active{background:#da190b}"
    ".status{text-align:center;padding:10px;background:#333;border-radius:6px;margin:10px 0}"
    ".value{color:#4CAF50;font-size:18px;font-weight:bold}"
    "</style>"
    "</head>"
    "<body>"
    "<h1>🤖 Robot PID Tuning</h1>"

    "<div class='section'>"
    "<h3 style='margin-top:0;color:#4CAF50'>Angle PID</h3>"
    "<div class='label'>Kp (Proportional)</div>"
    "<input type='number' id='kp_a' value='18.0' step='0.5'>"
    "<div class='label'>Ki (Integral)</div>"
    "<input type='number' id='ki_a' value='100.0' step='5.0'>"
    "<div class='label'>Kd (Derivative)</div>"
    "<input type='number' id='kd_a' value='0.5' step='0.1'>"
    "<button onclick='updateAnglePID()'>Apply Angle PID</button>"
    "</div>"

    "<div class='section'>"
    "<h3 style='margin-top:0;color:#2196F3'>Position PID</h3>"
    "<div class='label'>Kp (Proportional)</div>"
    "<input type='number' id='kp_p' value='5.0' step='0.5'>"
    "<div class='label'>Ki (Integral)</div>"
    "<input type='number' id='ki_p' value='0.5' step='0.1'>"
    "<div class='label'>Kd (Derivative)</div>"
    "<input type='number' id='kd_p' value='0.2' step='0.05'>"
    "<button onclick='updatePositionPID()'>Apply Position PID</button>"
    "</div>"

    "<div class='section'>"
    "<h3 style='margin-top:0;color:#FF9800'>Control</h3>"
    "<button class='stop-btn' onclick='stop()'>⛔ Emergency Stop</button>"
    "<button onclick='getStatus()'>🔄 Get Status</button>"
    "</div>"

    "<div class='status' id='status'>Ready</div>"

    "<script>"
    "function updateAnglePID(){"
    "let kp=document.getElementById('kp_a').value;"
    "let ki=document.getElementById('ki_a').value;"
    "let kd=document.getElementById('kd_a').value;"
    "document.getElementById('status').innerHTML='⏳ Updating Angle PID...';"
    "fetch('/api/angle?kp='+kp+'&ki='+ki+'&kd='+kd)"
    ".then(r=>{if(!r.ok)throw new Error('Failed');return r.text();})"
    ".then(t=>{document.getElementById('status').innerHTML='✅ '+t;})"
    ".catch(e=>{document.getElementById('status').innerHTML='❌ Error: '+e.message;});"
    "}"
    "function updatePositionPID(){"
    "let kp=document.getElementById('kp_p').value;"
    "let ki=document.getElementById('ki_p').value;"
    "let kd=document.getElementById('kd_p').value;"
    "document.getElementById('status').innerHTML='⏳ Updating Position PID...';"
    "fetch('/api/position?kp='+kp+'&ki='+ki+'&kd='+kd)"
    ".then(r=>{if(!r.ok)throw new Error('Failed');return r.text();})"
    ".then(t=>{document.getElementById('status').innerHTML='✅ '+t;})"
    ".catch(e=>{document.getElementById('status').innerHTML='❌ Error: '+e.message;});"
    "}"
    "function stop(){"
    "document.getElementById('status').innerHTML='⏳ Stopping...';"
    "fetch('/api/stop')"
    ".then(r=>{if(!r.ok)throw new Error('Failed');return r.text();})"
    ".then(t=>{document.getElementById('status').innerHTML='⛔ '+t;})"
    ".catch(e=>{document.getElementById('status').innerHTML='❌ Error: '+e.message;});"
    "}"
    "function getStatus(){"
    "fetch('/api/status')"
    ".then(r=>{if(!r.ok)throw new Error('Failed');return r.text();})"
    ".then(t=>{document.getElementById('status').innerHTML=t;})"
    ".catch(e=>{document.getElementById('status').innerHTML='❌ Connection Lost';});"
    "}"
    "setInterval(getStatus,3000);"
    "getStatus();"
    "</script>"
    "</body>"
    "</html>";

// Handler para la página principal
static esp_err_t root_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// Handler para actualizar PID de ángulo
static esp_err_t angle_pid_handler(httpd_req_t* req) {
    char buf[100];
    int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));

    if (ret == ESP_OK) {
        char kp_str[10], ki_str[10], kd_str[10];

        if (httpd_query_key_value(buf, "kp", kp_str, sizeof(kp_str)) == ESP_OK &&
            httpd_query_key_value(buf, "ki", ki_str, sizeof(ki_str)) == ESP_OK &&
            httpd_query_key_value(buf, "kd", kd_str, sizeof(kd_str)) == ESP_OK) {
            float kp = atof(kp_str);
            float ki = atof(ki_str);
            float kd = atof(kd_str);

            // Validar rangos razonables
            if (kp < 0 || kp > 100 || ki < 0 || ki > 500 || kd < 0 || kd > 50) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Values out of range");
                return ESP_FAIL;
            }

            esp_err_t result = balance_control_set_angle_pid(kp, ki, kd);

            if (result == ESP_OK) {
                char response[100];
                snprintf(response, sizeof(response), "Angle PID Updated: Kp=%.2f Ki=%.2f Kd=%.2f",
                         kp, ki, kd);
                httpd_resp_send(req, response, strlen(response));

                ESP_LOGI(TAG, "Angle PID updated: Kp=%.2f Ki=%.2f Kd=%.2f", kp, ki, kd);
                return ESP_OK;
            } else {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to update PID");
                return ESP_FAIL;
            }
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
    return ESP_FAIL;
}

// Handler para actualizar PID de posición
static esp_err_t position_pid_handler(httpd_req_t* req) {
    char buf[100];
    int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));

    if (ret == ESP_OK) {
        char kp_str[10], ki_str[10], kd_str[10];

        if (httpd_query_key_value(buf, "kp", kp_str, sizeof(kp_str)) == ESP_OK &&
            httpd_query_key_value(buf, "ki", ki_str, sizeof(ki_str)) == ESP_OK &&
            httpd_query_key_value(buf, "kd", kd_str, sizeof(kd_str)) == ESP_OK) {
            float kp = atof(kp_str);
            float ki = atof(ki_str);
            float kd = atof(kd_str);

            // Validar rangos razonables
            if (kp < 0 || kp > 50 || ki < 0 || ki > 50 || kd < 0 || kd > 10) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Values out of range");
                return ESP_FAIL;
            }

            esp_err_t result = balance_control_set_position_pid(kp, ki, kd);

            if (result == ESP_OK) {
                char response[100];
                snprintf(response, sizeof(response),
                         "Position PID Updated: Kp=%.2f Ki=%.2f Kd=%.2f", kp, ki, kd);
                httpd_resp_send(req, response, strlen(response));

                ESP_LOGI(TAG, "Position PID updated: Kp=%.2f Ki=%.2f Kd=%.2f", kp, ki, kd);
                return ESP_OK;
            } else {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to update PID");
                return ESP_FAIL;
            }
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
    return ESP_FAIL;
}

// Handler para calibración
static esp_err_t calibrate_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "Calibration requested");

    esp_err_t ret = balance_control_calibrate_offset();

    if (ret == ESP_OK) {
        httpd_resp_send(req, "Calibration Complete!", 21);
    } else {
        httpd_resp_send(req, "Calibration Failed", 18);
    }

    return ESP_OK;
}

// Handler para parada de emergencia
static esp_err_t stop_handler(httpd_req_t* req) {
    ESP_LOGW(TAG, "Emergency stop requested");
    balance_control_stop();
    httpd_resp_send(req, "System Stopped", 14);
    return ESP_OK;
}

// Handler para obtener estado
static esp_err_t status_handler(httpd_req_t* req) {
    char response[300];

    // Mostrar estado del sistema
    snprintf(response, sizeof(response),
             "<div class='value'>System Active</div>"
             "<div style='font-size:12px;color:#aaa;margin-top:5px'>"
             "✓ WiFi AP: %s<br>"
             "✓ HTTP Server: Running<br>"
             "✓ Control Loop: Active<br>"
             "💡 Check serial monitor for real-time sensor data"
             "</div>",
             WIFI_SSID);

    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// Configurar rutas del servidor
static void configure_routes(httpd_handle_t server) {
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_register_uri_handler(server, &root);

    httpd_uri_t angle_pid = {.uri = "/api/angle", .method = HTTP_GET, .handler = angle_pid_handler};
    httpd_register_uri_handler(server, &angle_pid);

    httpd_uri_t position_pid = {
        .uri = "/api/position", .method = HTTP_GET, .handler = position_pid_handler};
    httpd_register_uri_handler(server, &position_pid);

    httpd_uri_t stop = {.uri = "/api/stop", .method = HTTP_GET, .handler = stop_handler};
    httpd_register_uri_handler(server, &stop);

    httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};
    httpd_register_uri_handler(server, &status);
}

// Event handler para WiFi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                               void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Station connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Station disconnected");
    }
}

esp_err_t web_interface_init(void) {
    ESP_LOGI(TAG, "Initializing web interface...");

    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Crear interfaz de red AP
    esp_netif_create_default_wifi_ap();

    // Configuración WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {.ssid = WIFI_SSID,
               .ssid_len = strlen(WIFI_SSID),
               .password = WIFI_PASS,
               .max_connection = 4,
               .authmode = WIFI_AUTH_WPA2_PSK},
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s, Password=%s", WIFI_SSID, WIFI_PASS);
    ESP_LOGI(TAG, "Connect to WiFi and navigate to: http://192.168.4.1");

    // Iniciar servidor HTTP
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 10;

    if (httpd_start(&server, &config) == ESP_OK) {
        configure_routes(server);
        ESP_LOGI(TAG, "HTTP server started on port 80");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}

esp_err_t web_interface_stop(void) {
    if (server) {
        httpd_stop(server);
        ESP_LOGI(TAG, "HTTP server stopped");
    }
    return ESP_OK;
}
