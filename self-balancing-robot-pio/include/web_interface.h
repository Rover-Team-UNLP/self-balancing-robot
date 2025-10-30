#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el servidor web y punto de acceso WiFi
esp_err_t web_interface_init(void);

// Detiene el servidor web
esp_err_t web_interface_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // WEB_INTERFACE_H
