#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include "esp_err.h"



// Inicializa el servidor web y punto de acceso WiFi
esp_err_t web_interface_init(void);

// Detiene el servidor web
esp_err_t web_interface_stop(void);


#endif  // WEB_INTERFACE_H
