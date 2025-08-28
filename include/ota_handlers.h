#ifndef OTA_HANDLERS_H
#define OTA_HANDLERS_H

#include <esp_http_server.h>
#include <esp_err.h>

// Handler per OTA firmware
esp_err_t ota_firmware_handler(httpd_req_t* req);

// Handler per OTA SPIFFS
esp_err_t ota_spiffs_handler(httpd_req_t* req);

// Funzione per registrare gli handler OTA sul server
esp_err_t register_ota_handlers(httpd_handle_t server);

#ifdef __cplusplus
extern "C" {
#endif

// Eventuali funzioni C-compatible
// (al momento non necessarie)

#ifdef __cplusplus
}
#endif

#endif // OTA_HANDLERS_H