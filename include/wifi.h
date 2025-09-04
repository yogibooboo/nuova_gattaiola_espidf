#ifndef WIFI_H
#define WIFI_H

#include "comune.h"
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Dichiarazioni funzioni WiFi e WebSocket
void setup_wifi(void);

// Funzioni per gestione client WebSocket
void register_ws_client(int fd);
void unregister_ws_client(int fd);
void broadcast_door_mode_change(DoorMode mode);
void add_door_mode_log(DoorMode mode);

// WebSocket handler (esportato per http_server.cpp)
esp_err_t ws_handler(httpd_req_t *req);

// Dimensioni buffer definite (devono corrispondere a buffers_psram.h)
#ifndef WS_FRAME_SIZE
#define WS_FRAME_SIZE 1024
#endif

#ifndef POST_BODY_SIZE  
#define POST_BODY_SIZE (16 * 1024)
#endif

#ifndef FILE_CHUNK_SIZE
#define FILE_CHUNK_SIZE (8 * 1024)
#endif

#ifdef __cplusplus
extern "C" {
#endif

void get_wifi_status(char* buffer, size_t buffer_size, const char* subcommand);

// Funzioni C-compatible se necessario
// (al momento tutte le funzioni sono già C++ compatible)

#ifdef __cplusplus
}
#endif

#endif // WIFI_H