// File: wifi.h

#ifndef WIFI_H
#define WIFI_H

#include "comune.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "cJSON.h"

// Dichiarazione di variabili e funzioni
extern volatile bool wifi_connected;
extern portMUX_TYPE doorModeMux;

extern SemaphoreHandle_t doorModeSemaphore;

// Funzioni per la gestione della configurazione, del WiFi e dei WebSocket
void setup_wifi();
void handle_set_door_mode(const char* message);
void save_config();

#endif