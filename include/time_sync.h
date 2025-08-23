#pragma once

#include <time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Avvia SNTP con server cablati (senza DHCP), modalità IMMED (jump)
esp_err_t time_sync_start(void);

// Ferma SNTP
void time_sync_stop(void);

// Imposta TZ = Europe/Rome (CET/CEST) e chiama tzset()
void time_sync_setup_timezone(void);

// Ritorna true se abbiamo già sincronizzato almeno una volta
bool time_sync_is_valid(void);

// Epoch dell’ultimo sync riuscito (0 se mai avvenuto)
time_t time_sync_last_epoch(void);

// Forza un nuovo ciclo di sincronizzazione (stop+start)
esp_err_t time_sync_force(void);

// Ritorna il nome del server configurato in posizione 0 (indicativo)
const char* time_sync_server_used(void);

#ifdef __cplusplus
}
#endif
