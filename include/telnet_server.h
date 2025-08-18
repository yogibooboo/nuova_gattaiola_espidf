#pragma once
#include <stddef.h>   // <— serve per size_t
#ifdef __cplusplus
extern "C" {
#endif

// Avvia/ferma il server "telnet-like" (TCP raw)
void telnet_start(int port);      // es. telnet_start(2323)
void telnet_stop(void);

// Scrittura best-effort dalla pipeline dei log verso il client attivo.
// Non blocca: se la socket è piena, scarta.
void telnet_write_best_effort(const char* data, size_t len);

// Ritorna true se c'è una sessione attiva (connessa)
bool telnet_has_client(void);

#ifdef __cplusplus
}
#endif
