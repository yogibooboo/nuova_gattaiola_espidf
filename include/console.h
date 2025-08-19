// include/console.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif




// Avvia la task di console (CLI su seriale)
void console_start(void);
// Esegui una riga di comando (riusata dal telnet)
void cli_exec_line(const char* line);

// Facoltativo, per il futuro: notifica disconnessioni (Wi-Fi/BLE)
// così la console può riattivare i log subito.
// void console_notify_disconnect(void);

void logs_mute(bool on);
void logs_resume(void);
bool logs_are_muted(void);

#ifdef __cplusplus
}
#endif
