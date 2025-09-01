// include/console.h
#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

// Tipi per il sistema di logging
typedef enum {
    LOG_ERROR = 0,
    LOG_WARN  = 1,
    LOG_INFO  = 2,
    LOG_DEBUG = 3,
    LOG_VERBOSE = 4
} log_level_t;

// ============================
// API principali
// ============================

// Avvia il sistema console unificato
void console_start(void);

// Esegue una linea di comando (usato sia da seriale che da telnet)
void cli_exec_line(const char* line);

// ============================
// Sistema di logging unificato
// ============================

// Logging senza tag (per sostituire printf/std::printf)
void unified_log_raw(const char* fmt, ...);

// Gestione stato log (UNICO PUNTO DI CONTROLLO)
void logs_mute(bool on);
void logs_resume(void);
bool logs_are_muted(void);

// Gestione timer auto-resume (usabile da tutte le interfacce)
void schedule_resume_timer_public(void);
void cancel_resume_timer_public(void);

// Gestione livelli di log
void set_log_level(log_level_t level);
log_level_t get_log_level(void);

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_H