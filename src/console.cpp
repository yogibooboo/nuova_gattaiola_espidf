// src/console.cpp
#include "console.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "telnet_server.h"  
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "sdkconfig.h"

#include <esp_log.h>

// ============================
// Configurazione console
// ============================

// Tempo dopo il quale i log si riattivano automaticamente se la console resta inattiva (ms)
#ifndef CONSOLE_AUTORESTORE_MS
#define CONSOLE_AUTORESTORE_MS 60000  // 60 secondi
#endif

// Stack della task console
#ifndef CONSOLE_TASK_STACK
#define CONSOLE_TASK_STACK 4096
#endif

// Priorità della task console
#ifndef CONSOLE_TASK_PRIO
#define CONSOLE_TASK_PRIO 5
#endif

// ============================
// Stato interno
// ============================

//static const char* TAGC = "CLI";  // usato solo per printf, non per ESP_LOGx

static volatile bool s_logs_muted = false;     // true -> log ESP mutati
static TimerHandle_t s_resume_timer = nullptr; // timer auto-ripristino log

// ============================
// Router dei log ESP-IDF
// ============================



static int vprintf_router(const char* fmt, va_list ap) {
    if (s_logs_muted) return 0;

    char buf[512];
    va_list copy;
    va_copy(copy, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);
    if (n < 0) return 0;

    // stampa locale (seriale)
    fwrite(buf, 1, (size_t) ((n < (int)sizeof(buf)) ? n : (int)sizeof(buf)-1), stdout);
    fflush(stdout);

    // duplica verso telnet client, se presente
    if (telnet_has_client()) {
        telnet_write_best_effort(buf, (size_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf)-1));
    }
    return n;
}


// API visibili dal resto del progetto
extern "C" void logs_mute(bool on) {
    s_logs_muted = on;
    esp_log_level_set("*", on ? ESP_LOG_NONE
                              : (esp_log_level_t)CONFIG_LOG_DEFAULT_LEVEL);
}

extern "C" void logs_resume(void) {
    s_logs_muted = false;
    esp_log_level_set("*", (esp_log_level_t)CONFIG_LOG_DEFAULT_LEVEL);
}

extern "C" bool logs_are_muted(void) { return s_logs_muted; }



// ============================
// Utility locali
// ============================

static inline void schedule_resume_timer() {
    if (!s_resume_timer) return;
    // (ri)programma il timer one-shot
    xTimerStop(s_resume_timer, 0);
    xTimerChangePeriod(s_resume_timer, pdMS_TO_TICKS(CONSOLE_AUTORESTORE_MS), 0);
    xTimerStart(s_resume_timer, 0);
}

static inline void cancel_resume_timer() {
    if (!s_resume_timer) return;
    xTimerStop(s_resume_timer, 0);
}

// NIENTE ESP_LOGx nella callback del timer!
static void on_resume_timer(TimerHandle_t) {
    // Riattiva semplicemente i log
    logs_resume();
}

// Trim spazi in coda
static void rtrim(char* s) {
    size_t n = strlen(s);
    while (n && (s[n-1] == ' ' || s[n-1] == '\t' || s[n-1] == '\r' || s[n-1] == '\n')) {
        s[--n] = '\0';
    }
}

// Confronto case-insensitive su stringhe semplici
static bool streq_ci(const char* a, const char* b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
        ++a; ++b;
    }
    return *a == '\0' && *b == '\0';
}

// ============================
// Parser comandi minimal
// ============================

static void print_help() {
    printf("\nComandi disponibili:\n");
    printf("  help            - mostra questo aiuto\n");
    printf("  log on          - riattiva i log\n");
    printf("  log off         - mette in pausa i log (resta nel prompt)\n");
    printf("\nSuggerimento: premi INVIO a vuoto per mettere in pausa i log e scrivere comandi.\n\n");
    fflush(stdout);
}

static void exec_command(const char* line) {
    // Copia locale per normalizzare
    char cmd[256];
    strncpy(cmd, line, sizeof(cmd)-1);
    cmd[sizeof(cmd)-1] = '\0';
    rtrim(cmd);

    if (cmd[0] == '\0') {
        // riga vuota: niente
        return;
    }

    if (streq_ci(cmd, "help") || streq_ci(cmd, "?")) {
        print_help();
        return;
    }

    if (streq_ci(cmd, "log on")) {
        logs_resume();
        cancel_resume_timer();
        ESP_LOGI("CLI","test log dopo log on");
        printf("OK: log attivi\n");
        fflush(stdout);
        
        return;
    }

    if (streq_ci(cmd, "log off")) {
        logs_mute(true);
        schedule_resume_timer();
        printf("OK: log in pausa\n");
        fflush(stdout);
        return;
    }

    // Qui puoi aggiungere altri comandi: es. "reset", "door open", ecc.

    // Default: comando sconosciuto
    printf("ERR: comando sconosciuto. Digita 'help'\n");
    fflush(stdout);
}

extern "C" void cli_exec_line(const char* line) {
    exec_command(line);
}


// ============================
// Task della console
// ============================

static void console_task(void*) {
    // Non toccare il buffering di stdin (deve restare bufferizzato).
    // Va bene disabilitare il buffering su stdout/stderr per vedere subito il prompt.
    // setvbuf(stdin,  nullptr, _IONBF, 0); // <-- NON FARLO
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    // Messaggio iniziale
    printf("Console pronta. Premi INVIO per mettere in pausa i log e digitare comandi. Digita 'help'.\n");
    fflush(stdout);

    bool paused = false;
    bool printed_prompt = false;
    bool last_was_cr = false;
    
    char line[256];
    size_t pos = 0;
    

    auto ensure_prompt = [&](){
        if (!printed_prompt){
            printf("(log in pausa) > ");
            fflush(stdout);
            printed_prompt = true;
        }
    };
    auto enter_paused = [&](){
        if (!paused){
            logs_mute(true);                 // ferma subito i log
            paused = true;
            printed_prompt = false;
            pos = 0;
            schedule_resume_timer();         // auto-resume dopo inattività
        }
        ensure_prompt();                     // mostra il prompt UNA sola volta
    };
    auto leave_paused = [&](){
        if (paused){
            cancel_resume_timer();
            paused = false;
            printed_prompt = false;
            pos = 0;

                printf("\n");                    // riga pulita per il ritorno dei log
            fflush(stdout);
        }
    };

    for (;;){
        // Se il timer ha riattivato i log, allinea lo stato locale
        if (paused && !logs_are_muted()){
            leave_paused();
        }

        int c = fgetc(stdin);
        if (c == '\r') {
            last_was_cr = true;
            c = '\n'; // processa come newline unica
        } else if (c == '\n') {
            if (last_was_cr) {
                last_was_cr = false;
                continue;   // ignora la \n successiva a una \r
            }
        } else {
            last_was_cr = false;
        }
        if (c == EOF){
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ⬇️ QUALSIASI tasto mette in pausa i log e apre il prompt
        if (!paused){
            enter_paused();
        }

        // === editing linea con echo (siamo in pausa) ===
        if (c == '\n') {
            line[pos] = '\0';
            rtrim(line);

            if (pos == 0) {
                ensure_prompt();
                schedule_resume_timer();
                continue;
            }

            bool want_log_on = streq_ci(line, "log on");
            

            exec_command(line);
            pos = 0;

            if (want_log_on) {
                leave_paused();              // esci SUBITO: i log ripartono
            } else {
                ensure_prompt();
                schedule_resume_timer();
            }
        continue;
        }

        

        // backspace/delete
        if (c == 0x08 || c == 0x7F){
            if (pos){
                pos--;
                printf("\b \b");
                fflush(stdout);
            }
            schedule_resume_timer();
            continue;
        }

        // caratteri stampabili
        if (c >= 32 && c < 127){
            if (pos < sizeof(line) - 1){
                line[pos++] = (char)c;
                putchar(c);                   // echo locale
                fflush(stdout);
            }
            schedule_resume_timer();
            continue;
        }

        // altri caratteri: ignora
    }

}

// ============================
// Avvio console (API)
// ============================

extern "C" void console_start(void) {
    // Instrada i log ESP-IDF attraverso il router
    esp_log_set_vprintf(vprintf_router);

    // Crea il timer one-shot per auto-ripristino log
    if (!s_resume_timer) {
        s_resume_timer = xTimerCreate(
            "cli_resume",
            pdMS_TO_TICKS(CONSOLE_AUTORESTORE_MS),
            pdFALSE,             // one-shot
            nullptr,
            on_resume_timer
        );
    }

    // Crea la task della console
    xTaskCreate(console_task, "console", CONSOLE_TASK_STACK, nullptr, CONSOLE_TASK_PRIO, nullptr);
}
