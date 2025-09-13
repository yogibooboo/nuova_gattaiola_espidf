// src/console.cpp - Sistema di logging e console unificato
#include "console.h"
#include "comune.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "telnet_server.h"  
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "sdkconfig.h"
#include <esp_system.h>

#include <esp_log.h>
#include <time.h>
#include "time_sync.h"

#include "core1.h"
#include "door.h"
#include "wifi.h"


// ============================
// Configurazione
// ============================

#ifndef CONSOLE_AUTORESTORE_MS
#define CONSOLE_AUTORESTORE_MS 20000  // 20 secondi per test
#endif

#ifndef CONSOLE_TASK_STACK
#define CONSOLE_TASK_STACK 4096
#endif

#ifndef CONSOLE_TASK_PRIO
#define CONSOLE_TASK_PRIO 5
#endif

// ============================
// Sistema di logging unificato - STATO UNICO
// ============================

// UNICO STATO GLOBALE - tutte le interfacce usano questo
static volatile bool s_logs_muted = false;
static volatile log_level_t s_current_log_level = LOG_INFO;
static TimerHandle_t s_resume_timer = nullptr;

// Buffer ultimo log periodico (implementato in main.cpp, allocato in PSRAM)
extern EXT_RAM_BSS_ATTR char last_periodic_log[512];

// Buffer unificato in PSRAM per tutti gli output lunghi CLI
static EXT_RAM_BSS_ATTR char cli_unified_buffer[2048];

// Dichiarazione funzione WebSocket (implementata in wifi.cpp)
extern "C" void websocket_broadcast_to_cli(const char* data, size_t len);

extern "C" void get_system_diagnostics(char* buffer, size_t buffer_size, const char* subcommand);


// Scrive su seriale, telnet e WebSocket CLI
static void write_both_interfaces(const char* data, size_t len) {
    // Seriale
    fwrite(data, 1, len, stdout);
    fflush(stdout);

    // Telnet con conversione LF -> CRLF
    if (telnet_has_client()) {
        size_t start = 0;
        for (size_t i = 0; i < len; ++i) {
            if (data[i] == '\n') {
                if (i > start) telnet_write_best_effort(data + start, i - start);
                telnet_write_best_effort("\r\n", 2);
                start = i + 1;
            }
        }
        if (start < len) telnet_write_best_effort(data + start, len - start);
    }

    // WebSocket CLI (invio diretto senza conversione CRLF)
    websocket_broadcast_to_cli(data, len);
}

// Router principale per tutti i log ESP-IDF
static int vprintf_router(const char* fmt, va_list ap) {
    if (s_logs_muted) return 0;

    char buf[512];
    va_list copy;
    va_copy(copy, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);
    if (n < 0) return 0;

    size_t to_write = (n < (int)sizeof(buf)) ? (size_t)n : sizeof(buf) - 1;
    write_both_interfaces(buf, to_write);
    return n;
}

// ============================
// API pubblica per logging unificato
// ============================

// Logging unificato senza tag (per sostituire std::printf)
extern "C" void unified_log_raw(const char* fmt, ...) {
    if (s_logs_muted) return;
    
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    
    if (n > 0) {
        size_t len = (n < (int)sizeof(buf)) ? (size_t)n : sizeof(buf) - 1;
        write_both_interfaces(buf, len);
    }
}

// Imposta livello di log
extern "C" void set_log_level(log_level_t level) {
    s_current_log_level = level;
}

extern "C" log_level_t get_log_level(void) {
    return s_current_log_level;
}

// ============================
// Gestione timer - funzioni interne
// ============================

static void schedule_resume_timer() {
    if (!s_resume_timer) return;
    xTimerStop(s_resume_timer, 0);
    xTimerChangePeriod(s_resume_timer, pdMS_TO_TICKS(CONSOLE_AUTORESTORE_MS), 0);
    xTimerStart(s_resume_timer, 0);
}

static void cancel_resume_timer() {
    if (!s_resume_timer) return;
    xTimerStop(s_resume_timer, 0);
}

static void on_resume_timer(TimerHandle_t) {
    logs_resume();
}

// ============================
// Gestione stato console - UNICO PUNTO DI CONTROLLO
// ============================

extern "C" void logs_mute(bool on) {
    s_logs_muted = on;
}

extern "C" void logs_resume(void) {
    s_logs_muted = false;
}

extern "C" bool logs_are_muted(void) { 
    return s_logs_muted; 
}

// API per gestire il timer da qualsiasi interfaccia
extern "C" void schedule_resume_timer_public() {
    schedule_resume_timer();
}

extern "C" void cancel_resume_timer_public() {
    cancel_resume_timer();
}

// ============================
// Utility CLI
// ============================

static void cli_write(const char* s, size_t len) {
    write_both_interfaces(s, len);
}

static void cli_puts(const char* s) { 
    cli_write(s, strlen(s)); 
}

// Funzione per output a chunk sicuro di buffer esistenti
static void cli_printbuf(const char* buffer) {
    if (!buffer) return;
    
    size_t len = strlen(buffer);
    const size_t chunk_size = 200;
    
    for (size_t i = 0; i < len; i += chunk_size) {
        size_t remaining = len - i;
        size_t to_send = (remaining > chunk_size) ? chunk_size : remaining;
        write_both_interfaces(buffer + i, to_send);
        
        // Pausa solo se ci sono altri chunk da inviare
        if (remaining > chunk_size) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// Versione modificata di cli_printf che usa il buffer unificato
static void cli_printf(const char* fmt, ...) {
    va_list ap; 
    va_start(ap, fmt);
    int n = vsnprintf(cli_unified_buffer, sizeof(cli_unified_buffer), fmt, ap);
    va_end(ap);
    
    if (n <= 0) return;
    
    size_t len = (size_t)((n < (int)sizeof(cli_unified_buffer)) ? n : (int)sizeof(cli_unified_buffer));
    
    // Output a chunk per evitare stack overflow
    cli_printbuf(cli_unified_buffer);
}

static void rtrim(char* s) {
    size_t n = strlen(s);
    while (n && (s[n-1] == ' ' || s[n-1] == '\t' || s[n-1] == '\r' || s[n-1] == '\n')) {
        s[--n] = '\0';
    }
}

static bool streq_ci(const char* a, const char* b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
        ++a; ++b;
    }
    return *a == '\0' && *b == '\0';
}

// ============================
// Parser comandi
// ============================

static void print_help() {
    cli_puts("\nComandi disponibili:\n");
    cli_puts("  help            - mostra questo aiuto\n");
    cli_puts("  log             - mostra ultimo log periodico\n");
    cli_puts("  log level <0-4> - imposta livello log (0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=VERBOSE)\n");
    cli_puts("  time            - mostra ora corrente\n");
    cli_puts("  ntp now         - forza sincronizzazione NTP\n");
    cli_puts("  ntp status      - stato sincronizzazione NTP\n");
    cli_puts("  memory          - stato dettagliato della memoria\n");
    cli_puts("  decod           - stato decoder RFID\n");
    cli_puts("  door            - stato door task e sensori\n");
    cli_puts("  wifi            - stato connessione WiFi\n");  
    cli_puts("  wifi scan       - scan reti WiFi disponibili\n");  // 
    cli_puts("  system          - diagnostiche avanzate sistema\n");
    cli_puts("  system flash    - informazioni memoria flash\n");
    cli_puts("  system task     - stato task FreeRTOS\n");
    cli_puts("  buff            - stato encoder buffer esteso\n");
    
}

static void exec_command(const char* line) {
    char cmd[256];
    strncpy(cmd, line, sizeof(cmd)-1);
    cmd[sizeof(cmd)-1] = '\0';
    rtrim(cmd);

    if (cmd[0] == '\0') return;

    if (streq_ci(cmd, "help") || streq_ci(cmd, "?")) {
        print_help();
        return;
    }

    if (streq_ci(cmd, "log")) {
        if (last_periodic_log[0] != '\0') {
            cli_printf("%s\n", last_periodic_log);
        } else {
            cli_puts("Nessun log periodico disponibile ancora\n");
        }
        return;
    }

    // Comando per livello log
    if (strncmp(cmd, "log level ", 10) == 0) {
        int level = atoi(cmd + 10);
        if (level >= 0 && level <= 4) {
            set_log_level((log_level_t)level);
            const char* level_names[] = {"ERROR", "WARN", "INFO", "DEBUG", "VERBOSE"};
            cli_printf("OK: livello log impostato a %d (%s)\n", level, level_names[level]);
        } else {
            cli_puts("ERR: livello deve essere 0-4\n");
        }
        return;
    }

    if (streq_ci(cmd, "time")) {
        time_t now = 0;
        time(&now);
        struct tm lt; 
        localtime_r(&now, &lt);

        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);

        // Calcola uptime dall'ultimo reset
        TickType_t uptime_ticks = xTaskGetTickCount();
        uint32_t uptime_ms = uptime_ticks * portTICK_PERIOD_MS;
        uint32_t uptime_sec = uptime_ms / 1000;
        uint32_t days = uptime_sec / (24 * 3600);
        uint32_t hours = (uptime_sec % (24 * 3600)) / 3600;
        uint32_t minutes = (uptime_sec % 3600) / 60;
        uint32_t seconds = uptime_sec % 60;

        cli_printf("Ora locale: %s\n", buf);
        cli_printf("Epoch: %ld\n", (long)now);
        cli_printf("Valid sync: %s\n", time_sync_is_valid() ? "yes" : "no");
        
        if (days > 0) {
            cli_printf("Uptime: %u giorni, %02u:%02u:%02u\n", days, hours, minutes, seconds);
        } else {
            cli_printf("Uptime: %02u:%02u:%02u\n", hours, minutes, seconds);
        }
        
        return;
    }

    if (streq_ci(cmd, "ntp now")) {
        time_sync_force();
        cli_puts("Forzato un tentativo di sincronizzazione.\n");
        return;
    }

    if (streq_ci(cmd, "ntp status")) {
        cli_printf("Ultimo server: %s\n", time_sync_server_used());

        time_t last = time_sync_last_epoch();
        if (last > 0) {
            struct tm lt; 
            localtime_r(&last, &lt);
            char buf[64];
            strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
            cli_printf("Ultimo sync: %s\n", buf);
        } else {
            cli_puts("Ultimo sync: n/a\n");
        }
        cli_printf("Valido: %s\n", time_sync_is_valid() ? "yes" : "no");
        return;
    }

    if (streq_ci(cmd, "memory") || streq_ci(cmd, "mem")) {
        cli_puts("=== STATO MEMORIA ESP32-S3 ===\n");
        
        // Heap generale (principalmente SRAM interna)
        size_t free_heap = esp_get_free_heap_size();
        size_t min_free_heap = esp_get_minimum_free_heap_size();
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
        
        cli_printf("HEAP PRINCIPALE (SRAM):\n");
        cli_printf("  Libera: %u bytes (%.1f KB)\n", 
                   (unsigned)free_heap, (float)free_heap / 1024.0f);
        cli_printf("  Minimo storico: %u bytes (%.1f KB)\n", 
                   (unsigned)min_free_heap, (float)min_free_heap / 1024.0f);
        cli_printf("  Totale: %u bytes (%.1f KB)\n", 
                   (unsigned)total_heap, (float)total_heap / 1024.0f);
        cli_printf("  Utilizzata: %.1f%%\n\n", 
                   (float)(total_heap - free_heap) * 100.0f / total_heap);

        // PSRAM (SPIRAM)
        size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        
        cli_printf("PSRAM (SPIRAM):\n");
        cli_printf("  Libera: %u bytes (%.1f KB)\n", 
                   (unsigned)free_psram, (float)free_psram / 1024.0f);
        cli_printf("  Totale: %u bytes (%.1f KB)\n", 
                   (unsigned)total_psram, (float)total_psram / 1024.0f);
        cli_printf("  Utilizzata: %.1f%%\n\n", 
                   (float)(total_psram - free_psram) * 100.0f / total_psram);

        // Memoria per DMA (importante per Bluetooth)
        size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
        size_t total_dma = heap_caps_get_total_size(MALLOC_CAP_DMA);
        
        cli_printf("MEMORIA DMA-CAPABLE:\n");
        cli_printf("  Libera: %u bytes (%.1f KB)\n", 
                   (unsigned)free_dma, (float)free_dma / 1024.0f);
        cli_printf("  Totale: %u bytes (%.1f KB)\n", 
                   (unsigned)total_dma, (float)total_dma / 1024.0f);
        cli_printf("  Utilizzata: %.1f%%\n\n", 
                   (float)(total_dma - free_dma) * 100.0f / total_dma);

        // Memoria interna (SRAM) - esclude PSRAM
        size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t total_internal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
        
        cli_printf("SRAM INTERNA PURA:\n");
        cli_printf("  Libera: %u bytes (%.1f KB)\n", 
                   (unsigned)free_internal, (float)free_internal / 1024.0f);
        cli_printf("  Totale: %u bytes (%.1f KB)\n", 
                   (unsigned)total_internal, (float)total_internal / 1024.0f);
        cli_printf("  Utilizzata: %.1f%%\n\n", 
                   (float)(total_internal - free_internal) * 100.0f / total_internal);

        // Stack del task corrente
        UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
        cli_printf("STACK TASK CORRENTE:\n");
        cli_printf("  Stack libero minimo: %u bytes\n", 
                   (unsigned)(stack_high_water * sizeof(StackType_t)));

        // Informazioni aggiuntive
        cli_printf("\nINFO AGGIUNTIVE:\n");
        cli_printf("  Task totali: %u\n", (unsigned)uxTaskGetNumberOfTasks());
        

        
        return;
    }
    
    if (strncmp(cmd, "decod", 5) == 0) {
        const char* subcmd = (strlen(cmd) > 6) ? cmd + 6 : "";
        
        // Usa il buffer unificato direttamente - zero copie
        get_decoder_status(cli_unified_buffer, sizeof(cli_unified_buffer), subcmd);
        cli_printbuf(cli_unified_buffer);
        return;
    }

    if (strncmp(cmd, "door", 4) == 0) {
    const char* subcmd = (strlen(cmd) > 5) ? cmd + 5 : "";
    
    //extern void get_door_status(char*, size_t, const char*);
    get_door_status(cli_unified_buffer, sizeof(cli_unified_buffer), subcmd);
    cli_printbuf(cli_unified_buffer);
    return;
    }

    if (strncmp(cmd, "buff", 4) == 0) {
    const char* subcmd = (strlen(cmd) > 5) ? cmd + 5 : "";
    
    //extern void get_buffer_extended_status(char*, size_t, const char*);
    get_buffer_extended_status(cli_unified_buffer, sizeof(cli_unified_buffer));
    cli_printbuf(cli_unified_buffer);
    return;
    }
    
    
    
    if (strncmp(cmd, "wifi", 4) == 0) {
        const char* subcmd = (strlen(cmd) > 5) ? cmd + 5 : "";
        
        extern void get_wifi_status(char*, size_t, const char*);
        get_wifi_status(cli_unified_buffer, sizeof(cli_unified_buffer), subcmd);
        cli_printbuf(cli_unified_buffer);
        return;
    }
        if (strncmp(cmd, "system", 6) == 0) {
        const char* subcmd = (strlen(cmd) > 7) ? cmd + 7 : "";
        
        get_system_diagnostics(cli_unified_buffer, sizeof(cli_unified_buffer), subcmd);
        cli_printbuf(cli_unified_buffer);
        return;
    }
    // Comando sconosciuto
    cli_puts("ERR: comando sconosciuto. Digita 'help'\n");
}

extern "C" void cli_exec_line(const char* line) {
    exec_command(line);
}

// ============================
// Task della console
// ============================

static void console_task(void*) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    unified_log_raw("Console pronta. Premi INVIO per mettere in pausa i log e digitare comandi. Digita 'help'.\n");

    bool printed_prompt = false;
    bool last_was_cr = false;
    
    char line[256];
    size_t pos = 0;

    auto ensure_prompt = [&](){
        if (!printed_prompt && logs_are_muted()){
            cli_puts("(log in pausa) > ");
            printed_prompt = true;
        }
    };
    
    auto handle_pause = [&](){
        if (!logs_are_muted()){
            logs_mute(true);
            printed_prompt = false;
            pos = 0;
            schedule_resume_timer();
        }
        ensure_prompt();
    };

    for (;;){
        // Resetta prompt se i log sono ripartiti (timer auto-resume)
        if (!logs_are_muted()) {
            printed_prompt = false;
        }

        int c = fgetc(stdin);
        if (c == '\r') {
            last_was_cr = true;
            c = '\n';
        } else if (c == '\n') {
            if (last_was_cr) {
                last_was_cr = false;
                continue;
            }
        } else {
            last_was_cr = false;
        }
        
        if (c == EOF){
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Qualsiasi tasto mette in pausa
        if (!logs_are_muted() && c != '\n'){
            logs_mute(true);
            printed_prompt = false;
            pos = 0;
            schedule_resume_timer();
        }
        ensure_prompt();

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
                // I log ripartono - non serve più il prompt
                printed_prompt = false;
                cancel_resume_timer();
                cli_puts("\n"); // Nuova riga per separare dal prompt
            } else {
                ensure_prompt();
                schedule_resume_timer();
            }
            continue;
        }

        // Backspace
        if (c == 0x08 || c == 0x7F){
            if (pos){
                pos--;
                cli_puts("\b \b");
            }
            schedule_resume_timer();
            continue;
        }

        // Caratteri stampabili
        if (c >= 32 && c < 127){
            if (pos < sizeof(line) - 1){
                line[pos++] = (char)c;
                char echo[2] = {(char)c, '\0'};
                cli_puts(echo);
            }
            schedule_resume_timer();
            continue;
        }
    }
}

// ============================
// API pubblica
// ============================

extern "C" void console_start(void) {
    // Instrada tutti i log ESP-IDF attraverso il nostro router
    esp_log_set_vprintf(vprintf_router);

    // Crea timer auto-resume
    if (!s_resume_timer) {
        s_resume_timer = xTimerCreate(
            "console_resume",
            pdMS_TO_TICKS(CONSOLE_AUTORESTORE_MS),
            pdFALSE,
            nullptr,
            on_resume_timer
        );
    }

    // Avvia task console
    xTaskCreatePinnedToCore(
        console_task, "console",
        CONSOLE_TASK_STACK, nullptr,
        CONSOLE_TASK_PRIO, nullptr,
        0 /* core 0 */
    );
}