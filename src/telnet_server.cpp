#include "telnet_server.h"
#include "console.h"          // per exec e log-mute/resume
#include <string.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <esp_log.h>

static const char* TAG = "TELNET";

#ifndef TELNET_STACK
#define TELNET_STACK 4096
#endif
#ifndef TELNET_PRIO
#define TELNET_PRIO  5
#endif
#ifndef TELNET_AUTORESTORE_MS
#define TELNET_AUTORESTORE_MS 60000
#endif

// Stato globale (una sola sessione)
static TaskHandle_t     s_listener_task = nullptr;
static TaskHandle_t     s_session_task  = nullptr;
static TimerHandle_t    s_resume_timer  = nullptr;
static int              s_listen_sock   = -1;
static int              s_client_sock   = -1;
static volatile bool    s_client_paused = false;  // pausa CLI lato rete
static volatile bool    s_running       = false;

static void autoresume_kick() {
    if (!s_resume_timer) return;
    xTimerStop(s_resume_timer, 0);
    xTimerChangePeriod(s_resume_timer, pdMS_TO_TICKS(TELNET_AUTORESTORE_MS), 0);
    xTimerStart(s_resume_timer, 0);
}
static void autoresume_cancel() {
    if (!s_resume_timer) return;
    xTimerStop(s_resume_timer, 0);
}
static void on_resume_timer(TimerHandle_t) {
    logs_resume();              // ripristina i log globali
    s_client_paused = false;    // esci dalla "modalità prompt" lato telnet
}

// Best-effort send (non blocca; se la socket non è pronta, si perde il pezzo)
void telnet_write_best_effort(const char* data, size_t len) {
    int cs = s_client_sock;
    if (cs < 0 || !data || len == 0) return;
    // MSG_DONTWAIT = non bloccare
    int r = send(cs, data, len, MSG_DONTWAIT);
    (void)r;
}

bool telnet_has_client(void) {
    return s_client_sock >= 0;
}

static void close_client() {
    if (s_client_sock >= 0) {
        shutdown(s_client_sock, SHUT_RDWR);
        close(s_client_sock);
        s_client_sock = -1;
    }
    s_client_paused = false;
    autoresume_cancel();
}

static void session_task(void* pv) {
    (void)pv;
    ESP_LOGI(TAG, "Sessione aperta");

    // editor riga minimal con echo (stile seriale)
    char line[256];
    size_t pos = 0;
    bool last_was_cr = false;

    // prompt helper
    auto send_prompt = [&](){
        static const char* p = "(log in pausa) > ";
        send(s_client_sock, p, strlen(p), 0);
    };

    // all’avvio i log scorrono; qualunque tasto mette in pausa
    for (;;) {
        char c;
        int n = recv(s_client_sock, &c, 1, 0);
        if (n == 0 || n == -1) {
            // chiuso dal peer o errore
            break;
        }

        if (!s_client_paused) {
            // qualunque carattere -> pausa globale
            logs_mute(true);
            s_client_paused = true;
            pos = 0;
            send_prompt();
            autoresume_kick();
        }

        // normalizza CRLF -> un solo '\n'
        if (c == '\r') { last_was_cr = true; c = '\n'; }
        else if (c == '\n') { if (last_was_cr){ last_was_cr = false; continue; } }
        else { last_was_cr = false; }

        // backspace
        if (c == 0x08 || c == 0x7F) {
            if (pos) {
                pos--;
                const char bs[3] = {'\b',' ','\b'};
                send(s_client_sock, bs, 3, 0);
            }
            autoresume_kick();
            continue;
        }

        if (c == '\n') {
            line[pos] = '\0';
            // trim finale
            while (pos && (line[pos-1]==' '||line[pos-1]=='\t')) { line[--pos] = '\0'; }

            if (pos == 0) {
                // invio a vuoto: resta in pausa
                send_prompt();
                autoresume_kick();
                continue;
            }

            // esegui il comando con lo stesso parser della seriale
            cli_exec_line(line);     // <--- UN SOLO PUNTO DI VERITÀ

            pos = 0;

            // se i log sono ancora mutati -> resta al prompt
            if (logs_are_muted()) {
                send_prompt();
                autoresume_kick();
            } else {
                // i log sono attivi: esci dalla modalità prompt (niente echo)
                s_client_paused = false;
                autoresume_cancel();
                static const char* nl = "\r\n";
                send(s_client_sock, nl, 2, 0);
            }
            continue;
        }

        // caratteri stampabili con echo
        if (c >= 32 && c < 127) {
            if (pos < sizeof(line)-1) {
                line[pos++] = c;
                send(s_client_sock, &c, 1, 0);
            }
            autoresume_kick();
        }
        // altri caratteri: ignora
    }

    ESP_LOGI(TAG, "Sessione chiusa");
    close_client();
    s_session_task = nullptr;
    vTaskDelete(NULL);
}

static void listener_task(void* pv) {
    int port = (int)(intptr_t)pv;

    // socket di ascolto
    s_listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (s_listen_sock < 0) {
        ESP_LOGE(TAG, "socket() fallita");
        s_listener_task = nullptr;
        vTaskDelete(NULL);
        return;
    }

    int yes = 1;
    setsockopt(s_listen_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons((uint16_t)port);

    if (bind(s_listen_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind(%d) fallita", port);
        close(s_listen_sock);
        s_listen_sock = -1;
        s_listener_task = nullptr;
        vTaskDelete(NULL);
        return;
    }
    listen(s_listen_sock, 1);
    ESP_LOGI(TAG, "In ascolto su porta %d", port);

    // timer auto-resume condiviso da questa sessione
    if (!s_resume_timer) {
        s_resume_timer = xTimerCreate("tn_autoresume", pdMS_TO_TICKS(TELNET_AUTORESTORE_MS),
                                      pdFALSE, nullptr, on_resume_timer);
    }

    while (s_running) {
        struct sockaddr_storage src_addr;
        socklen_t addr_len = sizeof(src_addr);
        int sock = accept(s_listen_sock, (struct sockaddr*)&src_addr, &addr_len);
        if (sock < 0) {
            if (!s_running) break;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (s_client_sock >= 0) {
            // già occupato
            static const char* busy = "Console occupata. Riprova piu' tardi.\r\n";
            send(sock, busy, strlen(busy), 0);
            shutdown(sock, SHUT_RDWR);
            close(sock);
            continue;
        }

        s_client_sock = sock;
        xTaskCreatePinnedToCore(
            session_task, "telnet_session",
            TELNET_STACK, nullptr,
            TELNET_PRIO, &s_session_task,
        0 /* core 0 */
        );

    }

    if (s_listen_sock >= 0) {
        shutdown(s_listen_sock, SHUT_RDWR);
        close(s_listen_sock);
        s_listen_sock = -1;
    }
    if (s_resume_timer) {
        xTimerDelete(s_resume_timer, 0);
        s_resume_timer = nullptr;
    }
    s_listener_task = nullptr;
    vTaskDelete(NULL);
}

void telnet_start(int port) {
    if (s_listener_task) return;
    s_running = true;
    xTaskCreatePinnedToCore(
    listener_task, "telnet_listen",
    TELNET_STACK, (void*)(intptr_t)port,
    TELNET_PRIO, &s_listener_task,
    0 /* core 0 */
);
}

void telnet_stop(void) {
    s_running = false;
    close_client();
    if (s_listen_sock >= 0) {
        shutdown(s_listen_sock, SHUT_RDWR);
        close(s_listen_sock);
        s_listen_sock = -1;
    }
}
