#include "wifi.h"

#include "comune.h"
#include "credentials.h"

#include <string.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include "telnet_server.h"
#include "time_sync.h"
#include "buffers_psram.h"
#include "core1.h"
#include "door.h"

#include <sys/socket.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>

#define MAX_FD_INFO_LENGTH 256


void start_webserver_if_not_running(void);
void stop_webserver(void);

// Dichiarazione funzione CLI (implementata in console.cpp)
extern "C" void cli_exec_line(const char* line);

static const char *TAG = "WIFI";

// Buffer temporaneo statico per copia encoder (32KB)
static EXT_RAM_BSS_ATTR EncoderData temp_encoder_buffer[ENCODER_BUFFER_SIZE] __attribute__((aligned(4)));

// Gestione client WebSocket per broadcast
#define MAX_WS_CLIENTS 4
static int g_ws_client_fds[MAX_WS_CLIENTS] = {-1, -1, -1, -1}; // Inizializza a -1
static int g_ws_client_count = 0;

// Gestione client WebSocket per CLI diagnostico
#define MAX_CLI_CLIENTS 2
static int g_cli_client_fds[MAX_CLI_CLIENTS] = {-1, -1}; // Inizializza a -1
static int g_cli_client_count = 0;

// Buffer per cattura output CLI - RIPRISTINATO SISTEMA ORIGINALE
static char cli_output_buffer[2048];
static size_t cli_output_pos = 0;
static bool cli_output_capture = false;

// Funzioni per gestione client WebSocket
void register_ws_client(int fd) {
    if (g_ws_client_count < MAX_WS_CLIENTS) {
        g_ws_client_fds[g_ws_client_count] = fd;
        g_ws_client_count++;
        ESP_LOGI(TAG, "Client WS registrato, totale: %d", g_ws_client_count);
    }
}

static int find_ws_client_by_fd(int fd) {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_ws_client_fds[i] == fd) {
            return i;
        }
    }
    return -1;
}
void unregister_ws_client(int fd) {
    int i = find_ws_client_by_fd(fd);
    if (i != -1) {
        ESP_LOGI(TAG, "Tentativo di rimozione WS, fd=%d, count=%d", fd, g_ws_client_count);
        g_ws_client_fds[i] = -1;
        g_ws_client_count--;
        
        // La chiusura del socket non è più gestita qui per evitare conflitti.
        // Lascia che sia il server httpd a gestire la chiusura del socket.
        
        ESP_LOGI(TAG, "Client WS rimosso, fd=%d, totale: %d", fd, g_ws_client_count);
    }
}

// Funzioni per gestione client CLI
void register_cli_client(int fd) {
    if (g_cli_client_count < MAX_CLI_CLIENTS) {
        g_cli_client_fds[g_cli_client_count] = fd;
        g_cli_client_count++;
        ESP_LOGI(TAG, "Client CLI registrato, totale: %d", g_cli_client_count);
    }
}

void unregister_cli_client(int fd) {
    if (fd < 0) return; // Ignora fd non validi
    ESP_LOGI(TAG, "Tentativo di rimozione CLI, fd=%d, count=%d", fd, g_cli_client_count);
    
    // Cerca il file descriptor nella lista
    for (int i = 0; i < g_cli_client_count; i++) {
        if (g_cli_client_fds[i] == fd) {
            // Rimuovi l'elemento dalla lista spostando gli elementi successivi
            for (int j = i; j < g_cli_client_count - 1; j++) {
                g_cli_client_fds[j] = g_cli_client_fds[j + 1];
            }
            g_cli_client_fds[g_cli_client_count - 1] = -1; // Pulisce l'ultimo elemento
            g_cli_client_count--;
            
            // Le chiamate per la chiusura del socket sono state rimosse.
            // Lascia che sia il server a gestirne la chiusura.
            
            ESP_LOGI(TAG, "Client CLI rimosso, fd=%d, totale: %d", fd, g_cli_client_count);
            break;
        }
    }
}


// 2. AGGIUNGI questa funzione di verifica socket dopo le funzioni register/unregister esistenti
static bool is_socket_valid(int fd) {
    if (fd < 0) return false;
    
    int socket_error = 0;
    socklen_t len = sizeof(socket_error);
    
    // Controlla se il socket ha errori
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &len) != 0) {
        return false;
    }
    
    if (socket_error != 0) {
        return false;
    }
    
    // Test aggiuntivo: prova un send non-bloccante di 0 byte
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        ssize_t result = send(fd, NULL, 0, MSG_NOSIGNAL);
        fcntl(fd, F_SETFL, flags); // Ripristina flag originali
        
        if (result < 0 && (errno == ECONNRESET || errno == EPIPE || errno == ENOTCONN)) {
            return false;
        }
    }
    
    return true;
}

// SISTEMA DI CATTURA ORIGINALE RIPRISTINATO
// Funzione per catturare output CLI - chiamata da console.cpp
extern "C" void websocket_broadcast_to_cli(const char* data, size_t len) {
    if (!cli_output_capture) return;
    
    // Aggiungi al buffer di cattura (con protezione overflow)
    size_t available = sizeof(cli_output_buffer) - cli_output_pos - 1;
    if (available > 0) {
        size_t to_copy = (len < available) ? len : available;
        memcpy(cli_output_buffer + cli_output_pos, data, to_copy);
        cli_output_pos += to_copy;
        cli_output_buffer[cli_output_pos] = '\0';
    }
}

// Funzioni per gestione cattura output
static void start_output_capture() {
    cli_output_pos = 0;
    cli_output_buffer[0] = '\0';
    cli_output_capture = true;
}

static void stop_output_capture() {
    cli_output_capture = false;
}

static const char* get_captured_output() {
    return cli_output_buffer;
}


void websocket_cleanup_dead_connections(void) {
    for (int i = 0; i < g_ws_client_count; i++) {
        if (g_ws_client_fds[i] >= 0 && !is_socket_valid(g_ws_client_fds[i])) {
            ESP_LOGW(TAG, "Rilevato socket WS non valido, fd=%d", g_ws_client_fds[i]);
            struct linger linger = { .l_onoff = 1, .l_linger = 0 }; // Chiusura immediata
            setsockopt(g_ws_client_fds[i], SOL_SOCKET, SO_LINGER, &linger, sizeof(linger));
            shutdown(g_ws_client_fds[i], SHUT_RDWR);
            close(g_ws_client_fds[i]);
            g_ws_client_fds[i] = -1;
            for (int j = i; j < g_ws_client_count - 1; j++) {
                g_ws_client_fds[j] = g_ws_client_fds[j + 1];
            }
            g_ws_client_count--;
            i--;
        }
    }
    for (int i = 0; i < g_cli_client_count; i++) {
        if (g_cli_client_fds[i] >= 0 && !is_socket_valid(g_cli_client_fds[i])) {
            ESP_LOGW(TAG, "Rilevato socket CLI non valido, fd=%d", g_cli_client_fds[i]);
            struct linger linger = { .l_onoff = 1, .l_linger = 0 }; // Chiusura immediata
            setsockopt(g_cli_client_fds[i], SOL_SOCKET, SO_LINGER, &linger, sizeof(linger));
            shutdown(g_cli_client_fds[i], SHUT_RDWR);
            close(g_cli_client_fds[i]);
            g_cli_client_fds[i] = -1;
            for (int j = i; j < g_cli_client_count - 1; j++) {
                g_cli_client_fds[j] = g_cli_client_fds[j + 1];
            }
            g_cli_client_count--;
            i--;
        }
    }
    ESP_LOGI(TAG, "Cleanup completato: %d client WS, %d client CLI", g_ws_client_count, g_cli_client_count);
}

// 4. AGGIUNGI questo task (inseriscilo dopo le funzioni esistenti, prima di ws_handler)
static void websocket_cleanup_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(15000)); // Ogni 15 secondi
        websocket_cleanup_dead_connections();
    }
}


        
// =====================================
// WebSocket handler
// =====================================

esp_err_t ws_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "File descriptor attivi: ws=%d, cli=%d", g_ws_client_count, g_cli_client_count);
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket completato");
        int fd = httpd_req_to_sockfd(req);
        if (fd >= 0 && is_socket_valid(fd)) {
            register_ws_client(fd);
        } else {
            ESP_LOGW(TAG, "Socket invalido durante handshake, fd=%d", fd);
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    int client_fd = httpd_req_to_sockfd(req);
    if (client_fd < 0 || !is_socket_valid(client_fd)) {
        ESP_LOGW(TAG, "Socket invalido rilevato durante ws_handler, fd=%d", client_fd);
        if (client_fd >= 0) {
            unregister_ws_client(client_fd);
            unregister_cli_client(client_fd);
            websocket_cleanup_dead_connections();
        }
        return ESP_FAIL;
    }



    // Primo passaggio: solo per conoscere la lunghezza
    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = NULL,
        .len = 0
    };

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WS recv (len) err: %s (0x%x)", esp_err_to_name(ret), ret);
        // MIGLIORATO: Cleanup più aggressivo su errori
        unregister_ws_client(client_fd);
        unregister_cli_client(client_fd);
        
        // Forza cleanup se necessario
        websocket_cleanup_dead_connections();
        return ret;
    
    }

        // Gestione frame di chiusura
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "Ricevuto frame di chiusura WebSocket, fd=%d", client_fd);
        unregister_ws_client(client_fd);
        unregister_cli_client(client_fd);
        websocket_cleanup_dead_connections();
        return ESP_OK;
    }

    if (ws_pkt.len >= WS_FRAME_SIZE) { // >= per lasciare spazio a '\0'
        ESP_LOGW(TAG, "WS frame troppo grande: %u >= %u", (unsigned)ws_pkt.len, (unsigned)WS_FRAME_SIZE);
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Frame troppo grande");
        return ESP_FAIL;
    }

    uint8_t* wbuf = (uint8_t*) buf_get_ws_frame(200);
    if (!wbuf) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_send(req, "Buffer WS non disponibile", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    ws_pkt.payload = wbuf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, WS_FRAME_SIZE);
    if (ret != ESP_OK) {
        buf_put_ws_frame(wbuf);
        ESP_LOGE(TAG, "WS recv err: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }
    wbuf[ws_pkt.len] = '\0';
    ESP_LOGI(TAG, "WS msg (len %u): %s", (unsigned)ws_pkt.len, (const char*)wbuf);

    // --- Gestione "get_buffer": invio di 10.000 campioni in un unico frame binario ---
    if (strcmp((const char*)wbuf, "get_buffer") == 0) {
        // Questa parte richiede accesso alle variabili globali del core1
        extern volatile uint32_t i_interrupt;
        extern volatile uint16_t adc_buffer[];
       
        
        // Buffer temporaneo in PSRAM
        static EXT_RAM_BSS_ATTR uint16_t temp_buffer[10000] __attribute__((aligned(4)));
        
        // Snapshot indietro di 10k rispetto al writer
        uint32_t current_index = i_interrupt;  // indice di scrittura "istantaneo"
        uint32_t start_index   = (current_index + ADC_BUFFER_SIZE - 10000) % ADC_BUFFER_SIZE;

        // Copia con gestione wrap (nessuna sezione critica: è diagnostica)
        if (start_index + 10000 <= ADC_BUFFER_SIZE) {
            memcpy(
            temp_buffer,
            (const void*)(const volatile void*)&adc_buffer[start_index],
            10000 * sizeof(uint16_t)
            );
        } else {
            uint32_t first_chunk  = ADC_BUFFER_SIZE - start_index;
            uint32_t second_chunk = 10000 - first_chunk;
            memcpy(
                temp_buffer,
                (const void*)(const volatile void*)&adc_buffer[start_index],
                first_chunk * sizeof(uint16_t)
            );

            memcpy(
                &temp_buffer[first_chunk],
                (const void*)(const volatile void*)adc_buffer,
                second_chunk * sizeof(uint16_t)
            );
        }

        // Invio WS: frame binario unico (little-endian, identico ad Arduino)
        httpd_ws_frame_t out = {
            .final       = true,
            .fragmented  = false,
            .type        = HTTPD_WS_TYPE_BINARY,
            .payload     = (uint8_t*)temp_buffer,
            .len         = 10000 * sizeof(uint16_t)
        };
        esp_err_t s = httpd_ws_send_frame(req, &out);

        // Rilascio buffer testo e ritorno (niente analyze_buffer_32 in IDF)
        buf_put_ws_frame(wbuf);
        return s;
    }

    // --- Gestione "get_encoder_buffer": invio encoder buffer completo ---
    if (strcmp((const char*)wbuf, "get_encoder_buffer") == 0) {
        ESP_LOGI(TAG, "Inizio acquisizione encoder_buffer per client");
        
        // Snapshot dell'indice corrente
        uint32_t current_index = encoder_buffer_index;
        uint32_t start_index = (current_index - ENCODER_BUFFER_SIZE + ENCODER_BUFFER_SIZE) % ENCODER_BUFFER_SIZE;
        
        ESP_LOGI(TAG, "Copia buffer: current_index=%lu, start_index=%lu", current_index, start_index);
        
        // Copia con gestione wrap
        if (start_index + ENCODER_BUFFER_SIZE <= ENCODER_BUFFER_SIZE) {
            memcpy(temp_encoder_buffer, &encoder_buffer[start_index], ENCODER_BUFFER_SIZE * sizeof(EncoderData));
        } else {
            uint32_t first_chunk_size = ENCODER_BUFFER_SIZE - start_index;
            uint32_t second_chunk_size = ENCODER_BUFFER_SIZE - first_chunk_size;
            memcpy(temp_encoder_buffer, &encoder_buffer[start_index], first_chunk_size * sizeof(EncoderData));
            memcpy(&temp_encoder_buffer[first_chunk_size], encoder_buffer, second_chunk_size * sizeof(EncoderData));
        }
        
        ESP_LOGI(TAG, "Buffer copiato, invio binario (%u byte)", ENCODER_BUFFER_SIZE * sizeof(EncoderData));
        
        // Invio frame binario
        httpd_ws_frame_t out_bin = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_BINARY,
            .payload = (uint8_t*)temp_encoder_buffer,
            .len = ENCODER_BUFFER_SIZE * sizeof(EncoderData)
        };
        esp_err_t ret_bin = httpd_ws_send_frame(req, &out_bin);
        
        if (ret_bin != ESP_OK) {
            ESP_LOGE(TAG, "Errore invio frame binario encoder: %s", esp_err_to_name(ret_bin));
            buf_put_ws_frame(wbuf);
            return ret_bin;
        }
        
        ESP_LOGI(TAG, "encoder_buffer inviato al client");
        
        // Invio JSON con metadati
        char json_buffer[256];
        time_t now;
        time(&now);
        snprintf(json_buffer, sizeof(json_buffer), 
                "{\"encoder_timestamp\":%lld,\"magnitude\":%u}", 
                (long long)now, (unsigned)lastMagnitude);
        
        ESP_LOGI(TAG, "Invio JSON: %s", json_buffer);
        
        httpd_ws_frame_t out_json = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t*)json_buffer,
            .len = strlen(json_buffer)
        };
        ret = httpd_ws_send_frame(req, &out_json);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Timestamp e magnitude inviati al client");
        }
        
        buf_put_ws_frame(wbuf);
        return ret;
    }



    // --- Gestione "get_buffer_window:timestamp:duration" ---
    if (strncmp((const char*)wbuf, "get_buffer_window:", 18) == 0) {
        // Parse parametri: "get_buffer_window:1705750025:30"
        const char* params = (const char*)wbuf + 18;
        
        // Trova il primo ':'
        const char* colon = strchr(params, ':');
        if (!colon) {
            const char* error_resp = "Errore: formato comando. Usa: get_buffer_window:timestamp:duration";
            httpd_ws_frame_t error_pkt = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)error_resp,
                .len = strlen(error_resp)
            };
            httpd_ws_send_frame(req, &error_pkt);
            buf_put_ws_frame(wbuf);
            return ESP_OK;
        }
        
        // Estrai timestamp
        char timestamp_str[32];
        size_t timestamp_len = colon - params;
        if (timestamp_len >= sizeof(timestamp_str)) {
            timestamp_len = sizeof(timestamp_str) - 1;
        }
        strncpy(timestamp_str, params, timestamp_len);
        timestamp_str[timestamp_len] = '\0';
        
        // Estrai duration
        const char* duration_str = colon + 1;
        
        // Converti a numeri
        time_t center_timestamp = (time_t)atoll(timestamp_str);
        int duration_seconds = atoi(duration_str);
        
        // Validazione parametri
        if (center_timestamp <= 0 || duration_seconds <= 0 || duration_seconds > 300) {
            const char* error_resp = "Errore: parametri non validi. Timestamp > 0, duration 1-300 sec";
            httpd_ws_frame_t error_pkt = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)error_resp,
                .len = strlen(error_resp)
            };
            httpd_ws_send_frame(req, &error_pkt);
            buf_put_ws_frame(wbuf);
            return ESP_OK;
        }
        
        ESP_LOGI(TAG, "Richiesta buffer window: timestamp=%ld, duration=%d sec", 
                 (long)center_timestamp, duration_seconds);
        
        // Buffer temporaneo per campioni 
        // ===== RIUSO BUFFER ESISTENTE - STILE EMBEDDED =====
        static_assert(sizeof(EncoderDataExtended) == sizeof(uint32_t), 
                    "EncoderDataExtended deve essere esattamente 32-bit");
        static_assert(sizeof(EncoderDataExtended) == 4, 
                    "Verifica dimensione EncoderDataExtended");
        static_assert(ENCODER_BUFFER_SIZE >= 7200, 
                    "Buffer encoder troppo piccolo per finestra max");

        // Cast intelligente del buffer esistente 
        EncoderDataExtended* window_buffer = (EncoderDataExtended*)temp_encoder_buffer;
        const size_t MAX_WINDOW_SAMPLES = ENCODER_BUFFER_SIZE; // Usa tutta la capacità disponibile (16384)

        ESP_LOGI(TAG, "Riuso buffer encoder esistente: %zu campioni disponibili (%zu KB)", 
                MAX_WINDOW_SAMPLES, (MAX_WINDOW_SAMPLES * sizeof(EncoderDataExtended)) / 1024);
        
        // Estrai finestra di dati
        time_t actual_start_timestamp;
        size_t extracted_samples = get_buffer_window_for_timestamp(
            center_timestamp, duration_seconds, 
            window_buffer, MAX_WINDOW_SAMPLES,
            &actual_start_timestamp
        );
        
        if (extracted_samples == 0) {
            const char* error_resp = "Errore: nessun dato disponibile per il timestamp richiesto";
            httpd_ws_frame_t error_pkt = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)error_resp,
                .len = strlen(error_resp)
            };
            httpd_ws_send_frame(req, &error_pkt);
            buf_put_ws_frame(wbuf);
            return ESP_OK;
        }
        
        ESP_LOGI(TAG, "Estratti %zu campioni, invio frame binario (%zu byte)", 
                 extracted_samples, extracted_samples * sizeof(EncoderDataExtended));
        
        // 1. Invia frame binario con i dati
        httpd_ws_frame_t data_frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_BINARY,
            .payload = (uint8_t*)window_buffer,
            .len = extracted_samples * sizeof(EncoderDataExtended)
        };
        
        esp_err_t ret_data = httpd_ws_send_frame(req, &data_frame);
        if (ret_data != ESP_OK) {
            ESP_LOGE(TAG, "Errore invio frame binario window: %s", esp_err_to_name(ret_data));
            buf_put_ws_frame(wbuf);
            return ret_data;
        }
        
        // 2. Invia frame JSON con metadati
        char json_metadata[512];
        time_t actual_end_timestamp = actual_start_timestamp + duration_seconds + 2;
        
        snprintf(json_metadata, sizeof(json_metadata),
            "{"
            "\"type\":\"buffer_window_metadata\","
            "\"timestamp_center\":%ld,"
            "\"timestamp_start\":%ld,"
            "\"timestamp_end\":%ld,"
            "\"duration_seconds\":%d,"
            "\"sample_count\":%zu,"
            "\"sample_interval_ms\":100"
            "}",
            (long)center_timestamp,
            (long)actual_start_timestamp,
            (long)actual_end_timestamp,
            duration_seconds,
            extracted_samples
        );
        
        httpd_ws_frame_t metadata_frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t*)json_metadata,
            .len = strlen(json_metadata)
        };
        
        ret = httpd_ws_send_frame(req, &metadata_frame);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Buffer window inviato: %zu campioni, metadati: %s", 
                     extracted_samples, json_metadata);
        } else {
            ESP_LOGE(TAG, "Errore invio metadati window: %s", esp_err_to_name(ret));
        }
        
        buf_put_ws_frame(wbuf);
        return ret;
    }

    // --- Gestione cambio modalità porta ---
    if (strncmp((const char*)wbuf, "set_door_mode:", 14) == 0) {
        const char* mode_str = (const char*)wbuf + 14;
        DoorMode old_mode = config.door_mode;
        DoorMode new_mode = old_mode;
        
        // Parse della modalità 
        ESP_LOGI(TAG, "Modalità porta vecchia  %d letta %s", old_mode, mode_str);
        if (strcmp(mode_str, "AUTO") == 0) new_mode = AUTO;
        else if (strcmp(mode_str, "ALWAYS_OPEN") == 0) new_mode = ALWAYS_OPEN;
        else if (strcmp(mode_str, "ALWAYS_CLOSED") == 0) new_mode = ALWAYS_CLOSED;
        else {
            // Modalità non valida
            const char* resp = "Modalità non valida";
            httpd_ws_frame_t resp_pkt = {
                .final = true,
                .fragmented = false, 
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)resp,
                .len = strlen(resp)
            };
            httpd_ws_send_frame(req, &resp_pkt);
            buf_put_ws_frame(wbuf);
            return ESP_OK;
        }
        
        if (new_mode != old_mode) {
            // Aggiorna configurazione
            config.door_mode = new_mode;
            save_config();
            

            
            ESP_LOGI(TAG, "Modalità porta cambiata da %d a %d", old_mode, new_mode);
            
            // Broadcast a tutti i client (placeholder)
            ESP_LOGI(TAG, "Broadcast modalità porta: %d", new_mode);
        }
        
        // Risposta al client che ha fatto la richiesta
        const char* resp = "Mode updated";
        httpd_ws_frame_t resp_pkt = {
            .final = true,
            .fragmented = false, 
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t*)resp,
            .len = strlen(resp)
        };
        ret = httpd_ws_send_frame(req, &resp_pkt);
        
        buf_put_ws_frame(wbuf);
        return ret;
    }

    // --- Gestione comandi CLI con cattura output - SISTEMA ORIGINALE ---
    if (strncmp((const char*)wbuf, "cli:", 4) == 0) {
        const char* command = (const char*)wbuf + 4;
        
        // Registra il client per ricevere log (se non già registrato)
        int client_fd = httpd_req_to_sockfd(req);
        bool already_registered = false;
        for (int i = 0; i < g_cli_client_count; i++) {
            if (g_cli_client_fds[i] == client_fd) {
                already_registered = true;
                break;
            }
        }
        if (!already_registered) {
            register_cli_client(client_fd);
        }
        
        ESP_LOGI(TAG, "Comando CLI ricevuto: %s", command);
        
        // Attiva cattura output
        start_output_capture();
        
        // Esegui il comando - l'output verrà catturato nel buffer
        cli_exec_line(command);
        
        // Ferma cattura e ottieni risultato
        stop_output_capture();
        const char* output = get_captured_output();
        
        ESP_LOGI(TAG, "Output catturato (%d char): %s", strlen(output), output);
        
        // Invia la risposta al client WebSocket
        if (strlen(output) > 0) {
            httpd_ws_frame_t resp_pkt = {
                .final = true,
                .fragmented = false, 
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)output,
                .len = strlen(output)
            };
            ret = httpd_ws_send_frame(req, &resp_pkt);
        } else {
            // Risposta vuota - invia conferma
            const char* resp = "Comando eseguito (nessun output)";
            httpd_ws_frame_t resp_pkt = {
                .final = true,
                .fragmented = false, 
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t*)resp,
                .len = strlen(resp)
            };
            ret = httpd_ws_send_frame(req, &resp_pkt);
        }
        
        buf_put_ws_frame(wbuf);
        return ret;
    }

    // Risposta di cortesia per altri messaggi
    const char *resp = "Messaggio ricevuto dal server!";
    httpd_ws_frame_t resp_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t*)resp,
        .len = strlen(resp)
    };
    ret = httpd_ws_send_frame(req, &resp_pkt);

    buf_put_ws_frame(wbuf);
    ESP_LOGI(TAG, "File descriptor attivi: ws=%d, cli=%d", g_ws_client_count, g_cli_client_count);
    return (ret == ESP_OK) ? ESP_OK : ret;
}
// =====================================
// Wi-Fi events & init - Con funzioni di tracking diagnostico
// =====================================

// Variabili di tracking diagnostico WiFi (spostate qui da wifi_utils.cpp)
static uint32_t wifi_reconnect_count = 0;
static uint8_t last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
static time_t last_disconnect_time = 0;
static time_t connection_start_time = 0;
static uint32_t total_disconnect_count = 0;

// Funzioni di tracking WiFi integrate
static void wifi_track_connection_start(void) {
    time(&connection_start_time);
    ESP_LOGI(TAG, "Tracking: connessione iniziata");
}

static void wifi_track_disconnect(uint8_t reason) {
    last_disconnect_reason = reason;
    time(&last_disconnect_time);
    total_disconnect_count++;
    ESP_LOGI(TAG, "Tracking: disconnessione #%lu, motivo: %d", total_disconnect_count, reason);
}

static void wifi_track_reconnect(void) {
    wifi_reconnect_count++;
    wifi_track_connection_start();
    ESP_LOGI(TAG, "Tracking: riconnessione #%lu", wifi_reconnect_count);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        wifi_rssi = 0;
        // Reset contatore client WebSocket
        g_ws_client_count = 0;
        g_cli_client_count = 0;
        ESP_LOGI(TAG, "Spegne LED Wi-Fi");
        gpio_set_level(WIFI_LED, LED_OFF);
        
        // Track disconnessione
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        wifi_track_disconnect(disconnected->reason);
        
        esp_wifi_connect();
        ESP_LOGW(TAG, "Connessione WiFi persa, riconnessione...");
        
        // Track tentativo riconnessione
        wifi_track_reconnect();
        
        stop_webserver();
        telnet_stop();
        time_sync_stop();  // opzionale; se lo lasci attivo fallirà e riproverà 

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connesso! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        
        // Track connessione stabilita
        wifi_track_connection_start();
        
        telnet_start(2323);   // porta di default
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            wifi_rssi = ap_info.rssi;
            ESP_LOGI(TAG, "RSSI Wi-Fi corrente: %d dBm", wifi_rssi);
        } else {
            ESP_LOGE(TAG, "Errore nella lettura delle informazioni dell'AP");
        }

        ESP_LOGI(TAG, "Accende LED Wi-Fi");
        gpio_set_level(WIFI_LED, LED_ON);

        // Avvio SNTP (time sync) all'ottenimento dell'IP
        time_sync_start();

        // Avvia il webserver fuori dal callback (task dedicata), solo se non già avviato
        start_webserver_if_not_running();
    }
}

void setup_wifi(void) {
    // (opzionale) inizializza i pool qui, oppure in app_main()
    buffers_psram_init();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Errore NVS, tentativo di erase...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    connection_start_time = 0;
    last_disconnect_time = 0;
    total_disconnect_count = 0;
    wifi_reconnect_count = 0;

    ESP_LOGI(TAG, "Inizializzazione WiFi completata");
    //xTaskCreate(websocket_cleanup_task, "ws_cleanup", 2048, NULL, 1, NULL);
    //ESP_LOGI(TAG, "WebSocket cleanup task avviato");

    ESP_LOGI(TAG, "Inizializzazione WiFi completata");
}

// wifi_utils.cpp - File incluso in wifi.cpp
// Contiene solo la diagnostica WiFi, il resto è stato spostato nel main

// =====================================
// Diagnostica WiFi completa
// =====================================
extern "C" void get_wifi_status(char* buffer, size_t buffer_size, const char* subcommand) {
    if (strlen(subcommand) == 0) {
        time_t now;
        time(&now);
        struct tm *tm_now = localtime(&now);
        char time_str[20];
        strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_now);
        
        // Informazioni base di configurazione
        char mac_str[18] = "N/A";
        uint8_t mac[6];
        if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
        
        // Stato connessione e qualità segnale
        const char* status_str = wifi_connected ? "CONNECTED" : "DISCONNECTED";
        int8_t current_rssi = wifi_rssi;
        
        // Aggiorna RSSI se connesso
        if (wifi_connected) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                current_rssi = ap_info.rssi;
            }
        }
        
        // Conversione RSSI in percentuale e "barre"
        int rssi_percent = 0;
        int rssi_bars = 0;
        if (current_rssi >= -30) { rssi_percent = 100; rssi_bars = 4; }
        else if (current_rssi >= -50) { rssi_percent = 75; rssi_bars = 3; }
        else if (current_rssi >= -70) { rssi_percent = 50; rssi_bars = 2; }
        else if (current_rssi >= -90) { rssi_percent = 25; rssi_bars = 1; }
        
        // Informazioni IP
        char ip_str[16] = "N/A", gw_str[16] = "N/A", mask_str[16] = "N/A";
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
                snprintf(gw_str, sizeof(gw_str), IPSTR, IP2STR(&ip_info.gw));
                snprintf(mask_str, sizeof(mask_str), IPSTR, IP2STR(&ip_info.netmask));
            }
        }
        
        // Durata connessione
       // Durata connessione - CON CONTROLLI DI VALIDITÀ
        char connection_duration[32] = "N/A";
        if (wifi_connected && connection_start_time > 0) {
            // Controllo specifico: se connection_start_time è troppo vecchio (pre-2020),
            // significa che è stato impostato prima della sincronizzazione NTP
            if (connection_start_time < 1577836800) { // 1 gennaio 2020
                snprintf(connection_duration, sizeof(connection_duration), "In attesa sync NTP");
            } else {
                time_t time_diff = now - connection_start_time;
                
                if (time_diff < 0 || time_diff > (7 * 24 * 3600)) {
                    snprintf(connection_duration, sizeof(connection_duration), "ERRORE_TEMPO");
                } else {
                    uint32_t duration_sec = (uint32_t)time_diff;
                    uint32_t days = duration_sec / (24 * 3600);
                    uint32_t hours = (duration_sec % (24 * 3600)) / 3600;
                    uint32_t minutes = (duration_sec % 3600) / 60;
                    uint32_t seconds = duration_sec % 60;
                    
                    if (days > 0) {
                        snprintf(connection_duration, sizeof(connection_duration), 
                                "%lud %02lu:%02lu:%02lu", 
                                (unsigned long)days, (unsigned long)hours, 
                                (unsigned long)minutes, (unsigned long)seconds);
                    } else {
                        snprintf(connection_duration, sizeof(connection_duration), 
                                "%02lu:%02lu:%02lu", 
                                (unsigned long)hours, (unsigned long)minutes, 
                                (unsigned long)seconds);
                    }
                }
            }
        }
        
        // Modalità power saving
        wifi_ps_type_t ps_type;
        const char* ps_str = "N/A";
        if (esp_wifi_get_ps(&ps_type) == ESP_OK) {
            switch (ps_type) {
                case WIFI_PS_NONE: ps_str = "NONE"; break;
                case WIFI_PS_MIN_MODEM: ps_str = "MIN_MODEM"; break;
                case WIFI_PS_MAX_MODEM: ps_str = "MAX_MODEM"; break;
                default: ps_str = "UNKNOWN"; break;
            }
        }
        
        // Motivo ultima disconnessione
        const char* disconnect_reason = "N/A";
        char disconnect_time_str[20] = "N/A";
        if (last_disconnect_time > 0) {
            struct tm *tm_disc = localtime(&last_disconnect_time);
            strftime(disconnect_time_str, sizeof(disconnect_time_str), "%H:%M:%S", tm_disc);
            
            // Traduzione motivi comuni
            switch (last_disconnect_reason) {
                case WIFI_REASON_UNSPECIFIED: disconnect_reason = "Non specificato"; break;
                case WIFI_REASON_AUTH_EXPIRE: disconnect_reason = "Auth scaduta"; break;
                case WIFI_REASON_AUTH_LEAVE: disconnect_reason = "Auth terminata"; break;
                case WIFI_REASON_ASSOC_EXPIRE: disconnect_reason = "Assoc scaduta"; break;
                case WIFI_REASON_ASSOC_TOOMANY: disconnect_reason = "Troppe assoc"; break;
                case WIFI_REASON_NOT_AUTHED: disconnect_reason = "Non autenticato"; break;
                case WIFI_REASON_NOT_ASSOCED: disconnect_reason = "Non associato"; break;
                case WIFI_REASON_ASSOC_LEAVE: disconnect_reason = "Assoc terminata"; break;
                case WIFI_REASON_BEACON_TIMEOUT: disconnect_reason = "Beacon timeout"; break;
                case WIFI_REASON_NO_AP_FOUND: disconnect_reason = "AP non trovato"; break;
                case WIFI_REASON_AUTH_FAIL: disconnect_reason = "Auth fallita"; break;
                case WIFI_REASON_ASSOC_FAIL: disconnect_reason = "Assoc fallita"; break;
                case WIFI_REASON_HANDSHAKE_TIMEOUT: disconnect_reason = "Handshake timeout"; break;
                default: disconnect_reason = "Altro"; break;
            }
        }
        int active_fds = 0;
        for (int fd = 0; fd < 20; fd++) { // Limite configurato a 20
            // Su ESP-IDF, un FD è valido se fstat() non restituisce errore
            struct stat fd_stat;
            if (fstat(fd, &fd_stat) == 0) {
                active_fds++;
            }
        }
        
        snprintf(buffer, buffer_size,
            "=== STATO WIFI ===\n"
            "Timestamp: %s\n"
            "\n"
            "CONFIGURAZIONE:\n"
            "  SSID: %s  |  MAC: %s\n"
            "  Power saving: %s\n"
            "\n" 
            "CONNESSIONE:\n"
            "  Stato: %s  |  Durata: %s\n"
            "  IP: %s  |  Gateway: %s\n"
            "  Netmask: %s\n"
            "\n"
            "SEGNALE:\n"
            "  RSSI: %d dBm (%d%%, %d barre)\n"
            "\n"
            "RISORSE SISTEMA:\n"
            "  File descriptor attivi: %d/20\n"
            "\n"
            "WEBSOCKET:\n"
            "  Client WS: %d  |  Client CLI: %d\n"
            "\n"
            "STATISTICHE:\n"
            "  Riconnessioni: %lu  |  Disconnessioni tot: %lu\n"
            "\n"
            "ULTIMA DISCONNESSIONE:\n"
            "  Motivo: %s  |  Quando: %s\n",
            
            time_str,
            WIFI_SSID, mac_str,
            ps_str,
            status_str, connection_duration,
            ip_str, gw_str,
            mask_str,
            current_rssi, rssi_percent, rssi_bars,
            active_fds,
            g_ws_client_count, g_cli_client_count,
            (unsigned long)wifi_reconnect_count, (unsigned long)total_disconnect_count,
            disconnect_reason, disconnect_time_str
        );
    } else if (strcmp(subcommand, "scan") == 0) {
        // Subcomando per scan reti
        snprintf(buffer, buffer_size,
            "=== SCAN RETI WIFI ===\n"
            "Funzione scan non ancora implementata.\n"
            "Richiede wifi_scan_start() asincrono."
        );
    } else {
        snprintf(buffer, buffer_size,
            "Subcomando '%s' non riconosciuto.\n"
            "Subcomandi disponibili: scan",
            subcommand
        );
    }
}