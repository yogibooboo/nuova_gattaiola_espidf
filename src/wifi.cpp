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

void start_webserver_if_not_running(void);
void stop_webserver(void);

// Dichiarazione funzione CLI (implementata in console.cpp)
extern "C" void cli_exec_line(const char* line);

static const char *TAG = "WIFI";

// Buffer temporaneo statico per copia encoder (32KB)
static EXT_RAM_BSS_ATTR EncoderData temp_encoder_buffer[ENCODER_BUFFER_SIZE] __attribute__((aligned(4)));

// Gestione client WebSocket per broadcast
#define MAX_WS_CLIENTS 4
static int g_ws_client_fds[MAX_WS_CLIENTS];
static int g_ws_client_count = 0;

// Gestione client WebSocket per CLI diagnostico
#define MAX_CLI_CLIENTS 2
static int g_cli_client_fds[MAX_CLI_CLIENTS];
static int g_cli_client_count = 0;

// Buffer per cattura output CLI
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

void unregister_ws_client(int fd) {
    for (int i = 0; i < g_ws_client_count; i++) {
        if (g_ws_client_fds[i] == fd) {
            // Sposta gli elementi rimanenti
            for (int j = i; j < g_ws_client_count - 1; j++) {
                g_ws_client_fds[j] = g_ws_client_fds[j + 1];
            }
            g_ws_client_count--;
            ESP_LOGI(TAG, "Client WS rimosso, totale: %d", g_ws_client_count);
            break;
        }
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
    for (int i = 0; i < g_cli_client_count; i++) {
        if (g_cli_client_fds[i] == fd) {
            // Sposta gli elementi rimanenti
            for (int j = i; j < g_cli_client_count - 1; j++) {
                g_cli_client_fds[j] = g_cli_client_fds[j + 1];
            }
            g_cli_client_count--;
            ESP_LOGI(TAG, "Client CLI rimosso, totale: %d", g_cli_client_count);
            break;
        }
    }
}

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

void broadcast_door_mode_change(DoorMode mode) {
    const char* mode_names[] = {"AUTO", "ALWAYS_OPEN", "ALWAYS_CLOSED"};
    char message[64];
    snprintf(message, sizeof(message), "door_mode:%s", mode_names[mode]);
    
    // Nota: in ESP-IDF non abbiamo accesso diretto ai fd dei client WebSocket
    // Questa implementazione è un placeholder - la logica di broadcast reale
    // richiederebbe modifiche più profonde al sistema HTTP server
    ESP_LOGI(TAG, "Broadcast modalità porta: %s (placeholder)", message);
}

void add_door_mode_log(DoorMode mode) {
    time_t now;
    time(&now);
    
    LogEntry* entry = &log_buffer[log_count % LOG_BUFFER_SIZE];
    entry->timestamp = now;
    entry->device_code = 0;
    entry->country_code = 0;
    entry->authorized = (mode == ALWAYS_OPEN);
    
    const char* mode_events[] = {"Modalità Automatica", "Modalità Sempre Aperto", "Modalità Sempre Chiuso"};
    strncpy(entry->event, mode_events[mode], sizeof(entry->event) - 1);
    entry->event[sizeof(entry->event) - 1] = '\0';
    
    log_count++;
    
    ESP_LOGI(TAG, "Aggiunta entry log cambio modalità: %s", mode_events[mode]);
}

// =====================================
// WebSocket handler
// =====================================

esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket completato");
        // Registra il client per il broadcast (placeholder - non funzionale in ESP-IDF standard)
        register_ws_client(httpd_req_to_sockfd(req));
        return ESP_OK;
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
        // Deregistra client in caso di errore
        int client_fd = httpd_req_to_sockfd(req);
        unregister_ws_client(client_fd);
        unregister_cli_client(client_fd);
        return ret;
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
        
        ESP_LOGI(TAG, "Copia buffer: current_index=%u, start_index=%u", current_index, start_index);
        
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
            
            // Broadcast a tutti i client
            broadcast_door_mode_change(new_mode);
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

    // --- Gestione comandi CLI con cattura output ---
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
    return (ret == ESP_OK) ? ESP_OK : ret;
}

// =====================================
// Wi-Fi events & init
// =====================================

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
        esp_wifi_connect();
        ESP_LOGW(TAG, "Connessione WiFi persa, riconnessione...");
        stop_webserver();
        telnet_stop();
        time_sync_stop();  // opzionale; se lo lasci attivo fallirà e riproverà 

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connesso! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
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

    ESP_LOGI(TAG, "Inizializzazione WiFi completata");
}