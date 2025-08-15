#include "comune.h"
#include "wifi.h"
#include "credentials.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_http_server.h>

static const char *TAG = "WIFI";

static httpd_handle_t server = NULL;


// === HANDLER ROOT ===
static esp_err_t root_handler(httpd_req_t *req) {
    const char resp[] = "Server HTTP & WebSocket attivo!";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// === HANDLER WS ===
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket completato");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));

    // Legge solo la lunghezza
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Errore lettura frame WS (len): %d", ret);
        return ret;
    }

    // Alloca buffer per payload
    ws_pkt.payload = (uint8_t *)malloc(ws_pkt.len + 1);
    if (!ws_pkt.payload) {
        ESP_LOGE(TAG, "Memoria insufficiente");
        return ESP_ERR_NO_MEM;
    }

    // Legge il payload
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret == ESP_OK) {
        ws_pkt.payload[ws_pkt.len] = '\0';
        ESP_LOGI(TAG, "WS ricevuto: %s", (char *)ws_pkt.payload);

        // Rispondo con lo stesso messaggio
        httpd_ws_frame_t ws_rsp;
        memset(&ws_rsp, 0, sizeof(ws_rsp));
        ws_rsp.type = HTTPD_WS_TYPE_TEXT;
        ws_rsp.payload = ws_pkt.payload;
        ws_rsp.len = strlen((char *)ws_pkt.payload);
        httpd_ws_send_frame(req, &ws_rsp);
    }

    free(ws_pkt.payload);
    return ret;
}

// === AVVIO SERVER ===
static esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };       
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t ws_uri = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = NULL,
            .is_websocket = true,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        httpd_register_uri_handler(server, &ws_uri);
    }
    return ESP_OK;
}

// === STOP SERVER ===
static esp_err_t stop_webserver(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}

// === EVENTI Wi-Fi ===
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
        ESP_LOGW(TAG, "Connessione WiFi persa, riconnessione...");
        stop_webserver();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connesso! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        start_webserver();
    }
}

// === FUNZIONI PLACEHOLDER ===
void ws_send_all(const char* message) {
    // TODO: broadcast ai client WS
}

void handle_set_door_mode(const char* mode_str) {
    // TODO: gestione comandi
}

// === AVVIO Wi-Fi ===
void setup_wifi() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NOT_INITIALIZED) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}