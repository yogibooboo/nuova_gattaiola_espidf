#include "wifi.h"
#include "comune.h"
#include "credentials.h"
#include <string.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include <nvs_flash.h>

static const char *TAG = "WIFI";
static httpd_handle_t server = NULL;

static esp_err_t root_handler(httpd_req_t *req) {
    const char* resp_str = "Server HTTP & WebSocket attivo!";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket completato");
        return ESP_OK;
    }
    return ESP_OK;
}

static esp_err_t start_webserver(void) {
    ESP_LOGI(TAG, "Avvio del web server...");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Inizializzazione server HTTP...");
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Server HTTP avviato con successo");
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };       
        ESP_LOGI(TAG, "Registrazione handler root...");
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
        ESP_LOGI(TAG, "Registrazione handler WebSocket...");
        httpd_register_uri_handler(server, &ws_uri);
        ESP_LOGI(TAG, "Web server completamente configurato");
    } else {
        ESP_LOGE(TAG, "Errore nell'avvio del server HTTP");
    }
    return ESP_OK;
}

static void start_webserver_task(void *pvParameters) {
    start_webserver();
    vTaskDelete(NULL); // Termina il task dopo l'avvio
}

void stop_webserver(void) {
    if (server) {
        ESP_LOGI(TAG, "Arresto del web server...");
        httpd_stop(server);
        server = NULL;
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        wifi_rssi = 0;
        ESP_LOGI(TAG, "Spegne LED Wi-Fi");
        gpio_set_level(WIFI_LED, LED_OFF);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Connessione WiFi persa, riconnessione...");
        stop_webserver();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connesso! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            wifi_rssi = ap_info.rssi;
            ESP_LOGI(TAG, "RSSI Wi-Fi corrente: %d dBm", wifi_rssi);
        } else {
            ESP_LOGE(TAG, "Errore nella lettura delle informazioni dell'AP");
        }
        ESP_LOGI(TAG, "Accende LED Wi-Fi");
        gpio_set_level(WIFI_LED, LED_ON);
        xTaskCreate(start_webserver_task, "WebServerTask", 4096, NULL, 5, NULL);
    }
}

void setup_wifi(void) {
    // Inizializzazione NVS
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