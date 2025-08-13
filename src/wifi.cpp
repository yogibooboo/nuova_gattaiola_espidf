#include "common.h"
#include "wifi.h"
#include "credentials.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include <esp_websocket_server.h>

static const char *TAG = "WIFI";

static httpd_handle_t server = NULL;
static esp_websocket_server_handle_t ws_server = NULL;
volatile bool wifi_connected = false;

static esp_err_t start_webserver(void);
static esp_err_t stop_webserver(void);

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
        ESP_LOGW(TAG, "Connessione WiFi persa, riconnessione in corso...");
        stop_webserver();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connesso! IP assegnato: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        start_webserver();
    }
}

void ws_send_all(const char* message) {
    // Implementazione placeholder
    // In questa fase non è necessario che funzioni, ma il compilatore la richiede
}

void handle_set_door_mode(const char* mode_str) {
    // Implementazione placeholder
}

static esp_err_t ws_handler(httpd_req_t *req) {
    // Implementazione placeholder
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *req) {
    // Implementazione placeholder
    return ESP_OK;
}

static esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
        .user_ctx = NULL
    };
    httpd_uri_t ws_uri = {
        .uri      = "/ws",
        .method   = HTTP_GET,
        .handler  = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &ws_uri);
    }
    return ESP_OK;
}

static esp_err_t stop_webserver(void) {
    if (server) {
        httpd_stop(server);
    }
    return ESP_OK;
}

void setup_wifi() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NOT_INITIALIZED) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}