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
#include <cJSON.h>
#include <driver/gpio.h> // Aggiunto per gpio_set_level

static const char *TAG = "WIFI";
static httpd_handle_t server = NULL;

// Handler generico per servire file statici da SPIFFS
static esp_err_t static_file_handler(httpd_req_t *req) {
    const char *uri = req->uri;
    char filepath[520];
    
  
    // Mappa "/" a "/index.html" e "/config" a "/config.html"
    if (strcmp(uri, "/") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/index.html");
    } else if (strcmp(uri, "/config") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/config.html");
    } else {
        snprintf(filepath, sizeof(filepath), "/spiffs%s", uri);
    }

    ESP_LOGI(TAG, "Tentativo di apertura file: %s", filepath); // Aggiungi log di debug
    FILE *fd = fopen(filepath, "r");
    if (fd == NULL) {
        ESP_LOGE(TAG, "Errore apertura file %s: %s", filepath, strerror(errno)); // Aggiungi errno per dettagli
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File non trovato");
        return ESP_FAIL;
    }

    // Imposta il tipo MIME in base all'estensione
    const char *content_type = "application/octet-stream"; // Default
    if (strstr(filepath, ".html")) {
        content_type = "text/html";
    } else if (strstr(filepath, ".png")) {
        content_type = "image/png";
    } else if (strstr(filepath, ".json")) {
        content_type = "application/json";
    } else if (strstr(filepath, ".css")) {
        content_type = "text/css";
    } else if (strstr(filepath, ".js")) {
        content_type = "application/javascript";
    }

    httpd_resp_set_type(req, content_type);

    char buf[1024];
    size_t bytes_read;
    while ((bytes_read = fread(buf, 1, sizeof(buf), fd)) > 0) {
        if (httpd_resp_send_chunk(req, buf, bytes_read) != ESP_OK) {
            fclose(fd);
            ESP_LOGE(TAG, "Errore invio file %s", filepath);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Errore invio file");
            return ESP_FAIL;
        }
    }

    fclose(fd);
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "File %s servito con successo", filepath);
    return ESP_OK;
}

static esp_err_t config_data_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Invia config.json corrente
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "door_mode", config.door_mode == AUTO ? "AUTO" : config.door_mode == ALWAYS_OPEN ? "ALWAYS_OPEN" : "ALWAYS_CLOSED");
        cJSON_AddNumberToObject(root, "DOOR_TIMEOUT", config.door_timeout);
        cJSON_AddNumberToObject(root, "STEPS_PER_MOVEMENT", config.steps_per_movement);
        cJSON_AddNumberToObject(root, "STEP_INTERVAL_US", config.step_interval_us);
        cJSON_AddNumberToObject(root, "WIFI_RECONNECT_DELAY", config.wifi_reconnect_delay);
        cJSON_AddNumberToObject(root, "UNAUTHORIZED_LOG_INTERVAL", config.unauthorized_log_interval);
        cJSON_AddBoolToObject(root, "WIFI_VERBOSE_LOG", config.wifi_verbose_log);
        cJSON_AddStringToObject(root, "motor_type", config.motor_type == SERVO ? "servo" : "step");
        cJSON_AddNumberToObject(root, "servo_open_us", config.servo_open_us);
        cJSON_AddNumberToObject(root, "servo_closed_us", config.servo_closed_us);
        cJSON_AddNumberToObject(root, "servo_transition_ms", config.servo_transition_ms);
        cJSON_AddNumberToObject(root, "config_01", config.config_01);
        cJSON_AddNumberToObject(root, "config_02", config.config_02);
        cJSON_AddNumberToObject(root, "config_03", config.config_03);
        cJSON_AddNumberToObject(root, "config_04", config.config_04);
        cJSON_AddNumberToObject(root, "config_05", config.config_05);
        cJSON_AddNumberToObject(root, "config_06", config.config_06);
        cJSON_AddNumberToObject(root, "config_07", config.config_07);
        cJSON_AddNumberToObject(root, "config_08", config.config_08);
        cJSON_AddNumberToObject(root, "config_09", config.config_09);
        cJSON_AddNumberToObject(root, "config_10", config.config_10);
        cJSON_AddNumberToObject(root, "door_rest", config.door_rest);
        cJSON_AddNumberToObject(root, "door_in", config.door_in);
        cJSON_AddNumberToObject(root, "door_out", config.door_out);

        cJSON *cats_array = cJSON_CreateArray();
        for (int i = 0; i < config.num_cats; i++) {
            cJSON *cat = cJSON_CreateObject();
            cJSON_AddNumberToObject(cat, "device_code", (double)config.authorized_cats[i].device_code);
            cJSON_AddNumberToObject(cat, "country_code", config.authorized_cats[i].country_code);
            cJSON_AddStringToObject(cat, "name", config.authorized_cats[i].name.c_str());
            cJSON_AddBoolToObject(cat, "authorized", config.authorized_cats[i].authorized);
            cJSON_AddItemToArray(cats_array, cat);
        }
        cJSON_AddItemToObject(root, "authorized_cats", cats_array);

        char *json_str = cJSON_Print(root);
        cJSON_Delete(root);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, strlen(json_str));
        free(json_str);
        return ESP_OK;
    } else if (req->method == HTTP_POST) {
        // Ricevi dati JSON
        char buf[1024] = {0};
        int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
        if (ret <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Errore ricezione dati");
            return ESP_FAIL;
        }
        buf[ret] = '\0';

        cJSON *root = cJSON_Parse(buf);
        if (root == NULL) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Errore parsing JSON");
            return ESP_FAIL;
        }

        cJSON *action = cJSON_GetObjectItem(root, "action");
        if (!action || !cJSON_IsString(action)) {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Azione non specificata");
            return ESP_FAIL;
        }

        cJSON *response = cJSON_CreateObject();
        if (strcmp(action->valuestring, "add") == 0 || strcmp(action->valuestring, "update") == 0) {
            cJSON *cats = cJSON_GetObjectItem(root, "cats");
            if (!cats || !cJSON_IsArray(cats) || cJSON_GetArraySize(cats) != 1) {
                cJSON_Delete(root);
                cJSON_Delete(response);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cats array non valido");
                return ESP_FAIL;
            }

            cJSON *cat = cJSON_GetArrayItem(cats, 0);
            uint64_t device_code = (uint64_t)cJSON_GetObjectItem(cat, "device_code")->valuedouble;
            uint16_t country_code = cJSON_GetObjectItem(cat, "country_code")->valueint;
            const char *name = cJSON_GetObjectItem(cat, "name")->valuestring;
            bool authorized = cJSON_GetObjectItem(cat, "authorized")->valueint;

            if (strcmp(action->valuestring, "add") == 0) {
                if (config.num_cats >= MAX_CATS) {
                    cJSON_Delete(root);
                    cJSON_Delete(response);
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Limite gatti raggiunto");
                    return ESP_FAIL;
                }
                config.authorized_cats[config.num_cats].device_code = device_code;
                config.authorized_cats[config.num_cats].country_code = country_code;
                config.authorized_cats[config.num_cats].name = name;
                config.authorized_cats[config.num_cats].authorized = authorized;
                config.num_cats++;
            } else {
                cJSON *original_device_code = cJSON_GetObjectItem(cat, "original_device_code");
                if (!original_device_code) {
                    cJSON_Delete(root);
                    cJSON_Delete(response);
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Original device code mancante");
                    return ESP_FAIL;
                }
                uint64_t orig_device_code = (uint64_t)original_device_code->valuedouble;
                for (int i = 0; i < config.num_cats; i++) {
                    if (config.authorized_cats[i].device_code == orig_device_code) {
                        config.authorized_cats[i].device_code = device_code;
                        config.authorized_cats[i].country_code = country_code;
                        config.authorized_cats[i].name = name;
                        config.authorized_cats[i].authorized = authorized;
                        break;
                    }
                }
            }
            save_config();
            cJSON_AddBoolToObject(response, "success", true);
        } else if (strcmp(action->valuestring, "delete") == 0) {
            cJSON *cats = cJSON_GetObjectItem(root, "cats");
            if (!cats || !cJSON_IsArray(cats) || cJSON_GetArraySize(cats) != 1) {
                cJSON_Delete(root);
                cJSON_Delete(response);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cats array non valido");
                return ESP_FAIL;
            }

            cJSON *cat = cJSON_GetArrayItem(cats, 0);
            uint64_t device_code = (uint64_t)cJSON_GetObjectItem(cat, "device_code")->valuedouble;
            for (int i = 0; i < config.num_cats; i++) {
                if (config.authorized_cats[i].device_code == device_code) {
                    for (int j = i; j < config.num_cats - 1; j++) {
                        config.authorized_cats[j] = config.authorized_cats[j + 1];
                    }
                    config.num_cats--;
                    break;
                }
            }
            save_config();
            cJSON_AddBoolToObject(response, "success", true);
        } else if (strcmp(action->valuestring, "update_params") == 0) {
            cJSON *params = cJSON_GetObjectItem(root, "params");
            if (!params) {
                cJSON_Delete(root);
                cJSON_Delete(response);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parametri non validi");
                return ESP_FAIL;
            }

            cJSON *door_mode = cJSON_GetObjectItem(params, "door_mode");
            if (door_mode && cJSON_IsString(door_mode)) {
                if (strcmp(door_mode->valuestring, "AUTO") == 0) config.door_mode = AUTO;
                else if (strcmp(door_mode->valuestring, "ALWAYS_OPEN") == 0) config.door_mode = ALWAYS_OPEN;
                else if (strcmp(door_mode->valuestring, "ALWAYS_CLOSED") == 0) config.door_mode = ALWAYS_CLOSED;
            }

            cJSON *door_timeout = cJSON_GetObjectItem(params, "DOOR_TIMEOUT");
            if (door_timeout && cJSON_IsNumber(door_timeout)) config.door_timeout = door_timeout->valueint;

            cJSON *steps_per_movement = cJSON_GetObjectItem(params, "STEPS_PER_MOVEMENT");
            if (steps_per_movement && cJSON_IsNumber(steps_per_movement)) config.steps_per_movement = steps_per_movement->valueint;

            cJSON *step_interval_us = cJSON_GetObjectItem(params, "STEP_INTERVAL_US");
            if (step_interval_us && cJSON_IsNumber(step_interval_us)) config.step_interval_us = step_interval_us->valueint;

            cJSON *wifi_reconnect_delay = cJSON_GetObjectItem(params, "WIFI_RECONNECT_DELAY");
            if (wifi_reconnect_delay && cJSON_IsNumber(wifi_reconnect_delay)) config.wifi_reconnect_delay = wifi_reconnect_delay->valueint;

            cJSON *unauthorized_log_interval = cJSON_GetObjectItem(params, "UNAUTHORIZED_LOG_INTERVAL");
            if (unauthorized_log_interval && cJSON_IsNumber(unauthorized_log_interval)) config.unauthorized_log_interval = unauthorized_log_interval->valueint;

            cJSON *wifi_verbose_log = cJSON_GetObjectItem(params, "WIFI_VERBOSE_LOG");
            if (wifi_verbose_log && cJSON_IsBool(wifi_verbose_log)) config.wifi_verbose_log = wifi_verbose_log->valueint;

            cJSON *motor_type = cJSON_GetObjectItem(params, "motor_type");
            if (motor_type && cJSON_IsString(motor_type)) {
                config.motor_type = (strcmp(motor_type->valuestring, "servo") == 0) ? SERVO : STEP;
            }

            cJSON *servo_open_us = cJSON_GetObjectItem(params, "servo_open_us");
            if (servo_open_us && cJSON_IsNumber(servo_open_us)) config.servo_open_us = servo_open_us->valueint;

            cJSON *servo_closed_us = cJSON_GetObjectItem(params, "servo_closed_us");
            if (servo_closed_us && cJSON_IsNumber(servo_closed_us)) config.servo_closed_us = servo_closed_us->valueint;

            cJSON *servo_transition_ms = cJSON_GetObjectItem(params, "servo_transition_ms");
            if (servo_transition_ms && cJSON_IsNumber(servo_transition_ms)) config.servo_transition_ms = servo_transition_ms->valueint;

            cJSON *config_01 = cJSON_GetObjectItem(params, "config_01");
            if (config_01 && cJSON_IsNumber(config_01)) config.config_01 = config_01->valueint;

            cJSON *config_02 = cJSON_GetObjectItem(params, "config_02");
            if (config_02 && cJSON_IsNumber(config_02)) config.config_02 = config_02->valueint;

            cJSON *config_03 = cJSON_GetObjectItem(params, "config_03");
            if (config_03 && cJSON_IsNumber(config_03)) config.config_03 = config_03->valueint;

            cJSON *config_04 = cJSON_GetObjectItem(params, "config_04");
            if (config_04 && cJSON_IsNumber(config_04)) config.config_04 = config_04->valueint;

            cJSON *config_05 = cJSON_GetObjectItem(params, "config_05");
            if (config_05 && cJSON_IsNumber(config_05)) config.config_05 = config_05->valueint;

            cJSON *config_06 = cJSON_GetObjectItem(params, "config_06");
            if (config_06 && cJSON_IsNumber(config_06)) config.config_06 = config_06->valueint;

            cJSON *config_07 = cJSON_GetObjectItem(params, "config_07");
            if (config_07 && cJSON_IsNumber(config_07)) config.config_07 = config_07->valueint;

            cJSON *config_08 = cJSON_GetObjectItem(params, "config_08");
            if (config_08 && cJSON_IsNumber(config_08)) config.config_08 = config_08->valueint;

            cJSON *config_09 = cJSON_GetObjectItem(params, "config_09");
            if (config_09 && cJSON_IsNumber(config_09)) config.config_09 = config_09->valueint;

            cJSON *config_10 = cJSON_GetObjectItem(params, "config_10");
            if (config_10 && cJSON_IsNumber(config_10)) config.config_10 = config_10->valueint;

            cJSON *door_rest = cJSON_GetObjectItem(params, "door_rest");
            if (door_rest && cJSON_IsNumber(door_rest)) config.door_rest = door_rest->valueint;

            cJSON *door_in = cJSON_GetObjectItem(params, "door_in");
            if (door_in && cJSON_IsNumber(door_in)) config.door_in = door_in->valueint;

            cJSON *door_out = cJSON_GetObjectItem(params, "door_out");
            if (door_out && cJSON_IsNumber(door_out)) config.door_out = door_out->valueint;

            save_config();
            cJSON_AddBoolToObject(response, "success", true);
        } else if (strcmp(action->valuestring, "reset_defaults") == 0) {
            config.door_mode = AUTO;
            config.door_timeout = 10000;
            config.steps_per_movement = 2500;
            config.step_interval_us = 500;
            config.wifi_reconnect_delay = 1000;
            config.unauthorized_log_interval = 60000;
            config.wifi_verbose_log = true;
            config.motor_type = SERVO;
            config.servo_open_us = 1800;
            config.servo_closed_us = 1200;
            config.servo_transition_ms = 1000;
            config.config_01 = 0;
            config.config_02 = 113;
            config.config_03 = 682;
            config.config_04 = 200;
            config.config_05 = 0;
            config.config_06 = 0;
            config.config_07 = 0;
            config.config_08 = 0;
            config.config_09 = 0;
            config.config_10 = 0;
            config.door_rest = 2051;
            config.door_in = 971;
            config.door_out = 3072;
            config.num_cats = 0; // Non resetta i gatti
            save_config();
            cJSON_AddBoolToObject(response, "success", true);
        } else {
            cJSON_AddBoolToObject(response, "success", false);
            cJSON_AddStringToObject(response, "error", "Azione non valida");
        }

        cJSON_Delete(root);
        char *json_str = cJSON_Print(response);
        cJSON_Delete(response);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, strlen(json_str));
        free(json_str);
        return ESP_OK;
    }

    httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
    return ESP_FAIL;
}

static esp_err_t clear_log_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        clear_log();
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "success", true);
        char *json_str = cJSON_Print(response);
        cJSON_Delete(response);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, strlen(json_str));
        free(json_str);
        return ESP_OK;
    }

    httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
    return ESP_FAIL;
}

static esp_err_t reset_system_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "success", true);
        char *json_str = cJSON_Print(response);
        cJSON_Delete(response);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, strlen(json_str));
        free(json_str);

        ESP_LOGI(TAG, "Riavvio del sistema...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return ESP_OK;
    }

    httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
    return ESP_FAIL;
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket completato");
        return ESP_OK;
    }

    uint8_t buf[128] = {0};
    httpd_ws_frame_t ws_pkt = {
        .final = false,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = buf,
        .len = 0
    };

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Errore ricezione frame WebSocket: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }

    buf[ws_pkt.len] = '\0';
    ESP_LOGI(TAG, "Messaggio ricevuto (tipo: %d, lunghezza: %d): %s", ws_pkt.type, ws_pkt.len, buf);

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        char resp[] = "Messaggio ricevuto dal server!";
        httpd_ws_frame_t resp_pkt = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t*)resp,
            .len = strlen(resp)
        };

        ret = httpd_ws_send_frame(req, &resp_pkt);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Errore invio risposta WebSocket: %s (0x%x)", esp_err_to_name(ret), ret);
        } else {
            ESP_LOGI(TAG, "Risposta inviata: %s", resp);
        }
    } else {
        ESP_LOGW(TAG, "Frame non di tipo testo, ignorato (tipo: %d)", ws_pkt.type);
    }

    return ESP_OK;
}

static esp_err_t start_webserver(void) {
    ESP_LOGI(TAG, "Avvio del web server...");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Inizializzazione server HTTP...");
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Server HTTP avviato con successo");

        // Regista prima l'handler generico (con la massima specificità)
        httpd_uri_t generic_uri = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = static_file_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        ESP_LOGI(TAG, "Registrazione handler generico per /*...");
        httpd_register_uri_handler(server, &generic_uri);
        
        // Regista gli handler specifici dopo
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = static_file_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        ESP_LOGI(TAG, "Registrazione handler statico per /...");
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t config_uri = {
            .uri = "/config",
            .method = HTTP_GET,
            .handler = static_file_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        ESP_LOGI(TAG, "Registrazione handler statico per /config...");
        httpd_register_uri_handler(server, &config_uri);

        //... (rest of your handlers)
        // Handler per /config_data (GET e POST)
        httpd_uri_t config_data_uri = {
            .uri = "/config_data",
            .method = HTTP_GET,
            .handler = config_data_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        httpd_register_uri_handler(server, &config_data_uri);
        config_data_uri.method = HTTP_POST;
        httpd_register_uri_handler(server, &config_data_uri);

        //... (rest of your handlers)
        // Handler per /clear_log
        httpd_uri_t clear_log_uri = {
            .uri = "/clear_log",
            .method = HTTP_POST,
            .handler = clear_log_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        ESP_LOGI(TAG, "Registrazione handler clear_log...");
        httpd_register_uri_handler(server, &clear_log_uri);

        //... (rest of your handlers)
        // Handler per /reset_system
        httpd_uri_t reset_system_uri = {
            .uri = "/reset_system",
            .method = HTTP_POST,
            .handler = reset_system_handler,
            .user_ctx = NULL,
            .is_websocket = false,
            .handle_ws_control_frames = NULL,
            .supported_subprotocol = NULL
        };
        ESP_LOGI(TAG, "Registrazione handler reset_system...");
        httpd_register_uri_handler(server, &reset_system_uri);

        // Handler per WebSocket
        httpd_uri_t ws_uri = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = NULL,
            .is_websocket = true,
            .handle_ws_control_frames = true,
            .supported_subprotocol = NULL
        };
        ESP_LOGI(TAG, "Registrazione handler WebSocket...");
        httpd_register_uri_handler(server, &ws_uri);

        ESP_LOGI(TAG, "Web server completamente configurato");
    } else {
        ESP_LOGE(TAG, "Errore nell'avvio del server HTTP");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void start_webserver_task(void *pvParameters) {
    start_webserver();
    vTaskDelete(NULL);
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