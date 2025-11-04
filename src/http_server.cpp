#include "ota_handlers.h"
#include "wifi.h"
#include "comune.h"

#include <string.h>
#include <errno.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_http_server.h>
#include <cJSON.h>
#include "buffers_psram.h"
#include <ctime>

// =====================================
// Dichiarazioni inline (ex http_server.h)
// =====================================
void start_webserver_if_not_running(void);
void stop_webserver(void);

static const char *TAG = "HTTP_SERVER";
static httpd_handle_t server = NULL;

// =====================================
// Helpers
// =====================================

static inline const char* mime_from_path(const char* path) {
    if (strstr(path, ".html")) return "text/html";
    if (strstr(path, ".css"))  return "text/css";
    if (strstr(path, ".js"))   return "application/javascript";
    if (strstr(path, ".json")) return "application/json";
    if (strstr(path, ".png"))  return "image/png";
    if (strstr(path, ".ico"))  return "image/x-icon";
    if (strstr(path, ".svg"))  return "image/svg+xml";
    return "application/octet-stream";
}

// Helper per inviare "503 Service Unavailable" su IDF che non ha la costante
static esp_err_t resp_503(httpd_req_t* req, const char* text) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}

// =====================================
// Helper functions per config_data (implementate per prime)
// =====================================

static esp_err_t handle_cat_add_update(cJSON *root, cJSON *response, const char* action) {
    cJSON *cats = cJSON_GetObjectItem(root, "cats");
    if (!cats || !cJSON_IsArray(cats) || cJSON_GetArraySize(cats) != 1) {
        return ESP_FAIL;
    }

    cJSON *cat = cJSON_GetArrayItem(cats, 0);
    uint64_t device_code = (uint64_t)cJSON_GetObjectItem(cat, "device_code")->valuedouble;
    uint16_t country_code = cJSON_GetObjectItem(cat, "country_code")->valueint;
    const cJSON *jname = cJSON_GetObjectItem(cat, "name");
    const cJSON *jauth = cJSON_GetObjectItem(cat, "authorized");
    const char *name = jname ? jname->valuestring : "";
    bool authorized = jauth ? (jauth->valueint != 0) : true;

    if (strcmp(action, "add") == 0) {
        if (config.num_cats >= MAX_CATS) {
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
    return ESP_OK;
}

static esp_err_t handle_cat_delete(cJSON *root, cJSON *response) {
    cJSON *cats = cJSON_GetObjectItem(root, "cats");
    if (!cats || !cJSON_IsArray(cats) || cJSON_GetArraySize(cats) != 1) {
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
    return ESP_OK;
}

static esp_err_t handle_params_update(cJSON *root, cJSON *response) {
    cJSON *params = cJSON_GetObjectItem(root, "params");
    if (!params) {
        return ESP_FAIL;
    }

    // Update dei parametri (versione compatta)
    cJSON *item;
    if ((item = cJSON_GetObjectItem(params, "door_mode")) && cJSON_IsString(item)) {
        if (strcmp(item->valuestring, "AUTO") == 0) config.door_mode = AUTO;
        else if (strcmp(item->valuestring, "ALWAYS_OPEN") == 0) config.door_mode = ALWAYS_OPEN;
        else if (strcmp(item->valuestring, "ALWAYS_CLOSED") == 0) config.door_mode = ALWAYS_CLOSED;
    }
    
    // Macro per semplificare l'aggiornamento dei parametri numerici
    #define UPDATE_NUM_PARAM(param_name, config_field) \
        if ((item = cJSON_GetObjectItem(params, param_name)) && cJSON_IsNumber(item)) \
            config.config_field = item->valueint;

    UPDATE_NUM_PARAM("DOOR_TIMEOUT", door_timeout)
    UPDATE_NUM_PARAM("STEPS_PER_MOVEMENT", steps_per_movement)
    UPDATE_NUM_PARAM("STEP_INTERVAL_US", step_interval_us)
    UPDATE_NUM_PARAM("WIFI_RECONNECT_DELAY", wifi_reconnect_delay)
    UPDATE_NUM_PARAM("UNAUTHORIZED_LOG_INTERVAL", unauthorized_log_interval)
    UPDATE_NUM_PARAM("servo_open_us", servo_open_us)
    UPDATE_NUM_PARAM("servo_closed_us", servo_closed_us)
    UPDATE_NUM_PARAM("servo_transition_ms", servo_transition_ms)
    UPDATE_NUM_PARAM("config_01", config_01)
    UPDATE_NUM_PARAM("config_02", config_02)
    UPDATE_NUM_PARAM("config_03", config_03)
    UPDATE_NUM_PARAM("config_04", config_04)
    UPDATE_NUM_PARAM("config_05", config_05)
    UPDATE_NUM_PARAM("config_06", config_06)
    UPDATE_NUM_PARAM("config_07", config_07)
    UPDATE_NUM_PARAM("config_08", config_08)
    UPDATE_NUM_PARAM("config_09", config_09)
    UPDATE_NUM_PARAM("config_10", config_10)
    UPDATE_NUM_PARAM("door_rest", door_rest)
    UPDATE_NUM_PARAM("door_in", door_in)
    UPDATE_NUM_PARAM("door_out", door_out)
    
    #undef UPDATE_NUM_PARAM

    if ((item = cJSON_GetObjectItem(params, "WIFI_VERBOSE_LOG")) && cJSON_IsBool(item))
        config.wifi_verbose_log = item->valueint;

    if ((item = cJSON_GetObjectItem(params, "motor_type")) && cJSON_IsString(item))
        config.motor_type = (strcmp(item->valuestring, "servo") == 0) ? SERVO : STEP;

    save_config();
    cJSON_AddBoolToObject(response, "success", true);
    return ESP_OK;
}

static esp_err_t handle_reset_defaults(cJSON *response) {
    // Default invariati
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
    // NON resetto i gatti
    save_config();
    cJSON_AddBoolToObject(response, "success", true);
    return ESP_OK;
}

// =====================================
// Handler statici (SPIFFS)
// =====================================

static esp_err_t static_file_handler(httpd_req_t *req) {
    const char *uri = req->uri;

    // Protezione base contro path traversal
    if (strstr(uri, "..")) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad path");
        return ESP_FAIL;
    }

    char filepath[520];

    // Mappa "/" a "/index.html", "/config" a "/config.html" e "/console" a "/console.html"
    if (strcmp(uri, "/") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/index.html");
    } else if (strcmp(uri, "/config") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/config.html");
    } else if (strcmp(uri, "/console") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/console.html");
    } else if (strcmp(uri, "/update") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/update.html");
    } else {
        snprintf(filepath, sizeof(filepath), "/spiffs%s", uri);
    }

    ESP_LOGI(TAG, "Tentativo di apertura file: %s", filepath);
    FILE *fd = fopen(filepath, "rb"); // binario
    if (fd == NULL) {
        ESP_LOGE(TAG, "Errore apertura file %s: %s", filepath, strerror(errno));
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File non trovato");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, mime_from_path(filepath));
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache"); // opzionale (dev)

    // === buffer dal pool PSRAM ===
    uint8_t* buf = (uint8_t*) buf_get_file_chunk(200);  // attende max 200ms
    if (!buf) {
        fclose(fd);
        return resp_503(req, "Buffer non disponibile");
    }

    size_t bytes_read;
    while ((bytes_read = fread(buf, 1, FILE_CHUNK_SIZE, fd)) > 0) {
        if (httpd_resp_send_chunk(req, (const char*)buf, bytes_read) != ESP_OK) {
            buf_put_file_chunk(buf);
            fclose(fd);
            ESP_LOGE(TAG, "Errore invio file %s", filepath);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Errore invio file");
            return ESP_FAIL;
        }
    }

    buf_put_file_chunk(buf);
    fclose(fd);
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "File %s servito con successo", filepath);
    return ESP_OK;
}

// =====================================
// /config_data (GET/POST)
// =====================================

static esp_err_t config_data_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        cJSON *root = cJSON_CreateObject();
        if (!root) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON alloc failed");
            return ESP_FAIL;
        }

        cJSON_AddStringToObject(root, "door_mode",
            (config.door_mode == AUTO) ? "AUTO" :
            (config.door_mode == ALWAYS_OPEN) ? "ALWAYS_OPEN" : "ALWAYS_CLOSED");

        cJSON_AddNumberToObject(root, "DOOR_TIMEOUT", config.door_timeout);
        cJSON_AddNumberToObject(root, "STEPS_PER_MOVEMENT", config.steps_per_movement);
        cJSON_AddNumberToObject(root, "STEP_INTERVAL_US", config.step_interval_us);
        cJSON_AddNumberToObject(root, "WIFI_RECONNECT_DELAY", config.wifi_reconnect_delay);
        cJSON_AddNumberToObject(root, "UNAUTHORIZED_LOG_INTERVAL", config.unauthorized_log_interval);
        cJSON_AddBoolToObject(root,   "WIFI_VERBOSE_LOG", config.wifi_verbose_log);

        cJSON_AddStringToObject(root, "motor_type", (config.motor_type == SERVO) ? "servo" : "step");
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
        cJSON_AddNumberToObject(root, "door_in",   config.door_in);
        cJSON_AddNumberToObject(root, "door_out",  config.door_out);

        cJSON *cats_array = cJSON_CreateArray();
        if (!cats_array) {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON alloc failed");
            return ESP_FAIL;
        }
        for (int i = 0; i < config.num_cats; i++) {
            cJSON *cat = cJSON_CreateObject();
            if (!cat) continue;
            // lasciamo device_code/country_code come numeri per compatibilità col tuo formato
            cJSON_AddNumberToObject(cat, "device_code", (double)config.authorized_cats[i].device_code);
            cJSON_AddNumberToObject(cat, "country_code", config.authorized_cats[i].country_code);
            cJSON_AddStringToObject(cat, "name", config.authorized_cats[i].name.c_str());
            cJSON_AddBoolToObject(cat, "authorized", config.authorized_cats[i].authorized);
            cJSON_AddItemToArray(cats_array, cat);
        }
        cJSON_AddItemToObject(root, "authorized_cats", cats_array);

        char *json_str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (!json_str) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON print failed");
            return ESP_FAIL;
        }

        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        free(json_str);
        return ESP_OK;
    }

    if (req->method == HTTP_POST) {
        // Controllo dimensione (usa il limite del pool POST)
        if (req->content_len <= 0 || req->content_len > POST_BODY_SIZE) {
            httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Payload troppo grande");
            return ESP_FAIL;
        }

        // Buffer PSRAM per il body
        char* pbody = (char*) buf_get_post_body(5000);  // attende fino a 5 s
        if (!pbody) {
            return resp_503(req, "Risorsa temporaneamente occupata");
        }

        size_t off = 0;
        while (off < (size_t)req->content_len) {
            int r = httpd_req_recv(req, pbody + off, req->content_len - off);
            if (r <= 0) {
                if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
                buf_put_post_body(pbody);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Errore ricezione dati");
                return ESP_FAIL;
            }
            off += (size_t)r;
        }
        pbody[off] = '\0';

        cJSON *root = cJSON_Parse(pbody);
        buf_put_post_body(pbody);
        if (!root) {
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
        if (!response) {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON alloc failed");
            return ESP_FAIL;
        }

        // Gestione azioni - ora le funzioni sono visibili
        esp_err_t result = ESP_OK;
        if (strcmp(action->valuestring, "add") == 0 || strcmp(action->valuestring, "update") == 0) {
            result = handle_cat_add_update(root, response, action->valuestring);
        } else if (strcmp(action->valuestring, "delete") == 0) {
            result = handle_cat_delete(root, response);
        } else if (strcmp(action->valuestring, "update_params") == 0) {
            result = handle_params_update(root, response);
        } else if (strcmp(action->valuestring, "reset_defaults") == 0) {
            result = handle_reset_defaults(response);
        } else {
            cJSON_AddBoolToObject(response, "success", false);
            cJSON_AddStringToObject(response, "error", "Azione non valida");
        }

        cJSON_Delete(root);

        if (result != ESP_OK) {
            cJSON_Delete(response);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Errore elaborazione richiesta");
            return ESP_FAIL;
        }

        char *json_str = cJSON_PrintUnformatted(response);
        cJSON_Delete(response);

        if (!json_str) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON print failed");
            return ESP_FAIL;
        }

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        free(json_str);
        return ESP_OK;
    }

    httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
    return ESP_FAIL;
}

// =====================================
// /clear_log e /reset_system (POST)
// =====================================

static esp_err_t clear_log_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        clear_log();
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "success", true);
        char *json_str = cJSON_PrintUnformatted(response);
        cJSON_Delete(response);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
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
        char *json_str = cJSON_PrintUnformatted(response);
        cJSON_Delete(response);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        free(json_str);

        ESP_LOGI(TAG, "Riavvio del sistema...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return ESP_OK;
    }
    httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
    return ESP_FAIL;
}


// Aggiungi questo handler in http_server.cpp dopo gli altri handler

// =====================================
// /api/status (GET) - per polling
// =====================================

static esp_err_t status_handler(httpd_req_t *req) {
    if (req->method != HTTP_GET) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON alloc failed");
        return ESP_FAIL;
    }

    // Modalità porta corrente
    const char* mode_str;
    switch (config.door_mode) {
        case AUTO: mode_str = "AUTO"; break;
        case ALWAYS_OPEN: mode_str = "ALWAYS_OPEN"; break;
        case ALWAYS_CLOSED: mode_str = "ALWAYS_CLOSED"; break;
        default: mode_str = "AUTO"; break;
    }
    cJSON_AddStringToObject(root, "door_mode", mode_str);

    // Log buffer (stesso formato del WebSocket Arduino)
    cJSON *log_array = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "log", log_array);
    if (!log_array) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON array failed");
        return ESP_FAIL;
    }

    // Popola il log in ordine cronologico inverso (più recenti per primi)
    for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
        size_t idx = (log_count - 1 - i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
        
        // Salta entry vuote
        if (log_buffer[idx].timestamp == 0 && log_buffer[idx].event[0] == '\0') {
            continue;
        }

        cJSON *entry = cJSON_CreateObject();
        if (!entry) continue;

        // Converti timestamp in stringa formato Arduino originale
        time_t ts = log_buffer[idx].timestamp;
        struct tm *timeinfo = localtime(&ts);
        char timestamp_str[20];
        strftime(timestamp_str, sizeof(timestamp_str), "%d/%m/%Y %H:%M:%S", timeinfo);

        cJSON_AddStringToObject(entry, "timestamp", timestamp_str);
        cJSON_AddStringToObject(entry, "type", log_buffer[idx].event);
        
        // Nome del gatto (dedotto dal device_code se presente)
        const char* name = "Sconosciuto";
        if (log_buffer[idx].device_code != 0) {
            for (int j = 0; j < config.num_cats; j++) {
                if (config.authorized_cats[j].device_code == log_buffer[idx].device_code) {
                    name = config.authorized_cats[j].name.c_str();
                    break;
                }
            }
        }
        cJSON_AddStringToObject(entry, "name", name);
        
        cJSON_AddNumberToObject(entry, "country_code", log_buffer[idx].country_code);
        
        // Converti device_code in stringa per compatibilità JavaScript
        char device_code_str[32];
        snprintf(device_code_str, sizeof(device_code_str), "%llu", 
                 (unsigned long long)log_buffer[idx].device_code);
        cJSON_AddStringToObject(entry, "device_code", device_code_str);
        
        cJSON_AddBoolToObject(entry, "authorized", log_buffer[idx].authorized);

        cJSON_AddItemToArray(log_array, entry);

        // Limita a max 100 entry (dimensione buffer eventi)
        if (i >= 99) break;
    }

    // Serializza e invia
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_str) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cJSON print failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);

    return ESP_OK;
}



// =====================================
// Avvio/stop webserver
// =====================================

static esp_err_t start_webserver(void) {
    if (server) {
        ESP_LOGW(TAG, "Web server già avviato");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Avvio del web server...");

    httpd_config_t config_ws = HTTPD_DEFAULT_CONFIG();
    config_ws.core_id = 0;
    config_ws.lru_purge_enable = true;
    config_ws.uri_match_fn     = httpd_uri_match_wildcard;
    config_ws.max_uri_handlers = 24;
    config_ws.stack_size       = 8192;
    config_ws.recv_wait_timeout = 60;
    config_ws.send_wait_timeout = 60;

    httpd_handle_t local = nullptr;
    esp_err_t err = httpd_start(&local, &config_ws);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() fallita: %s", esp_err_to_name(err));
        return err;
    }
    server = local;

    // Registrazione URI handlers
    httpd_uri_t handlers[] = {
        {"/config_data", HTTP_GET, config_data_handler, NULL, false, NULL, NULL},
        {"/config_data", HTTP_POST, config_data_handler, NULL, false, NULL, NULL},
        {"/clear_log", HTTP_POST, clear_log_handler, NULL, false, NULL, NULL},
        {"/reset_system", HTTP_POST, reset_system_handler, NULL, false, NULL, NULL},
        {"/ws", HTTP_GET, ws_handler, NULL, true, true, NULL},
        {"/", HTTP_GET, static_file_handler, NULL, false, NULL, NULL},
        {"/config", HTTP_GET, static_file_handler, NULL, false, NULL, NULL},
        {"/console", HTTP_GET, static_file_handler, NULL, false, NULL, NULL},
        {"/update", HTTP_GET, static_file_handler, NULL, false, NULL, NULL},
        {"/api/status", HTTP_GET, status_handler, NULL, false, NULL, NULL},
    };

    for (size_t i = 0; i < sizeof(handlers)/sizeof(handlers[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &handlers[i]));
        ESP_LOGI(TAG, "Handler registrato: %s %s", 
            (handlers[i].method == HTTP_GET) ? "GET" : "POST", handlers[i].uri);
    }

    // Registra handlers OTA
    register_ota_handlers(server);

    // Catch-all per file statici (deve essere ultimo)
    httpd_uri_t generic_get = {"/*", HTTP_GET, static_file_handler, NULL, false, NULL, NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &generic_get));
    ESP_LOGI(TAG, "Handler registrato: GET /*");

    ESP_LOGI(TAG, "Web server completamente configurato");
    return ESP_OK;
}

static void start_webserver_task(void *pvParameters) {
    (void)pvParameters;
    start_webserver();
    vTaskDelete(NULL);
}

void start_webserver_if_not_running(void) {
    if (!server) {
        xTaskCreatePinnedToCore(start_webserver_task, "WebServerTask",
                    4096, nullptr, 5, nullptr, 0);
    }
}

void stop_webserver(void) {
    if (server) {
        ESP_LOGI(TAG, "Arresto del web server...");
        httpd_stop(server);
        server = NULL;
    }
}