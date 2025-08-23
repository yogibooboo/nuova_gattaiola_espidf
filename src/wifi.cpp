#include "wifi.h"
#include "comune.h"
#include "credentials.h"

#include <string.h>
#include <errno.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include <nvs_flash.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include "telnet_server.h"

#include "buffers_psram.h"   // <-- pool di buffer in PSRAM

#include <esp_ota_ops.h>    // esp_ota_begin/write/end, set_boot_partition, get_next_update_partition
#include <esp_partition.h>  // esp_partition_find_first/erase_range/write per SPIFFS
#include "time_sync.h"

static const char *TAG = "WIFI";
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

    // Mappa "/" a "/index.html" e "/config" a "/config.html"
    if (strcmp(uri, "/") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/index.html");
    } else if (strcmp(uri, "/config") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/config.html");
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
        return ESP_FAIL;
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
            return ESP_FAIL;
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
            const cJSON *jname = cJSON_GetObjectItem(cat, "name");
            const cJSON *jauth = cJSON_GetObjectItem(cat, "authorized");
            const char *name = jname ? jname->valuestring : "";
            bool authorized = jauth ? (jauth->valueint != 0) : true;

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

        } else {
            cJSON_AddBoolToObject(response, "success", false);
            cJSON_AddStringToObject(response, "error", "Azione non valida");
        }

        cJSON_Delete(root);

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

// ota_firmware_handler
static esp_err_t ota_firmware_handler(httpd_req_t* req) {
    if (req->method != HTTP_POST) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
        return ESP_FAIL;
    }

    // Partizione di destinazione
    const esp_partition_t* update_part = esp_ota_get_next_update_partition(nullptr);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Nessuna partizione OTA");
        return ESP_FAIL;
    }

    // (opzionale) controllo Content-Length
    if (req->content_len <= 0 || req->content_len > (int)update_part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Dimensione non valida");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin fallita");
        return ESP_FAIL;
    }

    // buffer in PSRAM se disponibile
    const size_t CHUNK = 16 * 1024;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = (uint8_t*)malloc(CHUNK);
    if (!buf) {
        esp_ota_end(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "alloc fallita");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = remaining > (int)CHUNK ? (int)CHUNK : remaining;
        int r = httpd_req_recv(req, (char*)buf, to_read);
        if (r <= 0) {
            free(buf);
            esp_ota_end(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ricezione fallita");
            return ESP_FAIL;
        }
        err = esp_ota_write(ota, buf, r);
        if (err != ESP_OK) {
            free(buf);
            esp_ota_end(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scrittura OTA fallita");
            return ESP_FAIL;
        }
        remaining -= r;
        // lascia respirare il WDT cooperativo
        vTaskDelay(1);
    }
    free(buf);

    if ((err = esp_ota_end(ota)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_end fallita");
        return ESP_FAIL;
    }
    if ((err = esp_ota_set_boot_partition(update_part)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot fallita");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"OTA firmware OK. Riavvio...\"}");

    // breve delay per permettere al client di ricevere la risposta
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}


// ota_spiffs_handler

static esp_err_t ota_spiffs_handler(httpd_req_t* req) {
    if (req->method != HTTP_POST) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
        return ESP_FAIL;
    }

    const esp_partition_t* part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if (!part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Partizione SPIFFS non trovata");
        return ESP_FAIL;
    }

    if (req->content_len <= 0 || req->content_len > (int)part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Dimensione immagine non valida");
        return ESP_FAIL;
    }

    // 1) Smonta il FS (niente accessi concorrenti)
    spiffs_unmount();

    // 2) Preparazione buffer
    const size_t CHUNK = 16 * 1024;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = (uint8_t*)malloc(CHUNK);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "alloc fallita");
        return ESP_FAIL;
    }

    // 3) Cancella man mano (erase step) e scrivi i chunk ricevuti
    size_t offset = 0;
    int remaining = req->content_len;

    // tener traccia di quanto abbiamo già cancellato (allineato a 4K)
    size_t erased_until = 0;
    const size_t ERASE_STEP = 64 * 1024; // cancella in step da 64KiB per ridurre pause lunghe

    while (remaining > 0) {
        int to_read = remaining > (int)CHUNK ? (int)CHUNK : remaining;
        int r = httpd_req_recv(req, (char*)buf, to_read);
        if (r <= 0) {
            free(buf);
            // in caso di errore, prova un mount rigoroso per lasciare il sistema vivo
            spiffs_mount_strict();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ricezione fallita");
            return ESP_FAIL;
        }

        // Assicura che l'area da scrivere sia cancellata
        size_t need_end = offset + r;
        // arrotonda l'area da cancellare a multipli di 4KiB
        if (need_end > erased_until) {
            size_t erase_to = need_end;
            // cancella a step per non bloccare troppo
            while (erased_until < erase_to && erased_until < part->size) {
                size_t step = ERASE_STEP;
                if (erased_until + step > part->size) step = part->size - erased_until;
                // allinea lo start a 4KiB
                size_t aligned_start = erased_until & ~((size_t)0xFFF);
                // allinea la lunghezza a multipli di 4KiB
                size_t aligned_len = (step + (erased_until - aligned_start) + 0xFFF) & ~((size_t)0xFFF);
                if (aligned_start + aligned_len > part->size) {
                    aligned_len = part->size - aligned_start;
                }
                if (esp_partition_erase_range(part, aligned_start, aligned_len) != ESP_OK) {
                    free(buf);
                    spiffs_mount_strict();
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase fallito");
                    return ESP_FAIL;
                }
                erased_until = aligned_start + aligned_len;
                vTaskDelay(1); // lascia respirare il WDT/stack TCP
            }
        }

        // Scrivi il pezzo
        if (esp_partition_write(part, offset, buf, r) != ESP_OK) {
            free(buf);
            spiffs_mount_strict();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scrittura SPIFFS fallita");
            return ESP_FAIL;
        }

        offset += r;
        remaining -= r;

        // feed cooperativo
        vTaskDelay(1);
    }

    // 4) Se l'immagine è più piccola della partizione, cancella la "coda" residua
    //    per evitare sporcizia che potrebbe confondere SPIFFS
    if (offset < part->size) {
        size_t tail_start = (offset + 0xFFF) & ~((size_t)0xFFF); // allinea a 4KiB
        if (tail_start < part->size) {
            size_t tail_len = part->size - tail_start;
            // Erase a step per non bloccare per troppo
            size_t done = 0;
            while (done < tail_len) {
                size_t step = ERASE_STEP;
                if (done + step > tail_len) step = tail_len - done;
                if (esp_partition_erase_range(part, tail_start + done, step) != ESP_OK) {
                    // non è critico per la leggibilità dell'immagine, ma meglio saperlo
                    // rimonta e rispondi errore
                    spiffs_mount_strict();
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase coda fallito");
                    return ESP_FAIL;
                }
                done += step;
                vTaskDelay(1);
            }
        }
    }

    free(buf);

    // 5) Rispondi e riavvia
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"SPIFFS aggiornato. Riavvio...\"}");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}



// =====================================
// /clear_log (POST)
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

// =====================================
// /reset_system (POST)
// =====================================

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

// =====================================
// /ws (WebSocket)
// =====================================

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake WebSocket completato");
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
        return ret;
    }

    if (ws_pkt.len >= WS_FRAME_SIZE) { // >= per lasciare spazio a '\0'
        ESP_LOGW(TAG, "WS frame troppo grande: %u >= %u", (unsigned)ws_pkt.len, (unsigned)WS_FRAME_SIZE);
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Frame troppo grande");
        return ESP_FAIL;
    }

    uint8_t* wbuf = (uint8_t*) buf_get_ws_frame(200);
    if (!wbuf) {
        return resp_503(req, "Buffer WS non disponibile");
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

    // Risposta di cortesia
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
// Avvio/stop webserver
// =====================================

static esp_err_t start_webserver(void) {
    if (server) {
        ESP_LOGW(TAG, "Web server già avviato");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Avvio del web server...");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;
    config.lru_purge_enable = true;
    config.uri_match_fn     = httpd_uri_match_wildcard;
    config.max_uri_handlers = 24;
    config.stack_size       = 8192;   // più respiro per gli handler

    config.recv_wait_timeout = 60;  // attesa massima per ricevere il body (es. upload)
    config.send_wait_timeout = 60;  // attesa massima per inviare la risposta

    httpd_handle_t local = nullptr;
    esp_err_t err = httpd_start(&local, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() fallita: %s", esp_err_to_name(err));
        return err;
    }
    server = local;

    auto method_str = [](httpd_method_t m) {
        switch (m) {
            case HTTP_GET:  return "GET";
            case HTTP_POST: return "POST";
            case HTTP_PUT:  return "PUT";
            case HTTP_DELETE:return "DELETE";
            default:        return "?";
        }
    };
    auto REG = [&](const httpd_uri_t& u) {
        esp_err_t r = httpd_register_uri_handler(server, &u);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "Handler registrato: %-4s %s", method_str(u.method), u.uri);
        } else {
            ESP_LOGE(TAG, "REG FALLITA: %-4s %s -> %s", method_str(u.method), u.uri, esp_err_to_name(r));
        }
        return r;
    };

    // ===== 1) ENDPOINT SPECIFICI (API) =====
    httpd_uri_t config_data_get = {
        .uri = "/config_data",
        .method = HTTP_GET,
        .handler = config_data_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    REG(config_data_get);

    httpd_uri_t config_data_post = config_data_get;
    config_data_post.method = HTTP_POST;
    REG(config_data_post);

    httpd_uri_t clear_log_post = {
        .uri = "/clear_log",
        .method = HTTP_POST,
        .handler = clear_log_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    REG(clear_log_post);

    httpd_uri_t reset_system_post = {
        .uri = "/reset_system",
        .method = HTTP_POST,
        .handler = reset_system_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    REG(reset_system_post);

    // ===== 2) WEBSOCKET =====
    httpd_uri_t ws_get = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true,
        .supported_subprotocol = NULL
    };
    REG(ws_get);

    // ===== 3) STATICI PUNTUALI =====
    httpd_uri_t root_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    REG(root_get);

    httpd_uri_t config_get = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    REG(config_get);



        // POST /ota_firmware
    httpd_uri_t ota_fw_post = {
        .uri = "/ota_firmware",
        .method = HTTP_POST,
        .handler = ota_firmware_handler,
        .user_ctx = NULL
    };
    REG(ota_fw_post);

    // POST /ota_spiffs
    httpd_uri_t ota_fs_post = {
        .uri = "/ota_spiffs",
        .method = HTTP_POST,
        .handler = ota_spiffs_handler,
        .user_ctx = NULL
    };
    REG(ota_fs_post);

    // GET /update -> serve la pagina
    httpd_uri_t update_get = {
        .uri = "/update",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = NULL
    };
    REG(update_get);

    // ===== 4) CATCH-ALL STATICO (solo GET) — ALLA FINE =====
    httpd_uri_t generic_get = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    REG(generic_get);

    ESP_LOGI(TAG, "Web server completamente configurato");
    return ESP_OK;
}

static void start_webserver_task(void *pvParameters) {
    (void)pvParameters;
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

// =====================================
// Wi-Fi events & init
// =====================================

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
        if (!server) {
            xTaskCreatePinnedToCore(start_webserver_task, "WebServerTask",
                        4096, nullptr, 5, nullptr, 0);
        }
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
