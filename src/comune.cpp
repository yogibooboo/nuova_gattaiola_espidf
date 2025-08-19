#include "comune.h"
#include <string.h>
#include <cJSON.h>
#include <esp_spiffs.h>
#include <driver/gpio.h> // Aggiunto per sicurezza
#include <esp_log.h>

static const char* TAGFS = "SPIFFS";
#define SPIFFS_MAX_FILES 16

static esp_err_t spiffs_mount_internal(bool format_if_mount_failed) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = SPIFFS_MAX_FILES,
        .format_if_mount_failed = format_if_mount_failed
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAGFS, "Partizione SPIFFS 'storage' non trovata");
        } else {
            ESP_LOGE(TAGFS, "Mount SPIFFS fallito: %s (0x%x)", esp_err_to_name(ret), ret);
        }
        return ret;
    }
    return ESP_OK;
}

esp_err_t spiffs_mount_boot(void) {
    return spiffs_mount_internal(true);   // ok formattare al primo avvio
}

esp_err_t spiffs_mount_strict(void) {
    return spiffs_mount_internal(false);  // NO format (usalo dopo scrittura immagine)
}

void spiffs_unmount(void) {
    esp_vfs_spiffs_unregister("storage");
}

esp_err_t spiffs_get_info(size_t* total, size_t* used) {
    return esp_spiffs_info("storage", total, used);
}


static const char *TAG = "COMUNE";

config_t config;
LogEntry log_buffer[LOG_BUFFER_SIZE] = {0};
volatile bool wifi_connected = false; // Aggiunto volatile
int8_t wifi_rssi = 0;
uint32_t log_count = 0;

esp_err_t init_spiffs(void) {
    ESP_LOGI(TAG, "Inizializzazione SPIFFS...");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = SPIFFS_MAX_FILES,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Errore: Impossibile montare o formattare il filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Errore: Partizione SPIFFS non trovata");
        } else {
            ESP_LOGE(TAG, "Errore SPIFFS: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Errore ottenimento informazioni SPIFFS: %s", esp_err_to_name(ret));
        esp_vfs_spiffs_unregister(NULL);
        return ret;
    }
    ESP_LOGI(TAG, "SPIFFS: Totale: %d bytes, Usato: %d bytes", total, used);
    return ESP_OK;
}

esp_err_t save_config(void) {
    FILE *f = fopen("/spiffs/config.json", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Errore apertura /spiffs/config.json per scrittura");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "door_mode", config.door_mode == AUTO ? "AUTO" : config.door_mode == ALWAYS_OPEN ? "ALWAYS_OPEN" : "ALWAYS_CLOSED");
    cJSON_AddNumberToObject(root, "door_timeout", config.door_timeout);
    cJSON_AddNumberToObject(root, "steps_per_movement", config.steps_per_movement);
    cJSON_AddNumberToObject(root, "step_interval_us", config.step_interval_us);
    cJSON_AddNumberToObject(root, "wifi_reconnect_delay", config.wifi_reconnect_delay);
    cJSON_AddNumberToObject(root, "unauthorized_log_interval", config.unauthorized_log_interval);
    cJSON_AddBoolToObject(root, "wifi_verbose_log", config.wifi_verbose_log);
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

    if (fputs(json_str, f) == EOF) {
        ESP_LOGE(TAG, "Errore scrittura /spiffs/config.json");
        free(json_str);
        fclose(f);
        return ESP_FAIL;
    }

    free(json_str);
    fclose(f);
    ESP_LOGI(TAG, "Configurazione salvata su /spiffs/config.json");
    return ESP_OK;
}

esp_err_t load_config(void) {
    FILE *f = fopen("/spiffs/config.json", "r");
    if (f == NULL) {
        ESP_LOGW(TAG, "File /spiffs/config.json non trovato, utilizzo configurazione di default");
        // Inizializzazione esplicita di config
        config = {
            .door_mode = AUTO,
            .door_timeout = 10000,
            .steps_per_movement = 2500,
            .step_interval_us = 500,
            .wifi_reconnect_delay = 1000,
            .unauthorized_log_interval = 60000,
            .wifi_verbose_log = true,
            .motor_type = SERVO,
            .servo_open_us = 1800,
            .servo_closed_us = 1200,
            .servo_transition_ms = 1000,
            .config_01 = 0,
            .config_02 = 113,
            .config_03 = 682,
            .config_04 = 200,
            .config_05 = 0,
            .config_06 = 0,
            .config_07 = 0,
            .config_08 = 0,
            .config_09 = 0,
            .config_10 = 0,
            .door_rest = 2051,
            .door_in = 971,
            .door_out = 3072,
            .authorized_cats = {},
            .num_cats = 0
        };
        return save_config();
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *buffer = (char *)malloc(fsize + 1);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Errore allocazione memoria per config.json");
        fclose(f);
        return ESP_FAIL;
    }

    size_t read_size = fread(buffer, 1, fsize, f);
    fclose(f);
    buffer[read_size] = '\0';

    cJSON *root = cJSON_Parse(buffer);
    free(buffer);
    if (root == NULL) {
        ESP_LOGE(TAG, "Errore parsing config.json");
        return ESP_FAIL;
    }

    cJSON *door_mode = cJSON_GetObjectItem(root, "door_mode");
    if (door_mode && cJSON_IsString(door_mode)) {
        if (strcmp(door_mode->valuestring, "AUTO") == 0) config.door_mode = AUTO;
        else if (strcmp(door_mode->valuestring, "ALWAYS_OPEN") == 0) config.door_mode = ALWAYS_OPEN;
        else if (strcmp(door_mode->valuestring, "ALWAYS_CLOSED") == 0) config.door_mode = ALWAYS_CLOSED;
    }

    cJSON *door_timeout = cJSON_GetObjectItem(root, "door_timeout");
    if (door_timeout && cJSON_IsNumber(door_timeout)) config.door_timeout = door_timeout->valueint;

    cJSON *steps_per_movement = cJSON_GetObjectItem(root, "steps_per_movement");
    if (steps_per_movement && cJSON_IsNumber(steps_per_movement)) config.steps_per_movement = steps_per_movement->valueint;

    cJSON *step_interval_us = cJSON_GetObjectItem(root, "step_interval_us");
    if (step_interval_us && cJSON_IsNumber(step_interval_us)) config.step_interval_us = step_interval_us->valueint;

    cJSON *wifi_reconnect_delay = cJSON_GetObjectItem(root, "wifi_reconnect_delay");
    if (wifi_reconnect_delay && cJSON_IsNumber(wifi_reconnect_delay)) config.wifi_reconnect_delay = wifi_reconnect_delay->valueint;

    cJSON *unauthorized_log_interval = cJSON_GetObjectItem(root, "unauthorized_log_interval");
    if (unauthorized_log_interval && cJSON_IsNumber(unauthorized_log_interval)) config.unauthorized_log_interval = unauthorized_log_interval->valueint;

    cJSON *wifi_verbose_log = cJSON_GetObjectItem(root, "wifi_verbose_log");
    if (wifi_verbose_log && cJSON_IsBool(wifi_verbose_log)) config.wifi_verbose_log = wifi_verbose_log->valueint;

    cJSON *motor_type = cJSON_GetObjectItem(root, "motor_type");
    if (motor_type && cJSON_IsString(motor_type)) {
        config.motor_type = (strcmp(motor_type->valuestring, "servo") == 0) ? SERVO : STEP;
    }

    cJSON *servo_open_us = cJSON_GetObjectItem(root, "servo_open_us");
    if (servo_open_us && cJSON_IsNumber(servo_open_us)) config.servo_open_us = servo_open_us->valueint;

    cJSON *servo_closed_us = cJSON_GetObjectItem(root, "servo_closed_us");
    if (servo_closed_us && cJSON_IsNumber(servo_closed_us)) config.servo_closed_us = servo_closed_us->valueint;

    cJSON *servo_transition_ms = cJSON_GetObjectItem(root, "servo_transition_ms");
    if (servo_transition_ms && cJSON_IsNumber(servo_transition_ms)) config.servo_transition_ms = servo_transition_ms->valueint;

    cJSON *config_01 = cJSON_GetObjectItem(root, "config_01");
    if (config_01 && cJSON_IsNumber(config_01)) config.config_01 = config_01->valueint;

    cJSON *config_02 = cJSON_GetObjectItem(root, "config_02");
    if (config_02 && cJSON_IsNumber(config_02)) config.config_02 = config_02->valueint;

    cJSON *config_03 = cJSON_GetObjectItem(root, "config_03");
    if (config_03 && cJSON_IsNumber(config_03)) config.config_03 = config_03->valueint;

    cJSON *config_04 = cJSON_GetObjectItem(root, "config_04");
    if (config_04 && cJSON_IsNumber(config_04)) config.config_04 = config_04->valueint;

    cJSON *config_05 = cJSON_GetObjectItem(root, "config_05");
    if (config_05 && cJSON_IsNumber(config_05)) config.config_05 = config_05->valueint;

    cJSON *config_06 = cJSON_GetObjectItem(root, "config_06");
    if (config_06 && cJSON_IsNumber(config_06)) config.config_06 = config_06->valueint;

    cJSON *config_07 = cJSON_GetObjectItem(root, "config_07");
    if (config_07 && cJSON_IsNumber(config_07)) config.config_07 = config_07->valueint;

    cJSON *config_08 = cJSON_GetObjectItem(root, "config_08");
    if (config_08 && cJSON_IsNumber(config_08)) config.config_08 = config_08->valueint;

    cJSON *config_09 = cJSON_GetObjectItem(root, "config_09");
    if (config_09 && cJSON_IsNumber(config_09)) config.config_09 = config_09->valueint;

    cJSON *config_10 = cJSON_GetObjectItem(root, "config_10");
    if (config_10 && cJSON_IsNumber(config_10)) config.config_10 = config_10->valueint;

    cJSON *door_rest = cJSON_GetObjectItem(root, "door_rest");
    if (door_rest && cJSON_IsNumber(door_rest)) config.door_rest = door_rest->valueint;

    cJSON *door_in = cJSON_GetObjectItem(root, "door_in");
    if (door_in && cJSON_IsNumber(door_in)) config.door_in = door_in->valueint;

    cJSON *door_out = cJSON_GetObjectItem(root, "door_out");
    if (door_out && cJSON_IsNumber(door_out)) config.door_out = door_out->valueint;

    cJSON *cats = cJSON_GetObjectItem(root, "authorized_cats");
    if (cats && cJSON_IsArray(cats)) {
        config.num_cats = cJSON_GetArraySize(cats);
        if (config.num_cats > MAX_CATS) config.num_cats = MAX_CATS;
        for (int i = 0; i < config.num_cats; i++) {
            cJSON *cat = cJSON_GetArrayItem(cats, i);
            cJSON *device_code = cJSON_GetObjectItem(cat, "device_code");
            if (device_code && cJSON_IsNumber(device_code)) config.authorized_cats[i].device_code = (uint64_t)device_code->valuedouble;

            cJSON *country_code = cJSON_GetObjectItem(cat, "country_code");
            if (country_code && cJSON_IsNumber(country_code)) config.authorized_cats[i].country_code = country_code->valueint;

            cJSON *name = cJSON_GetObjectItem(cat, "name");
            if (name && cJSON_IsString(name)) config.authorized_cats[i].name = name->valuestring;

            cJSON *authorized = cJSON_GetObjectItem(cat, "authorized");
            if (authorized && cJSON_IsBool(authorized)) config.authorized_cats[i].authorized = authorized->valueint;
        }
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Configurazione caricata con successo: %d gatti", config.num_cats);
    return ESP_OK;
}

void clear_log(void) {
    memset(log_buffer, 0, sizeof(log_buffer));
    log_count = 0;
    ESP_LOGI(TAG, "Log cancellato");
}