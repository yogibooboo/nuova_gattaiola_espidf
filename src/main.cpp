#include "comune.h"
#include "wifi.h"
#include "door.h"
#include "credentials.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include <esp_spiffs.h>

static const char *TAG = "MAIN_APP";

void print_task(void *pvParameters);

extern "C" void app_main() {
    ESP_LOGI(TAG, "Avvio del sistema Pet Door");

    // Inizializzazione SPIFFS
    ESP_LOGI(TAG, "Inizializzazione SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Partizione SPIFFS 'storage' non trovata");
        } else {
            ESP_LOGE(TAG, "Errore inizializzazione SPIFFS: %s (0x%x)", esp_err_to_name(ret), ret);
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("storage", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS: Totale: %d bytes, Usato: %d bytes", total, used);
    } else {
        ESP_LOGE(TAG, "Errore lettura informazioni SPIFFS: %s (0x%x)", esp_err_to_name(ret), ret);
    }

        // Carica la configurazione da /spiffs/config.json
    ESP_ERROR_CHECK(load_config());

    // Disabilita il watchdog per evitare conflitti (come nel Passo 4)
    ESP_ERROR_CHECK(esp_task_wdt_deinit());

    // Inizializzazione pin LED Wi-Fi
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << WIFI_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(WIFI_LED, LED_OFF); // LED spento all'avvio

    /*doorModeSemaphore = xSemaphoreCreateMutex();
    if (doorModeSemaphore == NULL) {
        ESP_LOGE(TAG, "Impossibile creare il semaforo per door_mode");
        return;
    }*/

    setup_wifi();  // Wi-Fi init e connessione

    xTaskCreate(print_task, "Print_Task", 4096, NULL, 1, NULL);
}

void print_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000);

    while (true) {
        ESP_LOGI(TAG, "RSSI Wi-Fi corrente: %d dBm, WiFi connesso: %d, Memoria libera: %u bytes",
                 wifi_rssi, wifi_connected, esp_get_free_heap_size());
        vTaskDelayUntil(&last_wake_time, interval);
    }
}