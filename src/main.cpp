#include "comune.h"
#include "wifi.h"
#include "door.h"
#include "credentials.h"
#include "console.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>

static const char *TAG = "MAIN_APP";

void print_task(void *pvParameters);

extern "C" void app_main() {
    ESP_LOGI(TAG, "Avvio del sistema Pet Door");

    // --- SPIFFS ---
    ESP_LOGI(TAG, "Inizializzazione SPIFFS...");
    ESP_ERROR_CHECK(spiffs_mount_boot());

    size_t total = 0, used = 0;
    if (spiffs_get_info(&total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS: Totale: %u bytes, Usato: %u bytes",
                 (unsigned)total, (unsigned)used);
    } else {
        ESP_LOGW(TAG, "Impossibile leggere info SPIFFS");
    }

    // Carica la configurazione da /spiffs/config.json
    ESP_ERROR_CHECK(load_config());

    // --- LED Wi-Fi ---
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << WIFI_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(WIFI_LED, LED_OFF); // LED spento all'avvio

    // --- Wi-Fi + Console ---
    setup_wifi();      // inizializza e connette
    console_start();   // avvia CLI (seriale + TCP)

    // --- Task periodica di diagnostica ---
    xTaskCreate(print_task, "Print_Task", 4096, NULL, 1, NULL);
}

void print_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000);

    while (true) {
        ESP_LOGI(TAG, "RSSI Wi-Fi zorrente: %d dBm, WiFi connesso: %d, Memoria libera: %u bytes",
                 wifi_rssi, wifi_connected, esp_get_free_heap_size());
        vTaskDelayUntil(&last_wake_time, interval);
    }
}
