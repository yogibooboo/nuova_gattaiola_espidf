#include "comune.h"
#include "core1.h"
#include "wifi.h"
#include "door.h"
#include "credentials.h"
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "MAIN_APP";

void print_task(void *pvParameters);
void motor_calibration_task(void *pvParameters);

// Inizializzazione delle variabili globali
volatile DoorMode door_mode = AUTO;

volatile uint32_t door_sync_count = 0;
volatile uint32_t display_sync_count = 0;

portMUX_TYPE doorModeMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t doorModeSemaphore;

LogEntry log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_index = 0;

extern "C" void app_main() {
    ESP_LOGI(TAG, "Avvio del sistema Pet Door");

    // Inizializzazione pin LED Wi-Fi
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << WIFI_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(WIFI_LED, 1); // LED spento all'avvio (HIGH)

    doorModeSemaphore = xSemaphoreCreateMutex();
    if (doorModeSemaphore == NULL) {
        ESP_LOGE(TAG, "Impossibile creare il semaforo per door_mode");
        return;
    }

    setup_wifi();  // Wi-Fi init e connessione

    xTaskCreate(print_task, "Print_Task", 4096, NULL, 1, NULL);
}

void print_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000);

    while (true) {
        // La logica di stampa e invio dati rimarrà in questa funzione
        vTaskDelayUntil(&last_wake_time, interval);
    }
}