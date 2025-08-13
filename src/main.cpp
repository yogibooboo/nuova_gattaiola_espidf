#include "common.h"
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
volatile bool wifi_connected = false;
volatile uint32_t door_sync_count = 0;
volatile uint32_t display_sync_count = 0;

portMUX_TYPE doorModeMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t doorModeSemaphore;

LogEntry log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_index = 0;

extern "C" void app_main() {
    //Serial.begin(115200);
    ESP_LOGI(TAG, "Avvio del sistema Pet Door");

    doorModeSemaphore = xSemaphoreCreateMutex();
    if (doorModeSemaphore == NULL) {
        ESP_LOGE(TAG, "Impossibile creare il semaforo per door_mode");
        return;
    }

    setup_wifi();
    // setup_door();
    // setup_rfid();
    // start_rfid_task();

    xTaskCreate(print_task, "Print_Task", 4096, NULL, 1, NULL);

    // I seguenti task sono commentati per una compilazione minima
    // xTaskCreatePinnedToCore(core1_task, "Core1_Task", 4096, NULL, 1, NULL, 1);
}

void print_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000);

    while (true) {
        // La logica di stampa e invio dati rimarrà in questa funzione
        vTaskDelayUntil(&last_wake_time, interval);
    }
}