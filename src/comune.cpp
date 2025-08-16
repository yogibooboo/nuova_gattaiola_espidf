#include "comune.h"

// Definizione reale delle variabili globali
int8_t wifi_rssi = 0;
volatile bool wifi_connected = false;
volatile DoorMode door_mode = AUTO;
volatile uint32_t door_sync_count = 0;
volatile uint32_t display_sync_count = 0;
portMUX_TYPE doorModeMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t doorModeSemaphore;
LogEntry log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_index = 0;

void save_config() {
    // Implementazione della funzione save_config (se necessaria)
    // Placeholder per salvare log_buffer su NVS
}