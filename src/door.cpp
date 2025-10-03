#include "comune.h"
#include "core1.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <string.h>
#include <ctime>
#include <driver/i2c.h>  // Per le funzioni I2C

static const char *TAG = "DOOR";

// =====================================
// Variabili globali di stato porta
// =====================================
EXT_RAM_BSS_ATTR EncoderData encoder_buffer[ENCODER_BUFFER_SIZE];
volatile size_t encoder_buffer_index = 0;

// BUFFER ESTESO - Allocazione dinamica in PSRAM
EncoderDataExtended* encoder_buffer_extended = nullptr;
volatile size_t encoder_buffer_extended_index = 0;
volatile uint32_t encoder_buffer_extended_wraps = 0;
volatile uint64_t encoder_buffer_extended_total_samples = 0;

static volatile bool door_open = false;
static TickType_t door_timer_start = 0;
static bool servo_inited = false;
static volatile bool interruptFlag = false;

// NUOVO: Variabili per gestione chiusura temporanea in ALWAYS_OPEN
static bool temp_closed_active = false; // Stato TEMP_CLOSED per gatti non autorizzati
static TickType_t temp_closed_start = 0; // Timestamp inizio chiusura temporanea

// Variabili per tracking ultimo gatto autorizzato
static volatile uint64_t last_authorized_device_code = 0;
static volatile uint16_t last_authorized_country_code = 0;

// Variabili diagnostiche esposte (spostate dalla funzione door_task)
static enum { IDLE, EVENTO_ATTIVO } stato_evento = IDLE;
static char timestamp_inizio[20] = "";
static time_t timestamp_inizio_time = 0;
static uint32_t indice_inizio = 0;
static bool passaggio_confermato = false;
static const char* direzione = NULL;
static uint32_t conteggio_senza_trigger = 0;
static uint32_t ultimo_trigger_idx = 0;
static int32_t trigger_porta_idx = -1;
static int32_t passaggio_porta_idx = -1;
static int32_t detect_idx = -1;
static int32_t infrared_idx = -1;
static char cat_name_evento[33] = "Sconosciuto";
static uint16_t country_code_evento = 0;
static uint64_t device_code_evento = 0;
static bool authorized_evento = false;
static bool codice_associato = false;
static unsigned long last_unauthorized_log = 0;

// Aggiungi questa variabile globale dopo le altre variabili globali in door.cpp:
static bool encoder_i2c_initialized = false;
static bool vl6180x_i2c_initialized = false;

// Variabile globale per distanza VL6180X (esportata)
volatile uint8_t lastDistance = 255;  // Default 255 = errore/non inizializzato

// Variabile per inversione servo (letta all'avvio da GPIO21)
static bool servo_inverted = false;

// =====================================
// Inizializzazione Buffer Esteso PSRAM
// =====================================
static esp_err_t init_extended_buffer(void) {
    if (encoder_buffer_extended != nullptr) {
        ESP_LOGI(TAG, "Buffer esteso già inizializzato");
        return ESP_OK;
    }
    
    size_t buffer_size = ENCODER_BUFFER_EXTENDED_SIZE * sizeof(EncoderDataExtended);
    float buffer_mb = buffer_size / (1024.0f * 1024.0f);
    
    ESP_LOGI(TAG, "Tentativo allocazione buffer esteso: %.1f MB in PSRAM", buffer_mb);
    
    encoder_buffer_extended = (EncoderDataExtended*) heap_caps_malloc(
        buffer_size, 
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT  // Stesso pattern del tuo BUFPSRAM
    );
    
    if (encoder_buffer_extended == nullptr) {
        ESP_LOGE(TAG, "ERRORE: Impossibile allocare %.1f MB in PSRAM", buffer_mb);
        return ESP_ERR_NO_MEM;
    }

    // Inizializza il buffer a zero a blocchi per evitare watchdog
    const size_t chunk_size = 65536;  // 64KB per blocco
    uint8_t* ptr = (uint8_t*)encoder_buffer_extended;
    for (size_t offset = 0; offset < buffer_size; offset += chunk_size) {
        size_t bytes_to_clear = (offset + chunk_size > buffer_size) ? (buffer_size - offset) : chunk_size;
        memset(ptr + offset, 0, bytes_to_clear);
        vTaskDelay(1);  // Yield ogni 64KB per non bloccare watchdog
    }

    ESP_LOGI(TAG, "Buffer esteso: %.1f MB allocati in PSRAM", buffer_mb);
    ESP_LOGI(TAG, "Indirizzo buffer esteso: %p", encoder_buffer_extended);
    
    return ESP_OK;
}

// =====================================
// Encoder placeholder
// =====================================
// Lettura ottimizzata AS5600: angle + magnitude in burst (4 byte totali)
static void encoder_readAngleAndMagnitude(uint16_t* angle, uint16_t* magnitude) {
    uint8_t data[4];
    *angle = 2000;      // Valori di default in caso di errore
    *magnitude = 2100;

    // Leggi 4 byte consecutivi in burst: ANGLE_MSB(0x0E), ANGLE_LSB(0x0F), poi skippa fino a MAGNITUDE_MSB(0x1B), MAGNITUDE_LSB(0x1C)
    // Soluzione: leggi prima angle, poi magnitude separatamente ma in modo ottimizzato

    // Burst read angle (2 byte: MSB + LSB)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0E, true); // AS5600_ANGLE_MSB_REG
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK); // Leggi 2 byte in sequenza
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *angle = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
        *angle &= 0x0FFF;
    }

    // Burst read magnitude (2 byte: MSB + LSB)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1B, true); // AS5600_MAGNITUDE_MSB
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, &data[2], 2, I2C_MASTER_LAST_NACK); // Leggi 2 byte in sequenza
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *magnitude = ((uint16_t)(data[2] & 0x0F) << 8) | data[3];
        *magnitude &= 0x0FFF;
    }
}

// =====================================
// VL6180X Time-of-Flight Distance Sensor
// =====================================
#define VL6180X_ADDRESS 0x29

// Registri VL6180X principali
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET   0x0016
#define VL6180X_SYSRANGE_START              0x0018
#define VL6180X_RESULT_RANGE_STATUS         0x004D
#define VL6180X_RESULT_RANGE_VAL            0x0062

// Helper per scrivere registro VL6180X (16-bit address)
static esp_err_t vl6180x_write_reg(uint16_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);  // MSB
    i2c_master_write_byte(cmd, reg & 0xFF, true);          // LSB
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Helper per leggere registro VL6180X (16-bit address)
static esp_err_t vl6180x_read_reg(uint16_t reg, uint8_t* value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Inizializzazione completa VL6180X con configurazione raccomandata
static esp_err_t vl6180x_init(void) {
    uint8_t reset_status;

    // Leggi stato reset
    esp_err_t ret = vl6180x_read_reg(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, &reset_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL6180X: Errore lettura reset status");
        return ret;
    }

    ESP_LOGI(TAG, "VL6180X: Reset status iniziale = 0x%02X", reset_status);

    // Se NON è fresh out of reset, forza un soft reset
    if (reset_status != 0x01) {
        ESP_LOGW(TAG, "VL6180X: Sensore non fresh, applico soft reset...");

        // Soft reset: scrivi 0x01 al registro FRESH_OUT_OF_RESET
        vl6180x_write_reg(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));  // Attendi stabilizzazione

        // Rileggi per conferma
        ret = vl6180x_read_reg(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, &reset_status);
        ESP_LOGI(TAG, "VL6180X: Reset status dopo soft reset = 0x%02X", reset_status);
    }

    // Applica sempre la configurazione completa
    ESP_LOGI(TAG, "VL6180X: Applicazione configurazione raccomandata ST...");

    // Configurazione raccomandata da ST per ranging
    vl6180x_write_reg(0x0207, 0x01);
    vl6180x_write_reg(0x0208, 0x01);
    vl6180x_write_reg(0x0096, 0x00);
    vl6180x_write_reg(0x0097, 0xfd);
    vl6180x_write_reg(0x00e3, 0x00);
    vl6180x_write_reg(0x00e4, 0x04);
    vl6180x_write_reg(0x00e5, 0x02);
    vl6180x_write_reg(0x00e6, 0x01);
    vl6180x_write_reg(0x00e7, 0x03);
    vl6180x_write_reg(0x00f5, 0x02);
    vl6180x_write_reg(0x00d9, 0x05);
    vl6180x_write_reg(0x00db, 0xce);
    vl6180x_write_reg(0x00dc, 0x03);
    vl6180x_write_reg(0x00dd, 0xf8);
    vl6180x_write_reg(0x009f, 0x00);
    vl6180x_write_reg(0x00a3, 0x3c);
    vl6180x_write_reg(0x00b7, 0x00);
    vl6180x_write_reg(0x00bb, 0x3c);
    vl6180x_write_reg(0x00b2, 0x09);
    vl6180x_write_reg(0x00ca, 0x09);
    vl6180x_write_reg(0x0198, 0x01);
    vl6180x_write_reg(0x01b0, 0x17);
    vl6180x_write_reg(0x01ad, 0x00);
    vl6180x_write_reg(0x00ff, 0x05);
    vl6180x_write_reg(0x0100, 0x05);
    vl6180x_write_reg(0x0199, 0x05);
    vl6180x_write_reg(0x01a6, 0x1b);
    vl6180x_write_reg(0x01ac, 0x3e);
    vl6180x_write_reg(0x01a7, 0x1f);
    vl6180x_write_reg(0x0030, 0x00);

    // Configurazione range: convergence time e inter-measurement period
    vl6180x_write_reg(0x001b, 0x09);  // Convergence time (30ms)
    vl6180x_write_reg(0x003e, 0x31);  // Range check enables
    vl6180x_write_reg(0x0014, 0x24);  // Early convergence estimate

    // Azzera offset hardware (usiamo config.config_06 per calibrazione software)
    vl6180x_write_reg(0x0024, 0x00);  // SYSRANGE__PART_TO_PART_RANGE_OFFSET

    // Clear fresh out of reset
    vl6180x_write_reg(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);

    ESP_LOGI(TAG, "VL6180X: Configurazione completata");
    return ESP_OK;
}

// Lettura distanza VL6180X (non bloccante - ritorna ultimo valore disponibile)
static uint8_t vl6180x_readDistance(void) {
    static uint8_t last_valid_distance = 255;
    static bool measurement_started = false;
    uint8_t distance = 255;

    if (!measurement_started) {
        // Avvia misura single-shot
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, (VL6180X_SYSRANGE_START >> 8) & 0xFF, true);
        i2c_master_write_byte(cmd, VL6180X_SYSRANGE_START & 0xFF, true);
        i2c_master_write_byte(cmd, 0x01, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            measurement_started = true;
        }
        return last_valid_distance;
    }

    // Controlla se misura è pronta (non bloccante)
    uint8_t status;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (VL6180X_RESULT_RANGE_STATUS >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, VL6180X_RESULT_RANGE_STATUS & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        measurement_started = false;
        return last_valid_distance;
    }

    // Se la misura non è ancora pronta, ritorna ultimo valore valido
    if ((status & 0x01) == 0) {
        return last_valid_distance;
    }

    // Misura pronta - leggi distanza
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (VL6180X_RESULT_RANGE_VAL >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, VL6180X_RESULT_RANGE_VAL & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL6180X_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &distance, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // Applica offset calibrazione da config.config_06
        // Se config_06 >= 1000, sensore è disabilitato ma offset è (config_06 - 1000)
        uint32_t effective_offset = (config.config_06 >= 1000) ? (config.config_06 - 1000) : config.config_06;

        if (distance > effective_offset) {
            last_valid_distance = distance - effective_offset;
        } else {
            last_valid_distance = 0;  // Troppo vicino, clamp a 0
        }
    }

    measurement_started = false;  // Pronto per nuova misura
    return last_valid_distance;
}

// =====================================
// Interrupt handler per infrarossi
// =====================================
static void IRAM_ATTR handleInfraredInterrupt(void* arg) {
    interruptFlag = true;
}

// =====================================
// Servo control
// =====================================
static void servo_init_once(void) {
    if (servo_inited) return;

    // Timer 50 Hz (periodo 20 ms), risoluzione 14 bit
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ccfg = {
        .gpio_num       = SERVO_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,  // AGGIUNTO per warning
        .flags          = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    servo_inited = true;
}

static void setServoPosition(bool open) {
    static uint32_t current_servo_us = 0;
    if (current_servo_us == 0) current_servo_us = config.servo_closed_us;

    // Inverti logica se jumper GPIO21 è a LOW
    if (servo_inverted) {
        open = !open;
    }

    uint32_t target_us = open ? config.servo_open_us : config.servo_closed_us;
    int32_t delta_us = (int32_t)target_us - (int32_t)current_servo_us;
    if (delta_us == 0) return;

    uint32_t steps = config.servo_transition_ms / 10;
    if (steps == 0) steps = 1;

    for (uint32_t i = 0; i < steps; i++) {
        int32_t progress = (int32_t)(i + 1) * delta_us;
        uint32_t new_us = current_servo_us + (progress / (int32_t)steps);
        uint32_t duty = (new_us * (1ULL << LEDC_TIMER_14_BIT)) / 20000;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Imposta esattamente il valore finale
    uint32_t duty = (target_us * (1ULL << LEDC_TIMER_14_BIT)) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    current_servo_us = target_us;
}

// =====================================
// Log integration
// =====================================
static void add_log_entry(time_t timestamp, const char* type, const char* name, 
                         uint16_t country_code, uint64_t device_code, bool authorized){
    LogEntry* entry = &log_buffer[log_count % LOG_BUFFER_SIZE];
    
    entry->timestamp = timestamp;  // Usa il timestamp passato come parametro
    entry->device_code = device_code;
    entry->country_code = country_code;
    entry->authorized = authorized;
    strncpy(entry->event, type, sizeof(entry->event) - 1);
    entry->event[sizeof(entry->event) - 1] = '\0';
    
    log_count++;
    
    // Log con timestamp formattato per debug
    struct tm tm_log;
    localtime_r(&timestamp, &tm_log);
    char timestamp_str[20];
    strftime(timestamp_str, sizeof(timestamp_str), "%d/%m/%Y %H:%M:%S", &tm_log);
    
    ESP_LOGI(TAG, "Log aggiunto: [%s] Type: %s, Name: %s", timestamp_str, type, name);
}

// =====================================
// GPIO initialization
// =====================================
static void door_gpio_init_once(void) {
    static bool inited = false;
    if (inited) return;
    inited = true;

    gpio_config_t io = {};

    // LED rosso
    io.pin_bit_mask = 1ULL << LED_ROSSO;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(LED_ROSSO, LED_OFF);

    // LED blu
    io.pin_bit_mask = 1ULL << LED_BLU;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(LED_BLU, LED_OFF);

    // Sensore infrarosso (input con interrupt)
    io.pin_bit_mask = 1ULL << INFRARED;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(INFRARED, handleInfraredInterrupt, NULL));

    // Enable infrarosso (sempre alto)
    io.pin_bit_mask = 1ULL << INFRARED_ENABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(INFRARED_ENABLE, 1);

    // Detected LED
    io.pin_bit_mask = 1ULL << DETECTED;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(DETECTED, LED_OFF);

    // Pulsante blu
    io.pin_bit_mask = 1ULL << PULSANTE_BLU;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    // Jumper inversione servo (GPIO21 - safe GPIO)
    io.pin_bit_mask = 1ULL << SERVO_INVERT_PIN;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    // Leggi stato inversione servo (LOW = inverti)
    servo_inverted = !gpio_get_level(SERVO_INVERT_PIN);
    ESP_LOGI(TAG, "Servo inversione: %s (GPIO21=%d)",
             servo_inverted ? "ABILITATA" : "NORMALE",
             gpio_get_level(SERVO_INVERT_PIN));
}

// Funzione diagnostica per comando "door"
extern "C" void get_door_status(char* buffer, size_t buffer_size, const char* subcommand) {
    if (strlen(subcommand) == 0) {
        time_t now;
        time(&now);
        struct tm *tm_now = localtime(&now);
        char time_str[20];
        strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_now);
        
        const char* mode_str = (config.door_mode == AUTO) ? "AUTO" : 
                              (config.door_mode == ALWAYS_OPEN) ? "ALWAYS_OPEN" : "ALWAYS_CLOSED";
        
        // Conversione corrected angle in gradi: 0=-180°, 2048=0°, 4095=+180°
        float degrees = ((float)lastCorrectedAngle - 2048.0f) * 180.0f / 2048.0f;
        
        const char* stato_str = (stato_evento == IDLE) ? "IDLE" : "EVENTO_ATTIVO";
        
        // Calcolo timer
        TickType_t now_ticks = xTaskGetTickCount();
        uint32_t unauth_elapsed_sec = (now_ticks - last_unauthorized_log) / configTICK_RATE_HZ;
        uint32_t door_elapsed_sec = door_open ? (now_ticks - door_timer_start) / configTICK_RATE_HZ : 0;
        
        char temp_timer_str[32] = "N/A";
        if (temp_closed_active) {
            uint32_t temp_elapsed_ms = (now_ticks - temp_closed_start) * portTICK_PERIOD_MS;
            uint32_t temp_min = temp_elapsed_ms / 60000;
            uint32_t temp_sec = (temp_elapsed_ms % 60000) / 1000;
            snprintf(temp_timer_str, sizeof(temp_timer_str), "%lu:%02lu / %lu:00", temp_min, temp_sec, config.config_05);
        }
        
        // Status buffer esteso
        const char* buf_ext_status = encoder_buffer_extended ? "OK" : "ERR";
        
        snprintf(buffer, buffer_size,
            "=== STATO DOOR TASK ===\n"
            "Timestamp: %s\n"
            "\n"
            "PORTA:\n"
            "  Modalità: %s  |  Aperta: %s  |  Temp closed: %s\n"
            "\n"
            "ENCODER:\n"
            "  Raw: %u  |  Corrected: %u (%.1f°)  |  Magnitude: %u\n"
            "  Buffer index: %u  |  Buffer ext idx: %zu  |  I2C: %s\n"
            "  Buffer ext: %s  |  Buffer ext addr: %p\n"
            "\n"
            "GPIO:\n"
            "  Infrared: %s  |  Detect: %s  |  LED rosso: %s  |  LED blu: %s\n"
            "\n"
            "EVENTI:\n"
            "  Stato: %s  |  Confermato: %s  |  Direzione: %s\n"
            "  Codice assoc: %s  |  Nome: %s\n"
            "  Indice inizio: %lu  |  Trigger: %ld  |  Detect: %ld  |  Infrared: %ld\n"
            "  Senza trigger: %lu  |  Ultimo trigger: %lu\n"
            "\n"
            "TIMERS:\n"
            "  Last unauth elapsed: %lu sec  |  Door open: %lu sec\n"
            "  Temp closed: %s\n"
            "  Last auth DC: %llu  |  CC: %u\n",
            
            time_str,
            mode_str,
            door_open ? "SÌ" : "NO",
            temp_closed_active ? "SÌ" : "NO",
            lastRawAngle, lastCorrectedAngle, degrees, lastMagnitude,
            (unsigned)encoder_buffer_index, encoder_buffer_extended_index, 
            encoder_i2c_initialized ? "OK" : "ERR",
            buf_ext_status, encoder_buffer_extended,
            gpio_get_level(INFRARED) ? "ON" : "OFF",
            gpio_get_level(DETECTED) ? "ON" : "OFF",
            gpio_get_level(LED_ROSSO) ? "ON" : "OFF",
            gpio_get_level(LED_BLU) ? "ON" : "OFF",
            stato_str,
            passaggio_confermato ? "SÌ" : "NO",
            direzione ? direzione : "N/A",
            codice_associato ? "SÌ" : "NO",
            cat_name_evento,
            indice_inizio, trigger_porta_idx, detect_idx, infrared_idx,
            conteggio_senza_trigger, ultimo_trigger_idx,
            unauth_elapsed_sec, door_elapsed_sec,
            temp_timer_str,
            (unsigned long long)last_authorized_device_code,
            last_authorized_country_code
        );
    } else {
        snprintf(buffer, buffer_size, "Subcomandi non implementati per 'door'");
    }
}

// NUOVA FUNZIONE DIAGNOSTICA per buffer esteso
extern "C" void get_buffer_extended_status(char* buffer, size_t buffer_size) {
    if (encoder_buffer_extended == nullptr) {
        snprintf(buffer, buffer_size, "ERRORE: Buffer esteso non inizializzato");
        return;
    }
    
    time_t now;
    time(&now);
    struct tm *tm_now = localtime(&now);
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_now);
    
    // Calcola età del campione più vecchio nel buffer
    uint32_t samples_in_buffer = (encoder_buffer_extended_total_samples < ENCODER_BUFFER_EXTENDED_SIZE) 
                               ? (uint32_t)encoder_buffer_extended_total_samples 
                               : ENCODER_BUFFER_EXTENDED_SIZE;
    
    float hours_coverage = (samples_in_buffer * 0.1f) / 3600.0f;
    float memory_used_mb = (sizeof(EncoderDataExtended) * ENCODER_BUFFER_EXTENDED_SIZE) / (1024.0f * 1024.0f);
    float fill_percentage = (float)samples_in_buffer / ENCODER_BUFFER_EXTENDED_SIZE * 100.0f;
    
    snprintf(buffer, buffer_size,
        "=== BUFFER ESTESO ===\n"
        "Timestamp: %s\n"
        "Indirizzo: %p\n"
        "\n"
        "DIMENSIONI:\n"
        "  Capacità: %lu campioni (24h esatte)\n"
        "  Dimensione elemento: %zu bytes\n"
        "  Memoria totale: %.1f MB\n"
        "  Copertura: 24.0 ore\n"
        "\n"
        "STATO ATTUALE:\n"
        "  Indice corrente: %zu\n"
        "  Campioni nel buffer: %lu\n"
        "  Riempimento: %.1f%%\n"
        "  Copertura attuale: %.1f ore\n"
        "\n"
        "STATISTICHE:\n"
        "  Campioni totali: %llu\n"
        "  Avvolgimenti: %lu\n"
        "  Ultimo elemento:\n"
        "    Angle: %lu, IR: %lu, Detect: %lu\n"
        "    Door: %lu, TempClosed: %lu, Auth: %lu\n"
        "    Distance: %lu mm\n",

        time_str,
        encoder_buffer_extended,
        (unsigned long)ENCODER_BUFFER_EXTENDED_SIZE,
        sizeof(EncoderDataExtended),
        memory_used_mb,

        encoder_buffer_extended_index,
        (unsigned long)samples_in_buffer,
        fill_percentage,
        hours_coverage,

        (unsigned long long)encoder_buffer_extended_total_samples,
        (unsigned long)encoder_buffer_extended_wraps,

        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].rawAngle :
            (encoder_buffer_extended_total_samples > 0 ?
                encoder_buffer_extended[ENCODER_BUFFER_EXTENDED_SIZE - 1].rawAngle : 0)),
        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].infrared : 0),
        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].detect : 0),
        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].door_open : 0),
        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].temp_closed : 0),
        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].authorized : 0),
        (unsigned long)(encoder_buffer_extended_index > 0 ?
            encoder_buffer_extended[encoder_buffer_extended_index - 1].distance : 0)
    );
}


// Funzione per trovare l'indice del buffer più vicino a un timestamp
static size_t find_buffer_index_by_timestamp(time_t target_timestamp) {
    if (encoder_buffer_extended == nullptr || encoder_buffer_extended_total_samples == 0) {
        return 0;
    }
    
    // Calcola timestamp del campione più recente (ora corrente)
    time_t now;
    time(&now);
    
    // Calcola differenza in campioni (1 campione = 0.1s = 100ms)
    int64_t diff_seconds = now - target_timestamp;
    int64_t diff_samples = diff_seconds * 10; // 10 campioni per secondo
    
    // Gestisci casi limite
    if (diff_samples < 0) {
        // Timestamp nel futuro - ritorna l'indice più recente
        return encoder_buffer_extended_index;
    }
    
    if (diff_samples >= (int64_t)encoder_buffer_extended_total_samples) {
        // Timestamp troppo vecchio - ritorna il campione più vecchio disponibile
        if (encoder_buffer_extended_total_samples < ENCODER_BUFFER_EXTENDED_SIZE) {
            return 0; // Buffer non ancora pieno
        } else {
            return (encoder_buffer_extended_index + 1) % ENCODER_BUFFER_EXTENDED_SIZE;
        }
    }
    
    // Calcola indice target nel buffer circolare
    size_t samples_back = (size_t)diff_samples;
    size_t target_index = (encoder_buffer_extended_index + ENCODER_BUFFER_EXTENDED_SIZE - samples_back) 
                         % ENCODER_BUFFER_EXTENDED_SIZE;
    
    return target_index;
}

// Funzione per estrarre una finestra di campioni
    static size_t extract_buffer_window(time_t center_timestamp, int duration_seconds,   // MODIFICA: era duration_minutes
                                   EncoderDataExtended* output, size_t max_samples) {
    if (encoder_buffer_extended == nullptr || output == nullptr || max_samples == 0) {
        return 0;
    }
    
// Calcola finestra temporale: [center - 1s] ... [center + duration*60s + 1s]
    time_t start_timestamp = center_timestamp - 1;
    time_t end_timestamp = center_timestamp + duration_seconds + 1;  // MODIFICA: era (duration_minutes * 60)
    
    // Calcola numero di campioni richiesti
    int64_t window_seconds = end_timestamp - start_timestamp;
    size_t window_samples = (size_t)(window_seconds * 10); // 10 campioni/sec
    
    // Limita ai campioni disponibili
    if (window_samples > max_samples) {
        window_samples = max_samples;
    }
    
    // Trova indice di partenza (dal timestamp di inizio)
    size_t start_index = find_buffer_index_by_timestamp(start_timestamp);
    
    // Estrai campioni con gestione wrap circolare
    size_t extracted = 0;
    for (size_t i = 0; i < window_samples && extracted < max_samples; i++) {
        size_t buffer_index = (start_index + i) % ENCODER_BUFFER_EXTENDED_SIZE;
        
        // Verifica che il campione sia valido (non oltre i campioni totali)
        if (encoder_buffer_extended_total_samples < ENCODER_BUFFER_EXTENDED_SIZE) {
            // Buffer non ancora pieno
            if (buffer_index >= encoder_buffer_extended_total_samples) {
                break;
            }
        }
        
        output[extracted] = encoder_buffer_extended[buffer_index];
        extracted++;
    }
    
    ESP_LOGI("DOOR", "Estratti %zu campioni per finestra %ld-%ld (centro: %ld)", 
             extracted, (long)start_timestamp, (long)end_timestamp, (long)center_timestamp);
    
    return extracted;
}

// Funzione helper per calcolare timestamp di un campione
static time_t get_sample_timestamp(size_t sample_index) {
    time_t now;
    time(&now);
    
    // Calcola quanti campioni indietro rispetto ad ora
    size_t current_index = encoder_buffer_extended_index;
    size_t samples_back;
    
    if (sample_index <= current_index) {
        samples_back = current_index - sample_index;
    } else {
        // Gestione wrap
        samples_back = current_index + (ENCODER_BUFFER_EXTENDED_SIZE - sample_index);
    }
    
    // Converti in secondi (1 campione = 0.1s)
    time_t timestamp = now - (samples_back / 10);
    
    return timestamp;
}

extern "C" size_t get_buffer_window_for_timestamp(time_t center_timestamp, int duration_seconds,  // MODIFICA: era duration_minutes
                                                  EncoderDataExtended* output, size_t max_samples,
                                                  time_t* actual_start_timestamp) {
    if (actual_start_timestamp) {
        *actual_start_timestamp = center_timestamp - 1; // Inizia 1 secondo prima
    }
    
    return extract_buffer_window(center_timestamp, duration_seconds, output, max_samples);  // MODIFICA: era duration_minutes
}

// =====================================
// Main door task
// =====================================
extern "C" void door_task(void *pv) {
    door_gpio_init_once();
    servo_init_once();

    // NUOVO: Inizializza buffer esteso in PSRAM
    esp_err_t ret = init_extended_buffer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Impossibile inizializzare buffer esteso, task terminato");
        vTaskDelete(NULL);
        return;
    }

    // DEBUG: Verifica dove sono allocati i buffer
    ESP_LOGI(TAG, "=== DEBUG BUFFER ALLOCATION ===");
    ESP_LOGI(TAG, "encoder_buffer address: %p", encoder_buffer);
    ESP_LOGI(TAG, "encoder_buffer size: %u bytes", sizeof(encoder_buffer));
    
    // Verifica se è in PSRAM (indirizzi PSRAM iniziano tipicamente con 0x3C...)
    if ((uint32_t)encoder_buffer >= 0x3C000000 && (uint32_t)encoder_buffer < 0x3E000000) {
        ESP_LOGI(TAG, "encoder_buffer è in PSRAM ✓");
    } else {
        ESP_LOGI(TAG, "encoder_buffer è in DRAM interna ✗");
    }

    // Inizializza AS5600 (I2C_NUM_0)
    if (!encoder_i2c_initialized) {
        i2c_config_t i2c_config = {};
        i2c_config.mode = I2C_MODE_MASTER;
        i2c_config.sda_io_num = ENCODER_SDA;
        i2c_config.scl_io_num = ENCODER_SCL;
        i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_config.master.clk_speed = 400000; // 400kHz

        esp_err_t ret = i2c_param_config(I2C_NUM_0, &i2c_config);
        if (ret == ESP_OK) {
            ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Errore inizializzazione I2C encoder: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Encoder userà valori di default");
        } else {
            encoder_i2c_initialized = true;
            ESP_LOGI(TAG, "I2C encoder inizializzato su SDA=%d, SCL=%d", ENCODER_SDA, ENCODER_SCL);
        }
    }

    // Inizializza VL6180X (I2C_NUM_1 - bus separato) se abilitato
    // config_06 >= 1000 significa disabilitato (es. 1081 = disabilitato con offset 81 memorizzato)
    if (!vl6180x_i2c_initialized && config.config_06 < 1000) {
        i2c_config_t i2c_config = {};
        i2c_config.mode = I2C_MODE_MASTER;
        i2c_config.sda_io_num = VL6180X_SDA;
        i2c_config.scl_io_num = VL6180X_SCL;
        i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_config.master.clk_speed = 400000; // 400kHz

        esp_err_t ret = i2c_param_config(I2C_NUM_1, &i2c_config);
        if (ret == ESP_OK) {
            ret = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Errore inizializzazione I2C VL6180X: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "VL6180X non disponibile");
        } else {
            ESP_LOGI(TAG, "I2C VL6180X inizializzato su SDA=%d, SCL=%d", VL6180X_SDA, VL6180X_SCL);

            // Inizializza il sensore VL6180X
            ret = vl6180x_init();
            if (ret == ESP_OK) {
                vl6180x_i2c_initialized = true;
                ESP_LOGI(TAG, "VL6180X pronto");
            } else {
                ESP_LOGW(TAG, "VL6180X init fallita, sensore non risponde");
            }
        }
    } else if (config.config_06 >= 1000) {
        ESP_LOGI(TAG, "VL6180X disabilitato (config_06=%u >= 1000)", config.config_06);
    }

    char time_str[20];
    bool log_emitted = false;
    DoorMode last_mode = AUTO;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(100);
    bool detect = false;

    // NUOVO: Contatore per lampeggio LED rosso in TEMP_CLOSED (1 Hz, 5 cicli ON, 5 cicli OFF)
    static uint32_t led_blink_counter = 0;

    // Stato iniziale coerente con config.door_mode
    switch (config.door_mode) {
        case ALWAYS_OPEN:
            if (!door_open) {
                gpio_set_level(LED_ROSSO, LED_ON);
                door_open = true;
                setServoPosition(true);
                time_t now_init;
                time(&now_init);
                struct tm *tm_now = localtime(&now_init);
                strftime(time_str, sizeof(time_str), "%d/%m/%Y %H:%M:%S", tm_now);
                add_log_entry(now_init, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);
                ESP_LOGI(TAG, "Porta aperta per modalità ALWAYS_OPEN");
            }
            break;
        case ALWAYS_CLOSED:
            if (door_open) {
                gpio_set_level(LED_ROSSO, LED_OFF);
                door_open = false;
                setServoPosition(false);
                time_t now_init;
                time(&now_init);
                struct tm *tm_now = localtime(&now_init);
                strftime(time_str, sizeof(time_str), "%d/%m/%Y %H:%M:%S", tm_now);
                add_log_entry(now_init, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);
                ESP_LOGI(TAG, "Porta chiusa per modalità ALWAYS_CLOSED");
            }
            break;
        case AUTO:
        default:
            if (door_open) {
                gpio_set_level(LED_ROSSO, LED_OFF);
                door_open = false;
                setServoPosition(false);
                time_t now_init;
                time(&now_init);
                struct tm *tm_now = localtime(&now_init);
                strftime(time_str, sizeof(time_str), "%d/%m/%Y %H:%M:%S", tm_now);
                add_log_entry(now_init, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                ESP_LOGI(TAG, "Porta chiusa per modalità AUTO");
            }
            break;
    }
    last_mode = config.door_mode;

    while (true) {
        // Variabili per il ciclo corrente
        char cat_name[33] = "Sconosciuto";
        bool is_authorized = false;
        uint16_t country_code = 0;
        uint64_t device_code = 0;

        // Timestamp corrente
        time_t now;
        time(&now);
        struct tm *tm_now = localtime(&now);
        strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_now);
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", tm_now);

        // Controllo cambio modalità 
        DoorMode current_mode = config.door_mode;
        if (current_mode != last_mode) {
            // NUOVO: Resetta stato TEMP_CLOSED al cambio modalità 
            if (temp_closed_active) {
                temp_closed_active = false;
                temp_closed_start = 0;
                led_blink_counter = 0;
                gpio_set_level(LED_ROSSO, current_mode == ALWAYS_OPEN ? LED_ON : LED_OFF);
                ESP_LOGI(TAG, "[%s] TEMP_CLOSED resettato per cambio modalità", time_str);
            }
            if (current_mode == ALWAYS_OPEN) {
                if (!door_open) {
                    gpio_set_level(LED_ROSSO, LED_ON);
                    door_open = true;
                    setServoPosition(true);
                    add_log_entry(now, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);
                    ESP_LOGI(TAG, "Porta aperta per modalità ALWAYS_OPEN");
                }
            } else if (current_mode == ALWAYS_CLOSED) {
                if (door_open) {
                    gpio_set_level(LED_ROSSO, LED_OFF);
                    door_open = false;
                    setServoPosition(false);
                    add_log_entry(now, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);
                    ESP_LOGI(TAG, "Porta chiusa per modalità ALWAYS_CLOSED");
                }
            } else if (current_mode == AUTO) {
                if (door_open) {
                    gpio_set_level(LED_ROSSO, LED_OFF);
                    door_open = false;
                    setServoPosition(false);
                    add_log_entry(now, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                    ESP_LOGI(TAG, "Porta chiusa per modalità AUTO");
                } else {
                    add_log_entry(now, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                    ESP_LOGI(TAG, "Passaggio a modalità AUTO");
                }
            }
            last_mode = current_mode;
        }

        // NUOVO: Gestione lampeggio LED rosso in TEMP_CLOSED (1 Hz, 500 ms ON, 500 ms OFF)
        if (temp_closed_active) {
            led_blink_counter = (led_blink_counter + 1) % 10; // 10 cicli = 1 secondo (100 ms * 10)
            gpio_set_level(LED_ROSSO, (led_blink_counter < 5) ? LED_ON : LED_OFF);
        }

        // Rilevamento RFID (usa le variabili condivise da core1)
        detect = false;
        bool newcode = false;
        
        if (door_sync_count > 0) {
            gpio_set_level(DETECTED, LED_ON);
            detect = true;
            is_authorized = false;
            country_code = last_country_code;
            device_code = last_device_code;

            // Cerca nei gatti autorizzati
            for (int i = 0; i < config.num_cats; i++) {
                if (config.authorized_cats[i].device_code == last_device_code) {
                    strncpy(cat_name, config.authorized_cats[i].name.c_str(), sizeof(cat_name) - 1);
                    cat_name[sizeof(cat_name) - 1] = '\0';
                    is_authorized = config.authorized_cats[i].authorized;
                    country_code = config.authorized_cats[i].country_code;
                    break;
                }
            }

            // NUOVO: Gestione chiusura temporanea in ALWAYS_OPEN
            if (current_mode == ALWAYS_OPEN && config.config_05 > 0) {
                if (!is_authorized) {
                    // Gatto non autorizzato: chiudi la porta immediatamente
                    if (!temp_closed_active || door_open) {
                        door_open = false;
                        setServoPosition(false);
                        temp_closed_active = true;
                        temp_closed_start = xTaskGetTickCount();
                        led_blink_counter = 0; // Inizia lampeggio
                        add_log_entry(now, "Chiusura non autorizzato", cat_name, country_code, device_code, false);
                        ESP_LOGI(TAG, "[%s] Porta chiusa per gatto non autorizzato: %s", time_str, cat_name);
                    } else {
                        // Reinizializza il timer per nuovo codice non autorizzato
                        temp_closed_start = xTaskGetTickCount();
                        add_log_entry(now, "Chiusura prolungata non auth", cat_name, country_code, device_code, false);
                        ESP_LOGI(TAG, "[%s] Timeout chiusura prolungato per gatto non autorizzato: %s", time_str, cat_name);
                    }
                } else if (temp_closed_active) {
                    // Gatto autorizzato: riapri immediatamente
                    door_open = true;
                    temp_closed_active = false;
                    temp_closed_start = 0;
                    led_blink_counter = 0;
                    gpio_set_level(LED_ROSSO, LED_ON); // LED fisso per ALWAYS_OPEN
                    setServoPosition(true);
                    add_log_entry(now, "Riapertura autorizzato", cat_name, country_code, device_code, true);
                    ESP_LOGI(TAG, "[%s] Porta riaperta per gatto autorizzato: %s", time_str, cat_name);
                }
            }

            // Gestione newcode durante evento attivo
            if (stato_evento == EVENTO_ATTIVO) {
                if (!codice_associato) {
                    codice_associato = true;
                    strncpy(cat_name_evento, cat_name, sizeof(cat_name_evento) - 1);
                    cat_name_evento[sizeof(cat_name_evento) - 1] = '\0';
                    country_code_evento = country_code;
                    device_code_evento = device_code;
                    authorized_evento = is_authorized;
                    ESP_LOGI(TAG, " CODICE");
                } else if (device_code != device_code_evento || country_code != country_code_evento) {
                    newcode = true;
                    ESP_LOGI(TAG, " NEWCODE, CC: %u, DC: %llu,ECC: %u, EDC: %llu,",
                    country_code, (unsigned long long)device_code,country_code_evento, (unsigned long long)device_code_evento);
                }
            }

            // Gestione autorizzazione e log
            if (is_authorized) {
                if (last_device_code != last_authorized_device_code || 
                    country_code != last_authorized_country_code) {
                    log_emitted = false;
                    last_authorized_device_code = last_device_code;
                    last_authorized_country_code = country_code;
                }
                door_timer_start = xTaskGetTickCount();
                if (!log_emitted) {
                    ESP_LOGI(TAG, "[%s] Rilevato gatto: %s, Autorizzato: Sì", time_str, cat_name);
                    log_emitted = true;
                }
                if (!door_open && current_mode == AUTO) {
                    gpio_set_level(LED_ROSSO, LED_ON);
                    door_open = true;
                    setServoPosition(true);
                }
            } else {
                if (xTaskGetTickCount() - last_unauthorized_log >= pdMS_TO_TICKS(config.unauthorized_log_interval)) {
                    if (strcmp(cat_name, "Sconosciuto") != 0) {
                        ESP_LOGI(TAG, "[%s] Rilevato gatto: %s, Autorizzato: No", time_str, cat_name);
                    } else {
                        ESP_LOGI(TAG, "[%s] Gatto sconosciuto, CC: %u, DC: %llu, Non autorizzato",
                                time_str, country_code, (unsigned long long)device_code);
                    }
                    last_unauthorized_log = xTaskGetTickCount();
                }
            }
            door_sync_count = 0; // Reset dopo elaborazione
        } else {
            gpio_set_level(DETECTED, LED_OFF);
            
            // NUOVO: Controllo timeout TEMP_CLOSED
            if (temp_closed_active && config.config_05 > 0) {
                TickType_t now_ticks = xTaskGetTickCount();
                if ((now_ticks - temp_closed_start) >= pdMS_TO_TICKS(config.config_05 * 60000)) {
                    temp_closed_active = false;
                    temp_closed_start = 0;
                    led_blink_counter = 0;
                    if (current_mode == ALWAYS_OPEN) {
                        door_open = true;
                        setServoPosition(true);
                        gpio_set_level(LED_ROSSO, LED_ON); // LED fisso per ALWAYS_OPEN
                        add_log_entry(now, "Riapertura timeout", "Timeout TEMP_CLOSED", 0, 0, true);
                        ESP_LOGI(TAG, "[%s] Porta riaperta dopo timeout TEMP_CLOSED (%u min)", time_str, config.config_05);
                    }
                }
            }

            // Controllo pulsante manuale
            if (!gpio_get_level(PULSANTE_BLU) && !door_open && current_mode == AUTO) {
                gpio_set_level(LED_ROSSO, LED_ON);
                door_open = true;
                door_timer_start = xTaskGetTickCount();
                setServoPosition(true);
                ESP_LOGI(TAG, "[%s] Apertura manuale tramite pulsante", time_str);
                add_log_entry(now, "Manuale", "Manuale", 0, 0, true);
                log_emitted = true;
            } else if (door_open) {
                // Controllo timeout chiusura
                TickType_t now_ticks = xTaskGetTickCount();
                if ((now_ticks - door_timer_start) >= pdMS_TO_TICKS(config.door_timeout)) {
                    log_emitted = false;
                    if (current_mode == AUTO) {
                        gpio_set_level(LED_ROSSO, LED_OFF);
                        door_open = false;
                        setServoPosition(false);
                        ESP_LOGI(TAG, "[%s] Porta chiusa dopo timeout", time_str);
                    }
                }
            }
        }

        // Leggi encoder e infrared (ottimizzato: burst read)
        uint16_t rawAngle, magnitude;
        encoder_readAngleAndMagnitude(&rawAngle, &magnitude);
        bool infrared = gpio_get_level(INFRARED) || interruptFlag;  // Sente sia il livello che i fronti in interrupt
        interruptFlag = false;

        // Leggi distanza VL6180X (se inizializzato)
        if (vl6180x_i2c_initialized) {
            lastDistance = vl6180x_readDistance();
        }

        // LED blu per infrarosso
        gpio_set_level(LED_BLU, infrared ? LED_ON : LED_OFF);

        if (rawAngle == 0xFFFF) {
            rawAngle = 0x3FFF; // Gestione errore
        }

        // Calcolo correctedAngle (logica Arduino identica)
        uint16_t correctedAngle;
        if (rawAngle == config.door_rest) {
            correctedAngle = 2000;
        } else if (rawAngle >= config.door_rest) {
            if (config.door_out != config.door_rest) {
                int32_t temp = 2048 + (1024 * (int32_t)(rawAngle - config.door_rest)) / 
                              (int32_t)(config.door_out - config.door_rest);
                correctedAngle = (uint16_t)temp;
            } else {
                correctedAngle = 2048;
            }
        } else {
            if (config.door_rest != config.door_in) {
                int32_t temp = 1024 + (1024 * (int32_t)(rawAngle - config.door_in)) / 
                              (int32_t)(config.door_rest - config.door_in);
                correctedAngle = (uint16_t)temp;
            } else {
                correctedAngle = 2048;
            }
        }

        // ANALISI EVENTI (logica Arduino identica)
        int32_t angolo_deviazione = abs((int32_t)correctedAngle - 2048);
        bool trigger = (angolo_deviazione > (int32_t)config.config_02 || infrared || detect);

        if (stato_evento == IDLE) {
            if (trigger) {
                stato_evento = EVENTO_ATTIVO;
                timestamp_inizio_time = now;
                strncpy(timestamp_inizio, timestamp_full, sizeof(timestamp_inizio));
                indice_inizio = 0;
                passaggio_confermato = false;
                direzione = NULL;
                conteggio_senza_trigger = 0;
                ultimo_trigger_idx = 0;
                trigger_porta_idx = (angolo_deviazione > (int32_t)config.config_02) ? 0 : -1;
                passaggio_porta_idx = -1;
                detect_idx = detect ? 0 : -1;
                infrared_idx = infrared ? 0 : -1;
                codice_associato = false;
                
                if (detect) {
                    strncpy(cat_name_evento, cat_name, sizeof(cat_name_evento) - 1);
                    cat_name_evento[sizeof(cat_name_evento) - 1] = '\0';
                    country_code_evento = country_code;
                    device_code_evento = device_code;
                    authorized_evento = is_authorized;
                    codice_associato = true;
                } else {
                    strcpy(cat_name_evento, "Sconosciuto");
                    country_code_evento = 0;
                    device_code_evento = 0;
                    authorized_evento = false;
                }
                ESP_LOGD(TAG, "Evento iniziato a %s", timestamp_inizio);
            }
        } else if (stato_evento == EVENTO_ATTIVO) {
            indice_inizio++;
            
            if (angolo_deviazione > (int32_t)config.config_03 && !passaggio_confermato) {
                passaggio_confermato = true;
                direzione = (correctedAngle > 2048) ? "Uscita" : "Ingresso";
                passaggio_porta_idx = indice_inizio;
                ESP_LOGD(TAG, "Passaggio confermato (%s) al campione %u", direzione, indice_inizio);
            }
            
            if (detect && !codice_associato) {
                strncpy(cat_name_evento, cat_name, sizeof(cat_name_evento) - 1);
                cat_name_evento[sizeof(cat_name_evento) - 1] = '\0';
                country_code_evento = country_code;
                device_code_evento = device_code;
                authorized_evento = is_authorized;
                codice_associato = true;
            }
            
            if (trigger_porta_idx == -1 && angolo_deviazione > (int32_t)config.config_02) {
                trigger_porta_idx = indice_inizio;
            }
            if (detect_idx == -1 && detect) {
                detect_idx = indice_inizio;
            }
            if (infrared_idx == -1 && infrared) {
                infrared_idx = indice_inizio;
            }
            
            if (trigger) {
                conteggio_senza_trigger = 0;
                ultimo_trigger_idx = indice_inizio;
            } else {
                conteggio_senza_trigger++;
            }

            // Chiusura evento per newcode
            if (newcode) {
                float durata_effettiva = indice_inizio * 0.1f;
                const char* tipo = passaggio_confermato ? 
                    (strcmp(direzione, "Uscita") == 0 ? "*Uscita" : "*Ingresso") : "*Affaccio";
                
                add_log_entry(timestamp_inizio_time, tipo, cat_name_evento, country_code_evento, device_code_evento, authorized_evento);
                ESP_LOGI(TAG, "Evento chiuso forzatamente per newcode, tipo: %s, Nome: %s, Durata: %.2f s", 
                         tipo, cat_name_evento, durata_effettiva);
                stato_evento = IDLE;
                codice_associato = false;
                // Reinizializza per nuovo evento se trigger ancora attivo
                if (trigger) {
                    stato_evento = EVENTO_ATTIVO;
                    timestamp_inizio_time = now;
                    strncpy(timestamp_inizio, timestamp_full, sizeof(timestamp_inizio));
                    indice_inizio = 0;
                    passaggio_confermato = false;
                    direzione = NULL;
                    conteggio_senza_trigger = 0;
                    ultimo_trigger_idx = 0;
                    trigger_porta_idx = (angolo_deviazione > (int32_t)config.config_02) ? 0 : -1;
                    passaggio_porta_idx = -1;
                    detect_idx = detect ? 0 : -1;
                    infrared_idx = infrared ? 0 : -1;
                    codice_associato = detect;
                    if (detect) {
                        strncpy(cat_name_evento, cat_name, sizeof(cat_name_evento) - 1);
                        cat_name_evento[sizeof(cat_name_evento) - 1] = '\0';
                        country_code_evento = country_code;
                        device_code_evento = device_code;
                        authorized_evento = is_authorized;
                    } else {
                        strcpy(cat_name_evento, "Sconosciuto");
                        country_code_evento = 0;
                        device_code_evento = 0;
                        authorized_evento = false;
                    }
                }
            } else if (conteggio_senza_trigger >= config.config_04) {
                // Chiusura evento per timeout
                float durata_effettiva = (ultimo_trigger_idx + 1) * 0.1f;
                const char* tipo = passaggio_confermato ? 
                    (strcmp(direzione, "Uscita") == 0 ? "*Uscita" : "*Ingresso") : "*Affaccio";
                
                add_log_entry(timestamp_inizio_time, tipo, cat_name_evento, country_code_evento, device_code_evento, authorized_evento);
                ESP_LOGI(TAG, "Evento terminato, tipo: %s, Nome: %s, Durata: %.2f s", 
                         tipo, cat_name_evento, durata_effettiva);
                stato_evento = IDLE;
            }
        }

        // Salva nel buffer encoder ORIGINALE (mantenuto per compatibilità)
        EncoderData data = {};
        data.rawAngle = correctedAngle & 0xFFF; // 12 bit
        data.infrared = infrared ? 1 : 0;
        data.detect = detect ? 1 : 0;
        data.door_open = door_open ? 1 : 0;   
        data.newcode = newcode ? 1 : 0;
        
        encoder_buffer[encoder_buffer_index] = data;
        encoder_buffer_index = (encoder_buffer_index + 1) % ENCODER_BUFFER_SIZE;

        // Salva nel buffer ESTESO a 32-bit (nuovo) - SOLO se allocato
        if (encoder_buffer_extended != nullptr) {
            EncoderDataExtended data_ext = {};
            data_ext.rawAngle = correctedAngle & 0xFFF;     // 12 bit identico al vecchio
            data_ext.infrared = infrared ? 1 : 0;
            data_ext.detect = detect ? 1 : 0;
            data_ext.door_open = door_open ? 1 : 0;
            data_ext.newcode = newcode ? 1 : 0;
            data_ext.temp_closed = temp_closed_active ? 1 : 0;
            data_ext.manual_open = (!gpio_get_level(PULSANTE_BLU)) ? 1 : 0;
            data_ext.authorized = is_authorized ? 1 : 0;
            data_ext.servo_moving = 0;  // TODO: implementare rilevamento movimento servo
            data_ext.distance = lastDistance;  // Distanza VL6180X (0-255mm)
            data_ext.reserved = 0;
            
            // Salva con gestione avvolgimento circolare
            encoder_buffer_extended[encoder_buffer_extended_index] = data_ext;
            
            // CORREZIONE: Uso variabili temporanee per evitare warning volatile
            uint64_t temp_total = encoder_buffer_extended_total_samples;
            temp_total++;
            encoder_buffer_extended_total_samples = temp_total;
            
            encoder_buffer_extended_index = (encoder_buffer_extended_index + 1) % ENCODER_BUFFER_EXTENDED_SIZE;

            // Log avvolgimento ogni 24 ore
            if (temp_total > 0 && (temp_total % ENCODER_BUFFER_EXTENDED_SIZE) == 0) {
                uint32_t temp_wraps = encoder_buffer_extended_wraps;
                temp_wraps++;
                encoder_buffer_extended_wraps = temp_wraps;
                ESP_LOGI(TAG, "Buffer esteso avvolto - ciclo #%lu (campioni totali: %llu)", 
                         (unsigned long)temp_wraps, (unsigned long long)temp_total);
            }
        }

        lastRawAngle = rawAngle;
        lastMagnitude = magnitude;
        lastCorrectedAngle = correctedAngle;

        vTaskDelayUntil(&last_wake_time, interval);
    }
}