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
static volatile bool door_open = false;
static TickType_t door_timer_start = 0;
static bool servo_inited = false;
static volatile bool interruptFlag = false;

// Variabili per tracking ultimo gatto autorizzato
static volatile uint64_t last_authorized_device_code = 0;
static volatile uint16_t last_authorized_country_code = 0;

// =====================================
// Encoder placeholder
// =====================================
static uint16_t encoder_readAngle(void) {
    // Ora legge dall'AS5600 invece di restituire valore fisso
    uint8_t msb, lsb;
    uint16_t angle = 2000; // Valore di default in caso di errore
    
    // Leggi MSB (bits 11:8)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0E, true); // AS5600_ANGLE_MSB_REG
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &msb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return angle;
    
    // Leggi LSB (bits 7:0)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0F, true); // AS5600_ANGLE_LSB_REG
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return angle;
    
    // Combina MSB e LSB
    angle = ((uint16_t)(msb & 0x0F) << 8) | lsb;
    angle &= 0x0FFF; // Limita a 12-bit
    
    return angle;
}

static uint16_t encoder_readMagnitude(void) {
    // Ora legge dall'AS5600 invece di restituire valore fisso
    uint8_t msb, lsb;
    uint16_t magnitude = 2100; // Valore di default
    
    // Leggi MSB magnitude
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1B, true); // AS5600_MAGNITUDE_MSB
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &msb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return magnitude;
    
    // Leggi LSB magnitude
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1C, true); // AS5600_MAGNITUDE_LSB
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x36 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return magnitude;
    
    // Combina MSB e LSB
    magnitude = ((uint16_t)(msb & 0x0F) << 8) | lsb;
    magnitude &= 0x0FFF;
    
    return magnitude;
}

// 3. AGGIUNGI variabile per I2C inizializzato e MODIFICA la funzione door_task():

// Aggiungi questa variabile globale dopo le altre variabili globali in door.cpp:
static bool encoder_i2c_initialized = false;


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

        .flags          = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    servo_inited = true;
}

static void setServoPosition(bool open) {
    static uint32_t current_servo_us = 0;
    if (current_servo_us == 0) current_servo_us = config.servo_closed_us;

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
}

// =====================================
// Main door task
// =====================================
extern "C" void door_task(void *pv) {
    door_gpio_init_once();
    servo_init_once();

    // Inizializza AS5600
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

    char time_str[20];
    bool log_emitted = false;
    unsigned long last_unauthorized_log = 0;
    DoorMode last_mode = AUTO;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(100);
    bool detect = false;

    // Stato iniziale coerente con config.door_mode
    switch (config.door_mode) {
        case ALWAYS_OPEN:
            if (!door_open) {
                gpio_set_level(LED_ROSSO, LED_ON);
                door_open = true;
                setServoPosition(true);
                time_t now_init;
                time(&now_init);  // CORREZIONE: usa time_t invece di stringa
                struct tm *tm_now = localtime(&now_init);
                strftime(time_str, sizeof(time_str), "%d/%m/%Y %H:%M:%S", tm_now);
                add_log_entry(now_init, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);  // CORREZIONE
                ESP_LOGI(TAG, "Porta aperta per modalità ALWAYS_OPEN");
            }
            break;
        case ALWAYS_CLOSED:
            if (door_open) {
                gpio_set_level(LED_ROSSO, LED_OFF);
                door_open = false;
                setServoPosition(false);
                time_t now_init;
                time(&now_init);  // CORREZIONE: usa time_t invece di stringa
                struct tm *tm_now = localtime(&now_init);
                strftime(time_str, sizeof(time_str), "%d/%m/%Y %H:%M:%S", tm_now);
                add_log_entry(now_init, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);  // CORREZIONE
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
                time(&now_init);  // CORREZIONE: usa time_t invece di stringa
                struct tm *tm_now = localtime(&now_init);
                strftime(time_str, sizeof(time_str), "%d/%m/%Y %H:%M:%S", tm_now);
                add_log_entry(now_init, "Modalità automatica", "Modalità Automatica", 0, 0, false);  // CORREZIONE
                ESP_LOGI(TAG, "Porta chiusa per modalità AUTO");
            }
            break;
    }
    last_mode = config.door_mode;

    // Variabili statiche per l'analisi degli eventi
    static enum { IDLE, EVENTO_ATTIVO } stato = IDLE;
    static char timestamp_inizio[20] = "";
    static time_t timestamp_inizio_time = 0;  // CORREZIONE: aggiungi variabile time_t
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
            if (current_mode == ALWAYS_OPEN) {
                if (!door_open) {
                    gpio_set_level(LED_ROSSO, LED_ON);
                    door_open = true;
                    setServoPosition(true);
                    add_log_entry(now, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);  // CORREZIONE: usa now invece di timestamp_full
                    ESP_LOGI(TAG, "Porta aperta per modalità ALWAYS_OPEN");
                }
            } else if (current_mode == ALWAYS_CLOSED) {
                if (door_open) {
                    gpio_set_level(LED_ROSSO, LED_OFF);
                    door_open = false;
                    setServoPosition(false);
                    add_log_entry(now, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);  // CORREZIONE: usa now invece di timestamp_full
                    ESP_LOGI(TAG, "Porta chiusa per modalità ALWAYS_CLOSED");
                }
            } else if (current_mode == AUTO) {
                if (door_open) {
                    gpio_set_level(LED_ROSSO, LED_OFF);
                    door_open = false;
                    setServoPosition(false);
                    add_log_entry(now, "Modalità automatica", "Modalità Automatica", 0, 0, false);  // CORREZIONE: usa now invece di timestamp_full
                    ESP_LOGI(TAG, "Porta chiusa per modalità AUTO");
                } else {
                    add_log_entry(now, "Modalità automatica", "Modalità Automatica", 0, 0, false);  // CORREZIONE: usa now invece di timestamp_full
                    ESP_LOGI(TAG, "Passaggio a modalità AUTO");
                }
            }
            last_mode = current_mode;
        }

        // Rilevamento RFID (usa le variabili condivise da core1)
        detect = false;
        bool newcode = false;
        
        if (door_sync_count > 0) {
            gpio_set_level(DETECTED, LED_ON);
            detect = true;
            is_authorized = false;
            strcpy(cat_name_evento, "Sconosciuto"); //0830  aggiunto da Arduino
            country_code = last_country_code;
            device_code = last_device_code;

            // Cerca nei gatti autorizzati
            for (int i = 0; i < config.num_cats; i++) {
                if (config.authorized_cats[i].device_code == last_device_code) {
                    strncpy(cat_name, config.authorized_cats[i].name.c_str(), sizeof(cat_name) - 1);
                    cat_name[sizeof(cat_name) - 1] = '\0';
                    is_authorized = config.authorized_cats[i].authorized;
                    country_code = config.authorized_cats[i].country_code;  //0830 superfluo
                    break;
                }
            }

            // Gestione newcode durante evento attivo
            if (stato == EVENTO_ATTIVO) {
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
            //contaporta = 0; //0830 verificare
            door_sync_count = 0; // Reset dopo elaborazione
        } else {
            gpio_set_level(DETECTED, LED_OFF);
            
            // Controllo pulsante manuale
            if (!gpio_get_level(PULSANTE_BLU) && !door_open && current_mode == AUTO) {
                gpio_set_level(LED_ROSSO, LED_ON);
                door_open = true;
                door_timer_start = xTaskGetTickCount();
                setServoPosition(true);
                //contaporta = 0; //0830 verificare
                ESP_LOGI(TAG, "[%s] Apertura manuale tramite pulsante", time_str);
                add_log_entry(now, "Manuale", "Manuale", 0, 0, true);  // CORREZIONE: usa now invece di timestamp_full
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

        // Leggi encoder e infrared
        uint16_t rawAngle = encoder_readAngle();
        uint16_t magnitude = encoder_readMagnitude();
        (void)magnitude; // Silenzia warning unused
        bool infrared = gpio_get_level(INFRARED) || interruptFlag;  // Sente sia il livello che i fronti in interrupt
        interruptFlag = false;

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

        if (stato == IDLE) {
            if (trigger) {
                stato = EVENTO_ATTIVO;
                timestamp_inizio_time = now;  // CORREZIONE: salva il time_t
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
        } else if (stato == EVENTO_ATTIVO) {
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
                
                add_log_entry(timestamp_inizio_time, tipo, cat_name_evento, country_code_evento, device_code_evento, authorized_evento);  // CORREZIONE: usa timestamp_inizio_time
                ESP_LOGI(TAG, "Evento chiuso forzatamente per newcode, tipo: %s, Nome: %s, Durata: %.2f s", 
                         tipo, cat_name_evento, durata_effettiva);
                stato = IDLE;
                codice_associato = false;
                // Reinizializza per nuovo evento se trigger ancora attivo
                if (trigger) {
                    stato = EVENTO_ATTIVO;
                    timestamp_inizio_time = now;  // CORREZIONE: salva il time_t
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
                
                add_log_entry(timestamp_inizio_time, tipo, cat_name_evento, country_code_evento, device_code_evento, authorized_evento);  // CORREZIONE: usa timestamp_inizio_time
                ESP_LOGI(TAG, "Evento terminato, tipo: %s, Nome: %s, Durata: %.2f s", 
                         tipo, cat_name_evento, durata_effettiva);
                stato = IDLE;

                //0830   questo pezzo aggiundo copiato da Arduino
                /*passaggio_confermato = false;
                direzione = NULL;
                conteggio_senza_trigger = 0;
                ultimo_trigger_idx = 0;
                trigger_porta_idx = -1;
                passaggio_porta_idx = -1;
                detect_idx = -1;
                infrared_idx = -1;
                codice_associato = false;*/
            }
        }

        // Salva nel buffer encoder (bit-packed)
        EncoderData data = {};
        data.rawAngle = correctedAngle & 0xFFF; // 12 bit
        data.infrared = infrared ? 1 : 0;
        data.detect = detect ? 1 : 0;
        data.door_open = door_open ? 1 : 0;   
        data.newcode = newcode ? 1 : 0;
        
        encoder_buffer[encoder_buffer_index] = data;
        encoder_buffer_index = (encoder_buffer_index + 1) % ENCODER_BUFFER_SIZE;

        lastRawAngle = rawAngle;
        lastMagnitude = magnitude;
        lastCorrectedAngle = correctedAngle;

        vTaskDelayUntil(&last_wake_time, interval);
    }
}