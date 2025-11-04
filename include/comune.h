#ifndef COMUNE_H
#define COMUNE_H

#include <string>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_spiffs.h>
#include <driver/gpio.h> // Aggiunto per gpio_num_t


// Costanti
#define MAX_CATS 10
#define LOG_BUFFER_SIZE 100

#define WIFI_LED    GPIO_NUM_6      // LED giallo stato WiFi
#define F134KHZ      GPIO_NUM_14     // RFID Reader
#define FDX_B_PIN   GPIO_NUM_16     //Segnale di test FDX_B
#define ADC_PIN     GPIO_NUM_4      //Ingresso ADC
#define SCOPE_1     GPIO_NUM_21     //uscita test 1
#define SCOPE_2     GPIO_NUM_11     //uscita test 1

#define DETECTED         GPIO_NUM_15      // LED verde rilevato chip
#define LED_ROSSO        GPIO_NUM_7   // LED rosso (porta aperta)
#define LED_BLU          GPIO_NUM_5       // IR detected
#define PULSANTE_BLU     GPIO_NUM_39   // Pulsante blu per apertura manuale
#define INFRARED         GPIO_NUM_3   // Sensore infrarosso della gattaiola
#define INFRARED_ENABLE  GPIO_NUM_8      // Abilitazione sensore infrarosso
#define SERVO_PIN        GPIO_NUM_12    //Uscita pwm servomotore
#define SERVO_INVERT_PIN GPIO_NUM_21    //Jumper inversione servo (LOW=inverti, HIGH=normale)

#define ENCODER_SDA      GPIO_NUM_10
#define ENCODER_SCL      GPIO_NUM_9

#define VL6180X_SDA      GPIO_NUM_41
#define VL6180X_SCL      GPIO_NUM_42
#define VL6180X_VCC      GPIO_NUM_40   // Alimentazione controllata via GPIO
#define VL6180X_OFFSET_MM  35  // Offset calibrazione (mm da sottrarre)

#define LED_ON 0
#define LED_OFF 1

// Enumerazioni
typedef enum {
    AUTO,
    ALWAYS_OPEN,
    ALWAYS_CLOSED
} DoorMode;

typedef enum {
    SERVO,
    STEP
} MotorType;

// Struttura per un gatto autorizzato
typedef struct {
    uint64_t device_code;
    uint16_t country_code;
    std::string name;
    bool authorized;
} Cat;

// Struttura per un'entrata del log
typedef struct {
    uint64_t timestamp;
    uint64_t device_code;
    uint16_t country_code;
    bool authorized;
    char event[32];
} LogEntry;

// Struttura di configurazione
typedef struct {
    DoorMode door_mode;
    uint32_t door_timeout;
    uint32_t steps_per_movement;
    uint32_t step_interval_us;
    uint32_t wifi_reconnect_delay;
    uint32_t unauthorized_log_interval;
    bool wifi_verbose_log;
    MotorType motor_type;
    uint32_t servo_open_us;
    uint32_t servo_closed_us;
    uint32_t servo_transition_ms;
    uint32_t config_01; //RTX_B disable =0:
    uint32_t config_02; //Soglia porta trig:
    uint32_t config_03; //Soglia porta open:
    uint32_t config_04; //Door event durata:
    uint32_t config_05; //durata in minuti chiusuraper codice non autorizzato
    uint32_t config_06; //VL6180X: 0-999=offset mm, >=1000=disabilitato (es.1081=disab+offset81)
    uint32_t config_07; //dutycycle 134 khz 0= 50% <50= 0
    uint32_t config_08;
    uint32_t config_09;
    uint32_t config_10;
    uint32_t door_rest;
    uint32_t door_in;
    uint32_t door_out;
    Cat authorized_cats[MAX_CATS];
    uint8_t num_cats;
} config_t;



struct EncoderData {
    uint8_t infrared : 1;   // Bit 0
    uint8_t detect : 1;     // Bit 1
    uint8_t door_open : 1;  // Bit 2
    uint8_t newcode : 1;    // Bit 3
    uint16_t rawAngle : 12; // Bit 4-15
};

#define ENCODER_BUFFER_SIZE (1 << 14) // 2^14 = 16384
extern EXT_RAM_BSS_ATTR EncoderData encoder_buffer[ENCODER_BUFFER_SIZE];
extern volatile size_t encoder_buffer_index;



// Nuovo buffer a 32-bit per dati estesi (futuro replacement di encoder_buffer)
struct EncoderDataExtended {
    uint32_t infrared : 1;      // Bit 0 - Sensore infrarosso
    uint32_t detect : 1;        // Bit 1 - RFID rilevato
    uint32_t door_open : 1;     // Bit 2 - Porta aperta
    uint32_t newcode : 1;       // Bit 3 - Nuovo codice RFID
    uint32_t temp_closed : 1;   // Bit 4 - Stato temporaneamente chiuso (ALWAYS_OPEN mode)
    uint32_t manual_open : 1;   // Bit 5 - Apertura manuale (pulsante)
    uint32_t authorized : 1;    // Bit 6 - Codice autorizzato
    uint32_t servo_moving : 1;  // Bit 7 - Servo in movimento
    uint32_t rawAngle : 12;     // Bit 8-19 - Angolo raw (12 bit = 0-4095)
    uint32_t distance : 8;      // Bit 20-27 - Distanza VL6180X in mm (0-255mm)
    uint32_t reserved : 4;      // Bit 28-31 - Riservato per espansioni future
};

// Buffer esteso: esattamente 24 ore di campioni @ 100ms
// 24h * 3600s/h * 10 campioni/s = 864,000 campioni
#define ENCODER_BUFFER_EXTENDED_SIZE 864000

// Buffer esteso - Allocazione dinamica in PSRAM
extern EncoderDataExtended* encoder_buffer_extended;
extern volatile size_t encoder_buffer_extended_index;

// Statistiche buffer esteso
extern volatile uint32_t encoder_buffer_extended_wraps;  // Contatore avvolgimenti
extern volatile uint64_t encoder_buffer_extended_total_samples;  // Ca

// Variabili globali
extern config_t config;
extern LogEntry log_buffer[LOG_BUFFER_SIZE];
extern volatile bool wifi_connected; // Uniformato con volatile
extern int8_t wifi_rssi;
extern uint32_t log_count;

extern volatile uint16_t lastRawAngle;
extern volatile uint16_t lastMagnitude;
extern volatile uint16_t lastCorrectedAngle;
extern volatile uint8_t lastDistance;  // VL6180X distance in mm (0-255)


// Funzioni
esp_err_t init_spiffs(void);
esp_err_t save_config(void);
esp_err_t load_config(void);
void clear_log(void);

#ifdef __cplusplus
extern "C" {
#endif


// Monta SPIFFS per il boot: CONSENTE format se fallisce (utile al primo avvio)
esp_err_t spiffs_mount_boot(void);

// Monta SPIFFS in modalità “rigorosa”: NO format se fallisce (usalo dopo OTA dello SPIFFS)
esp_err_t spiffs_mount_strict(void);

// Smonta il filesystem
void spiffs_unmount(void);

// Info spazio
esp_err_t spiffs_get_info(size_t* total, size_t* used);

#ifdef __cplusplus
} // extern "C"
#endif
#endif // COMUNE