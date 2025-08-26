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
#define PWM_PIN     GPIO_NUM_14     // RFID Reader
#define FDX_B_PIN   GPIO_NUM_16     //Segnale di test FDX_B
#define ADC_PIN     GPIO_NUM_4      //Ingresso ADC
#define TEST_ADC    GPIO_NUM_21     //uscita test interrupt ADC

#define DETECTED         GPIO_NUM_15      // LED verde rilevato chip
#define LED_ROSSO        GPIO_NUM_7   // LED rosso (porta aperta)
#define LED_BLUE         GPIO_NUM_5       // IR detected
#define PULSANTE_BLU     GPIO_NUM_39   // Pulsante blu per apertura manuale
#define INFRARED         GPIO_NUM_3   // Sensore infrarosso della gattaiola
#define INFRARED_ENABLE  GPIO_NUM_8      // Abilitazione sensore infrarosso
#define SERVO_PIN        GPIO_NUM_12    //Uscita pwm servomotore

#define ENCODER_SDA      GPIO_NUM_10
#define ENCODER_SCL      GPIO_NUM_9

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
    uint32_t config_01;
    uint32_t config_02;
    uint32_t config_03;
    uint32_t config_04;
    uint32_t config_05;
    uint32_t config_06;
    uint32_t config_07;
    uint32_t config_08;
    uint32_t config_09;
    uint32_t config_10;
    uint32_t door_rest;
    uint32_t door_in;
    uint32_t door_out;
    Cat authorized_cats[MAX_CATS];
    uint8_t num_cats;
} config_t;

// Variabili globali
extern config_t config;
extern LogEntry log_buffer[LOG_BUFFER_SIZE];
extern volatile bool wifi_connected; // Uniformato con volatile
extern int8_t wifi_rssi;
extern uint32_t log_count;

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
#endif // COMUNE_H