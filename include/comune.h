#ifndef COMUNE_H
#define COMUNE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <string>

#define APERTO LOW
#define CHIUSO HIGH
#define LED_ON 0
#define LED_OFF 1

// Definizione dei pin
#define PWM_PIN                 GPIO_NUM_14      // RFID Reader
#define PBLUE                   GPIO_NUM_39      // Pulsante blu per apertura manuale
#define DETECTED                GPIO_NUM_15      // LED verde rilevato chip
#define LEDROSSO                GPIO_NUM_7       // LED rosso (porta aperta)
#define WIFI_LED                GPIO_NUM_6       // LED stato WiFi
#define LEDBLUE                 GPIO_NUM_5       // IR detected
#define INFRARED_PIN            GPIO_NUM_3       // Sensore infrarosso della gattaiola
#define INFRARED_ENABLE         GPIO_NUM_28      // Abilitazione sensore infrarosso
#define STEP_A_PLUS             GPIO_NUM_21
#define STEP_A_MINUS            GPIO_NUM_16
#define STEP_B_PLUS             GPIO_NUM_12
#define STEP_B_MINUS            GPIO_NUM_13
#define ENABLE_PIN              GPIO_NUM_37
#define SERVO_PWM_CHANNEL       0
#define SERVO_PWM_TIMER         LEDC_TIMER_0
#define SERVO_PWM_RESOLUTION    16
#define SERVO_PIN               GPIO_NUM_4
#define FDX_B_PIN               16
#define PWM_FREQ                134200
#define ENCODER_SDA             GPIO_NUM_10
#define ENCODER_SCL             GPIO_NUM_9

// Definizione dello stato della porta
typedef enum {
    AUTO,
    ALWAYS_OPEN,
    ALWAYS_CLOSED
} DoorMode;

// Dichiarazioni delle variabili globali
extern int8_t wifi_rssi;  // RSSI corrente Wi-Fi
extern volatile bool wifi_connected;
extern volatile DoorMode door_mode;
extern volatile uint32_t door_sync_count;
extern volatile uint32_t display_sync_count;
extern portMUX_TYPE doorModeMux;
extern SemaphoreHandle_t doorModeSemaphore;

// Strutture dati
typedef struct {
    char timestamp[20];
    const char* type;
    std::string name;
    uint16_t country_code;
    uint64_t device_code;
    bool authorized;
} LogEntry;

#define LOG_BUFFER_SIZE 100
extern LogEntry log_buffer[LOG_BUFFER_SIZE];
extern size_t log_buffer_index;

// Dichiarazioni di funzioni
void save_config();

#endif // COMUNE_H