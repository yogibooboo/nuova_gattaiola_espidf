#ifndef COMUNE_H
#define COMUNE_H

//#include <Arduino.h> // Manteniamo questo per ora per la compatibilità con i tipi di dato e le API di base di Arduino, ma lo sostituiremo in futuro.
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
//#include "AS5600.h"
#include <string> 

// Definizione dei pin con la nuova nomenclatura in maiuscolo
#define PWM_PIN                 GPIO_NUM_32      // RFID Reader
#define PBLUE                   GPIO_NUM_25      // Pulsante blu per apertura manuale
#define LEDROSSO                GPIO_NUM_26      // LED rosso (porta aperta)
#define WIFI_LED                GPIO_NUM_27    // LED stato WiFi
#define INFRARED_PIN            GPIO_NUM_34      // Sensore infrarosso della gattaiola
#define INFRARED_ENABLE         GPIO_NUM_33      // Abilitazione sensore infrarosso

#define STEP_A_PLUS             GPIO_NUM_12
#define STEP_A_MINUS            GPIO_NUM_13
#define STEP_B_PLUS             GPIO_NUM_14
#define STEP_B_MINUS            GPIO_NUM_15
#define ENABLE_PIN              GPIO_NUM_16

#define SERVO_PWM_CHANNEL       0
#define SERVO_PWM_TIMER         LEDC_TIMER_0
#define SERVO_PWM_RESOLUTION    16
#define SERVO_PIN               GPIO_NUM_4

//variabili comuni per il debug:
extern int8_t wifi_rssi;  // RSSI corrente Wi-Fi

// Definizione dello stato della porta
typedef enum {
    AUTO,
    ALWAYS_OPEN,
    ALWAYS_CLOSED
} DoorMode;

// Variabili globali (sincronizzazione)
extern volatile DoorMode door_mode;
extern volatile bool wifi_connected;
extern volatile uint32_t door_sync_count;
extern volatile uint32_t display_sync_count;

// Mutex e semafori
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

#define LOG_BUFFER_SIZE 20
extern LogEntry log_buffer[LOG_BUFFER_SIZE];
extern size_t log_buffer_index;

// Dichiarazioni di funzioni
void save_config();

#endif // COMMON_H