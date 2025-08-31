#ifndef CORE1_H
#define CORE1_H

#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Buffer sizes identici
#define ADC_BUFFER_SIZE    16384
#define CORR_BUFFER_SIZE     256
#define PEAKS_BUFFER_SIZE    256
#define DIST_BUFFER_SIZE     256

// Struttura Bit identica
typedef struct {
    int value;
    int pos;
} Bit;

// Variabili globali come nel tuo Arduino
extern volatile bool     buffer_ready;
extern volatile int32_t  available_samples;
extern uint32_t          ia;
extern volatile uint32_t i_interrupt;
extern int               statoacq;
extern volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
extern int32_t           filt[CORR_BUFFER_SIZE];
extern int32_t           corr[CORR_BUFFER_SIZE];
extern int32_t           peaks[PEAKS_BUFFER_SIZE];
extern int32_t           dist[DIST_BUFFER_SIZE];
extern Bit               bits[DIST_BUFFER_SIZE];
extern uint8_t           bytes[10];
extern uint32_t           num_picchi;
extern uint32_t           num_distanze;
extern uint32_t           num_bits;
extern uint16_t          country_code;
extern uint64_t          device_code;
extern bool              crc_ok;

// Nuove variabili globali (come nel tuo file)
extern volatile uint32_t sync_count;
extern volatile uint32_t door_sync_count;
extern volatile uint8_t  last_sequence[10];
extern volatile uint64_t last_device_code;
extern volatile uint16_t last_country_code;
extern volatile uint32_t last_sync_i;
//extern volatile bool     door_open;
//extern volatile TickType_t door_timer_start;
extern volatile uint32_t display_sync_count;
extern volatile uint16_t datoadc;   // lastADC

// Avvio: crea task *pinnata su core 1* e avvia il timer ISR per l'ADC
void start_rfid_task(void);

// ISR "storica" che fa il doppio push del campione
void onTimer(void);

#endif // CORE1_H
