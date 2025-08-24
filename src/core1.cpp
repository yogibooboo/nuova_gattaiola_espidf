// core1.cpp — versione corretta con calibrazione ADC
#include "core1.h"
#include "comune.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <driver/adc.h>
// #include <esp_adc_cal.h>  // *** COMMENTATO PER TEST ***
#include "esp_private/periph_ctrl.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "hal/adc_hal.h"
#include <esp_timer.h>

// ================== Variabili globali ==================
volatile bool     buffer_ready = false;
volatile int32_t  available_samples = 0;
uint32_t          ia = (uint32_t)-4;
volatile uint32_t i_interrupt = 0;
int               statoacq = 0;
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

int32_t           filt[CORR_BUFFER_SIZE];
int32_t           corr[CORR_BUFFER_SIZE];
int32_t           peaks[PEAKS_BUFFER_SIZE];
int32_t           dist[DIST_BUFFER_SIZE];
Bit               bits[DIST_BUFFER_SIZE];
uint8_t           bytes[10];
int32_t           num_picchi = 0;
int32_t           num_distanze = 0;
int32_t           num_bits = 0;
uint16_t          country_code = 0;
uint64_t          device_code = 0;
bool              crc_ok = false;

// Nuove variabili globali
volatile uint32_t sync_count = 0;
volatile uint32_t door_sync_count = 0;
volatile uint8_t  last_sequence[10];
volatile uint64_t last_device_code = 0;
volatile uint16_t last_country_code = 0;
volatile uint32_t last_sync_i = 0;
volatile bool     door_open = false;
volatile TickType_t door_timer_start = 0;
volatile uint32_t display_sync_count = 0;
volatile uint16_t datoadc = 0;

// *** VARIABILI GLOBALI SEMPLIFICATE ***
// *** CALIBRAZIONE ADC RIMOSSA PER TEST ***

// ===== Helper min/max =====
static inline int32_t imax(int32_t a, int32_t b){ return (a>b)?a:b; }
static inline int32_t imin(int32_t a, int32_t b){ return (a<b)?a:b; }

// *** VERSIONE SEMPLIFICATA SENZA CALIBRAZIONE (per test) ***
static bool adc_configured = false;

// ===== Fast-ADC ottimizzato per interrupt =====
static inline void fadcStart(uint8_t channel) {
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1U << channel);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
}

static inline bool fadcBusy() {
    return !(bool)SENS.sar_meas1_ctrl2.meas1_done_sar;
}

// Versione semplificata per debug: nessuna calibrazione
static inline uint16_t IRAM_ATTR fadcResult() {
    uint32_t raw = HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas1_ctrl2, meas1_data_sar) & 0x0FFF;
    return (uint16_t)raw;
}

// *** ELIMINA FUNZIONE CALIBRAZIONE ***

// Init semplificato: focus sui registri fondamentali 
static void fadcInit_IDF()
{
    printf("Inizializzazione ADC semplificata...\n");
    
    // 1. Alimenta SARADC
    periph_module_enable(PERIPH_SARADC_MODULE);

    // 2. Config GPIO come input analogico
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_DISABLE);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_FLOATING);

    // 3. Config ADC1 standard
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);

    // 4. Lettura dummy per inizializzare completamente l'ADC
    uint32_t dummy1 = adc1_get_raw(ADC1_CHANNEL_3);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    uint32_t dummy2 = adc1_get_raw(ADC1_CHANNEL_3);
    printf("Letture dummy: %lu, %lu\n", dummy1, dummy2);

    // 5. *** CONFIGURAZIONI REGISTRI SENS MINIME ***
    // Reset e setup base (compatibile ESP32-S3)
    SENS.sar_meas1_ctrl2.val = 0;  // Reset completo
    
    // Configurazioni essenziali
    SENS.sar_meas1_ctrl2.meas1_start_force = 1;     // controllo da CPU
    SENS.sar_meas1_ctrl2.sar1_en_pad_force = 1;     // selezione pad da CPU
    
    // Attenuazione diretta nel registro (12dB = 3)
    // sar_atten1: 2 bit per canale, CH3 = bit 6-7
    uint32_t atten_reg = SENS.sar_atten1;
    atten_reg &= ~(0x3 << 6);  // Clear CH3 bits
    atten_reg |= (0x3 << 6);   // Set 12dB per CH3
    SENS.sar_atten1 = atten_reg;
    
    // Prima conversione di setup
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1U << 3);   // CH3
    SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
    
    // Aspetta e verifica prima lettura
    int timeout = 1000;
    while(fadcBusy() && --timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (timeout <= 0) {
        printf("ERRORE: ADC non risponde!\n");
    } else {
        uint16_t first_read = fadcResult();
        printf("Prima lettura SENS: %u\n", first_read);
        
        // Confronto con API standard
        uint32_t std_read = adc1_get_raw(ADC1_CHANNEL_3);
        printf("Lettura API standard: %lu\n", std_read);
    }
    
    adc_configured = true;
    printf("ADC configurato (no calibrazione)\n");
}

// ====== GPTimer per scandire le conversioni ======
//static gptimer_handle_t s_adc_timer = nullptr;

// ISR GPIO sul pin portante: usa 1 fronte su 2 -> ~67 kHz
static void IRAM_ATTR pwm_edge_isr(void* /*arg*/) {
    static uint32_t edge = 0;
    edge += 1;
    if (edge & 1) {
        // dispari: salta (divide by 2)
        return;
    }

    // == era onTimer(): ==
    if (!fadcBusy()) {
        uint16_t s = fadcResult() & 0x0FFF;
        datoadc = s;
        adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
        adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
        fadcStart(3);
    } else {
        // opzionale: pezza se conversione non pronta -> ripeti ultimo campione
        uint16_t s = datoadc;
        adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
        adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
    }
}



// ====== Analisi (identica alla tua) ======
static void media_correlazione_32() {
    // [Il tuo codice di analisi rimane identico]
    static bool initialized = false;
    static int32_t max_i=0,min_i=0,max_i8=0,min_i8=0,min_i_iniziale=0,max_i_iniziale=0;
    static int32_t stato=1;
    static int32_t stato_decodifica=0,contatore_zeri=0,contatore_bytes=0,contatore_bits=0,stato_decobytes=0;
    static int32_t ultima_distanza=0,newbit=0,numbit=0;
    static bool newpeak=false;
    static uint8_t confro=0;

    if (!initialized) {
        num_picchi = num_distanze = num_bits = 0;
        for (int j=0;j<10;j++) bytes[j]=0;
        for (int j=0;j<CORR_BUFFER_SIZE;j++){ filt[j]=0; corr[j]=0; }
        initialized = true;
    }

    while (true) {
        available_samples = (int32_t)(i_interrupt - ia);
        if (available_samples <= 10) {
            vTaskDelay(pdMS_TO_TICKS(72));
            continue;
        }

        const int larghezza_finestra=8;
        const int lunghezza_correlazione=32;
        const int soglia_mezzo_bit=25;

        if (ia >= (uint32_t)lunghezza_correlazione) {
            // Filtro mobile
            filt[ ia        & (CORR_BUFFER_SIZE - 1)] =
                filt[(ia-1)  & (CORR_BUFFER_SIZE - 1)]
              - (int32_t)adc_buffer[(ia-4) & 0x3FFF] / larghezza_finestra
              + (int32_t)adc_buffer[(ia+3) & 0x3FFF] / larghezza_finestra;

            // “Correlazione” (2^ derivata)
            corr[(ia-16) & (CORR_BUFFER_SIZE - 1)] =
                corr[(ia-17) & (CORR_BUFFER_SIZE - 1)]
              - filt[(ia-32) & (CORR_BUFFER_SIZE - 1)]
              + 2 * filt[(ia-16) & (CORR_BUFFER_SIZE - 1)]
              -     filt[ ia     & (CORR_BUFFER_SIZE - 1)];

            newbit = 2; numbit = 0; newpeak = false;

            // Peak picking alternato
            if (stato == 1) {
                max_i  = imax(corr[(ia-16) & (CORR_BUFFER_SIZE - 1)], max_i);
                max_i8 = imax(corr[(ia-24) & (CORR_BUFFER_SIZE - 1)], max_i8);
                if ((max_i == max_i8) && (max_i != max_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = (int32_t)ia - 24;
                    num_picchi++;
                    stato = -1;
                    min_i = corr[(ia-16) & (CORR_BUFFER_SIZE - 1)];
                    min_i8 = corr[(ia-24) & (CORR_BUFFER_SIZE - 1)];
                    min_i_iniziale = min_i8;
                    newpeak = true;
                }
            } else {
                min_i  = imin(corr[(ia-16) & (CORR_BUFFER_SIZE - 1)], min_i);
                min_i8 = imin(corr[(ia-24) & (CORR_BUFFER_SIZE - 1)], min_i8);
                if ((min_i == min_i8) && (min_i != min_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = (int32_t)ia - 24;
                    num_picchi++;
                    stato = 1;
                    max_i = corr[(ia-16) & (CORR_BUFFER_SIZE - 1)];
                    max_i8 = corr[(ia-24) & (CORR_BUFFER_SIZE - 1)];
                    max_i_iniziale = max_i8;
                    newpeak = true;
                }
            }

            // Distanze/bit
            if (num_picchi > 1 && newpeak) {
                ultima_distanza = peaks[(num_picchi - 1) & 0xFF] - peaks[(num_picchi - 2) & 0xFF];
                dist[num_distanze & 0xFF] = ultima_distanza;
                num_distanze++;

                if (stato_decodifica == 0) {
                    if (ultima_distanza >= soglia_mezzo_bit) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                        num_bits++; newbit = 1; numbit = 1;
                    } else {
                        stato_decodifica = 1;
                    }
                } else if (stato_decodifica == 1) {
                    confro = 42;
                    if (bits[(num_bits - 1) & 0xFF].value == 1) confro = 48;

                    if ((ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= confro) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos   = (int32_t)ia - 24 - ultima_distanza;
                        num_bits++; newbit = 1; numbit = 1;
                        stato_decodifica = 2;

                        if ((ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= 52) {
                            bits[num_bits & 0xFF].value = 1;
                            bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                            num_bits++; newbit = 1; numbit = 2;
                            stato_decodifica = 0;
                        }
                    } else {
                        bits[num_bits & 0xFF].value = 0;
                        bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                        num_bits++; newbit = 0; numbit = 1;
                        stato_decodifica = 0;
                    }
                } else if (stato_decodifica == 2) {
                    stato_decodifica = 0;
                    bits[num_bits & 0xFF].value = 0;
                    bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                    num_bits++; newbit = 0; numbit = 1;
                }
            }

            // Decodifica byte (identica)
            while (numbit > 0) {
                switch (stato_decobytes) {
                    case 0:
                        if (newbit == 0) {
                            contatore_zeri++;
                        } else if (contatore_zeri == 10) {
                            stato_decobytes = 1;
                            contatore_bytes = contatore_bits = 0;
                            for (int j=0;j<10;j++) bytes[j]=0;
                            sync_count += 1;
                            last_sync_i = ia;
                        } else {
                            contatore_zeri = 0;
                        }
                        break;

                    case 1:
                        if (contatore_bits < 8) {
                            bytes[contatore_bytes] >>= 1;
                            if (newbit == 1) bytes[contatore_bytes] |= 0x80;
                            contatore_bits++;
                        } else if (newbit == 1) {
                            contatore_bytes++;
                            contatore_bits = 0;
                            if (contatore_bytes >= 10) {
                                uint16_t crc = 0x0;
                                const uint16_t polynomial = 0x1021;
                                for (int b=0; b<10; b++) {
                                    uint8_t byte = bytes[b];
                                    for (int j=0; j<8; j++) {
                                        bool bit = ((byte >> j) & 1) == 1;
                                        bool c15 = ((crc >> 15) & 1) == 1;
                                        crc <<= 1;
                                        if (c15 ^ bit) crc ^= polynomial;
                                        crc &= 0xffff;
                                    }
                                }
                                crc_ok = (crc == 0);
                                if (crc_ok) {
                                    country_code = (uint16_t)((bytes[5] << 2) | (bytes[4] >> 6));
                                    device_code  = ((uint64_t)(bytes[4] & 0x3F) << 32)
                                                 | ((uint64_t)bytes[3] << 24)
                                                 | ((uint64_t)bytes[2] << 16)
                                                 | ((uint64_t)bytes[1] << 8)
                                                 |  (uint64_t)bytes[0];

                                    memcpy((void*)last_sequence, bytes, 10);
                                    last_device_code  = device_code;
                                    last_country_code = country_code;
                                    door_sync_count += 1;
                                    display_sync_count += 1;
                                }
                                contatore_zeri = contatore_bytes = 0;
                                stato_decobytes = 0;
                            }
                        } else {
                            contatore_zeri = contatore_bits = contatore_bytes = 0;
                            stato_decobytes = 0;
                        }
                        break;
                }
                numbit--;
            }
        }
        
        ia++; // prossimo campione
    }
}

// ===== Task su core 1 =====
// === Task sul core 1: init ADC, aggancio IRQ su portante, poi analisi ===
// === Task sul core 1: init ADC, aggancio IRQ su portante, toggle su GPIO21, poi analisi ===
static void rfid_task(void *pvParameters)
{
    // 1) ADC “stile Arduino” + prima conversione
    fadcInit_IDF();
    fadcStart(3);

    // 2) GPIO21 come pin di misura (oscilloscopio)
    gpio_reset_pin(GPIO_NUM_21);
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_21, 0);

    // 3) PWM_PIN (GPIO14) come input + IRQ
    //gpio_set_direction(PWM_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(PWM_PIN, GPIO_FLOATING);
    gpio_set_intr_type(PWM_PIN, GPIO_INTR_POSEDGE);

    // 4) Installa servizio ISR (solo una volta)
    static bool isr_installed = false;
    if (!isr_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
        isr_installed = true;
    }

    // 5) Collega ISR al pin della portante e abilita interrupt
    ESP_ERROR_CHECK(gpio_isr_handler_add(PWM_PIN, pwm_edge_isr, nullptr));
    ESP_ERROR_CHECK(gpio_intr_enable(PWM_PIN));   // <- questa mancava

    // 6) Analisi sul core 1
    media_correlazione_32();

    vTaskDelete(nullptr);
}




void start_rfid_task(void) {
    xTaskCreatePinnedToCore(rfid_task, "RFID_Task", 8192, nullptr, 5, nullptr, 1);
}