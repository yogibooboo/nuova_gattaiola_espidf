// core1.cpp â€" versione corretta con calibrazione ADC
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
uint32_t           num_picchi = 0;
uint32_t           num_distanze = 0;
uint32_t           num_bits = 0;
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
//volatile bool     door_open = false;
//volatile TickType_t door_timer_start = 0;
volatile uint32_t display_sync_count = 0;
volatile uint16_t datoadc = 0;

// ================== Variabili diagnostiche (esposte da media_correlazione_32) ==================
static bool decoder_initialized = false;
static int32_t debug_max_i = 0;
static int32_t debug_min_i = 0;
static int32_t debug_max_i8 = 0;
static int32_t debug_min_i8 = 0;
static int32_t debug_min_i_iniziale = 0;
static int32_t debug_max_i_iniziale = 0;
static int32_t debug_stato = 1;
static int32_t debug_stato_decodifica = 0;
static int32_t debug_contatore_zeri = 0;
static int32_t debug_contatore_bytes = 0;
static int32_t debug_contatore_bits = 0;
static int32_t debug_stato_decobytes = 0;
static int32_t debug_ultima_distanza = 0;
static int32_t debug_newbit = 0;
static int32_t debug_numbit = 0;
static bool debug_newpeak = false;
static uint8_t debug_confro = 0;

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

#include "driver/pulse_cnt.h"

static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan = NULL;

static bool IRAM_ATTR pcnt_watch_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    //pcnt_unit_clear_count(unit);
    
    uint16_t s = SENS.sar_meas1_ctrl2.meas1_data_sar & 0x0FFF;   //lettura misura
    adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
    adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
    
    SENS.sar_meas1_ctrl2.meas1_start_sar = 0;   
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;   //restart misura
    
    return false;
}


// ====== Analisi (identica alla tua) ======
static void media_correlazione_32() {
    // [Il tuo codice di analisi rimane identico]

    if (!decoder_initialized) {
        num_picchi = num_distanze = num_bits = 0;
        for (int j=0;j<10;j++) bytes[j]=0;
        for (int j=0;j<CORR_BUFFER_SIZE;j++){ filt[j]=0; corr[j]=0; peaks[j]=0; dist[j]=0;}
        decoder_initialized = true;
    }

    ia = 32;
    vTaskDelay(10 / portTICK_PERIOD_MS);     //assicura che inizialmente ci siano un pÃ² di campioni

    while (true) {
        available_samples = (int32_t)(i_interrupt - ia);
        if (available_samples <= 10) {
            vTaskDelay(pdMS_TO_TICKS(72));
            continue;
        }
         
        //const int larghezza_finestra=8;
        const int lunghezza_correlazione=32;
        const int soglia_mezzo_bit=25;

        datoadc=adc_buffer[(ia) & 0x3FFF]; // per log

        //if (ia >= 32) {tolto, rimpiazzato con inizializzazione ia a 32 e ritardo iniziale per assicurare i primi 32 campioni
            gpio_set_level(SCOPE_1, 1);
            int32_t sum = 0;
            for(int j = 0; j < 8; j++) {
                sum += adc_buffer[(ia-4+j) & 0x3FFF];
            }
            filt[ia & 255] = sum;

            // â€œCorrelazioneâ€ 
            int32_t sum_corr = 0;
            for(int j = 0; j < 16; j++) {
                sum_corr += filt[(ia-31+j) & 0xFF];  // Primi 16: somma
                sum_corr -= filt[(ia-15+j) & 0xFF];  // Ultimi 16: sottrai  
            }
            corr[(ia-16) & 0xFF] = sum_corr;

            gpio_set_level(SCOPE_1, 0);

            debug_newbit = 2; debug_numbit = 0; debug_newpeak = false;

            // Peak picking alternato
            if (debug_stato == 1) {
                debug_max_i  = imax(corr[(ia-16) & (CORR_BUFFER_SIZE - 1)], debug_max_i);
                debug_max_i8 = imax(corr[(ia-24) & (CORR_BUFFER_SIZE - 1)], debug_max_i8);
                if ((debug_max_i == debug_max_i8) && (debug_max_i != debug_max_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = (int32_t)ia - 24;
                    num_picchi++;
                    debug_stato = -1;
                    debug_min_i = corr[(ia-16) & (CORR_BUFFER_SIZE - 1)];
                    debug_min_i8 = corr[(ia-24) & (CORR_BUFFER_SIZE - 1)];
                    debug_min_i_iniziale = debug_min_i8;
                    debug_newpeak = true;
                }
            } else {
                debug_min_i  = imin(corr[(ia-16) & (CORR_BUFFER_SIZE - 1)], debug_min_i);
                debug_min_i8 = imin(corr[(ia-24) & (CORR_BUFFER_SIZE - 1)], debug_min_i8);
                if ((debug_min_i == debug_min_i8) && (debug_min_i != debug_min_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = (int32_t)ia - 24;
                    num_picchi++;
                    debug_stato = 1;
                    debug_max_i = corr[(ia-16) & (CORR_BUFFER_SIZE - 1)];
                    debug_max_i8 = corr[(ia-24) & (CORR_BUFFER_SIZE - 1)];
                    debug_max_i_iniziale = debug_max_i8;
                    debug_newpeak = true;
                }
            }

            // Distanze/bit
            if (debug_newpeak) {    //tolto  if (num_picchi > 1 && newpeak) { che bloccava in overflow negativo
                debug_ultima_distanza = peaks[(num_picchi - 1) & 0xFF] - peaks[(num_picchi - 2) & 0xFF];
                dist[num_distanze & 0xFF] = debug_ultima_distanza;
                num_distanze++;

                if (debug_stato_decodifica == 0) {
                    if (debug_ultima_distanza >= soglia_mezzo_bit) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                        num_bits++; debug_newbit = 1; debug_numbit = 1;
                    } else {
                        debug_stato_decodifica = 1;
                    }
                } else if (debug_stato_decodifica == 1) {
                    debug_confro = 42;
                    if (bits[(num_bits - 1) & 0xFF].value == 1) debug_confro = 48;

                    if ((debug_ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= debug_confro) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos   = (int32_t)ia - 24 - debug_ultima_distanza;
                        num_bits++; debug_newbit = 1; debug_numbit = 1;
                        debug_stato_decodifica = 2;

                        if ((debug_ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= 52) {
                            bits[num_bits & 0xFF].value = 1;
                            bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                            num_bits++; debug_newbit = 1; debug_numbit = 2;
                            debug_stato_decodifica = 0;
                        }
                    } else {
                        bits[num_bits & 0xFF].value = 0;
                        bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                        num_bits++; debug_newbit = 0; debug_numbit = 1;
                        debug_stato_decodifica = 0;
                    }
                } else if (debug_stato_decodifica == 2) {
                    debug_stato_decodifica = 0;
                    bits[num_bits & 0xFF].value = 0;
                    bits[num_bits & 0xFF].pos   = (int32_t)ia - 24;
                    num_bits++; debug_newbit = 0; debug_numbit = 1;
                }
            }

            // Decodifica byte (identica)
            while (debug_numbit > 0) {
                switch (debug_stato_decobytes) {
                    case 0:
                        if (debug_newbit == 0) {
                            debug_contatore_zeri++;
                        } else if (debug_contatore_zeri == 10) {
                            debug_stato_decobytes = 1;
                            debug_contatore_bytes = debug_contatore_bits = 0;
                            for (int j=0;j<10;j++) bytes[j]=0;
                            sync_count += 1;
                            last_sync_i = ia;
                        } else {
                            debug_contatore_zeri = 0;
                        }
                        break;

                    case 1:
                        if (debug_contatore_bits < 8) {
                            bytes[debug_contatore_bytes] >>= 1;
                            if (debug_newbit == 1) bytes[debug_contatore_bytes] |= 0x80;
                            debug_contatore_bits++;
                        } else if (debug_newbit == 1) {
                            debug_contatore_bytes++;
                            debug_contatore_bits = 0;
                            if (debug_contatore_bytes >= 10) {
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
                                debug_contatore_zeri = debug_contatore_bytes = 0;
                                debug_stato_decobytes = 0;
                            }
                        } else {
                            debug_contatore_zeri = debug_contatore_bits = debug_contatore_bytes = 0;
                            debug_stato_decobytes = 0;
                        }
                        break;
                }
                debug_numbit--;
            }
        //} //if (ia >= 32) tolto
        
        ia++; // prossimo campione
    }
}


// Funzione diagnostica per comando "decod" - accede alle variabili statiche esposte
extern "C" void get_decoder_status(char* buffer, size_t buffer_size, const char* subcommand) {
    if (strlen(subcommand) == 0) {
        // Buffer esterno per ultimo log periodico
        extern EXT_RAM_BSS_ATTR char last_periodic_log[512];
        
        snprintf(buffer, buffer_size,
            "=== STATO DECODER RFID ===\n"
            "Log periodico: %s\n"
            "\n"
            "CONTATORI:\n"
            "  i_interrupt: %u\n"
            "  ia: %u\n"
            "  available_samples: %d\n"
            "\n"
            "STATI DECODER:\n"
            "  max_i: %ld\n"
            "  max_i8: %ld\n" 
            "  min_i: %ld\n"
            "  min_i8: %ld\n"
            "  stato_decodifica: %ld\n"
            "  stato_decobytes: %ld\n"
            "  contatore_bytes: %ld\n"
            "  contatore_bits: %ld\n"
            "  contatore_zeri: %ld\n"
            "\n"
            "BUFFER:\n"
            "  num_picchi: %u\n"
            "  num_distanze: %u\n"
            "  num_bits: %u\n"
            "  filt[0]: %ld\n"
            "  corr[0]: %ld\n"
            "  peaks[0]: %ld\n"
            "  dist[0]: %ld\n"
            "  datoadc: %u\n"
            "\n"
            "Subcomandi: filt, corr, peaks, dist, adc",
            
            last_periodic_log,
            (unsigned)i_interrupt,
            (unsigned)ia,
            (int)available_samples,
            debug_max_i,
            debug_max_i8,
            debug_min_i,
            debug_min_i8,
            debug_stato_decodifica,
            debug_stato_decobytes,
            debug_contatore_bytes,
            debug_contatore_bits,
            debug_contatore_zeri,
            (unsigned)num_picchi,
            (unsigned)num_distanze,
            (unsigned)num_bits,
            filt[0],
            corr[0],
            peaks[0],
            dist[0],
            (unsigned)datoadc
        );
    } else {
        // Subcomandi futuri
        snprintf(buffer, buffer_size,
            "Subcomando '%s' non ancora implementato.\n"
            "Subcomandi disponibili (futuri): filt, corr, peaks, dist, adc",
            subcommand
        );
    }
}
// ===== Task su core 1 =====
// === Task sul core 1: init ADC, aggancio IRQ su portante, poi analisi ===
static void rfid_task(void *pvParameters)
{
    // 1) ADC "stile Arduino" + prima conversione
    fadcInit_IDF();
    fadcStart(3);

    // 2) GPIO21 come pin di misura (oscilloscopio)
    gpio_reset_pin(SCOPE_1);
    gpio_set_direction(SCOPE_1, GPIO_MODE_OUTPUT);
    gpio_set_level(SCOPE_1, 0);

    // 2) GPIO11 come secondo pin di misura (oscilloscopio)
    gpio_reset_pin(SCOPE_2);
    gpio_set_direction(SCOPE_2, GPIO_MODE_OUTPUT);
    gpio_set_level(SCOPE_2, 0);

    // 3) Setup PCNT per divisore hardware /2
    pcnt_unit_config_t unit_config = {
        .low_limit = -2,
        .high_limit = 2,
        .intr_priority = 0,
        .flags = {},
    };
 
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = F134KHZ,
        .level_gpio_num = -1,
        .flags = {},
    };
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // 4) Configura per contare sui fronti di salita
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, 
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    // 5) Registra callback per watch point
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_watch_callback,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    // 6) Abilita e avvia PCNT con watch point a 2
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 2));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // 7) Analisi sul core 1
    media_correlazione_32();

    // Cleanup (mai raggiunto)
    pcnt_unit_stop(pcnt_unit);
    pcnt_del_unit(pcnt_unit);
    vTaskDelete(nullptr);
}


void start_rfid_task(void) {
    xTaskCreatePinnedToCore(rfid_task, "RFID_Task", 8192, nullptr, 5, nullptr, 1);
}