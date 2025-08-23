// core1.cpp — versione ISR + GPTimer (20 MHz / 298), agganciata all’RMT
#include "core1.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include "esp_private/periph_ctrl.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "hal/adc_hal.h"

// ================== Variabili globali (come le tue) ==================
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
volatile uint16_t datoadc = 0;   // lastADC

// ===== Helper min/max =====
static inline int32_t imax(int32_t a, int32_t b){ return (a>b)?a:b; }
static inline int32_t imin(int32_t a, int32_t b){ return (a<b)?a:b; }

// ===== Fast-ADC stile Arduino (SENS) =====
static inline void fadcStart(uint8_t channel) {
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1U << channel);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
}
static inline bool fadcBusy() {
    return !(bool)SENS.sar_meas1_ctrl2.meas1_done_sar;
}
static inline uint16_t fadcResult() {
    // leggi solo i 12 bit validi (come in Arduino)
    return HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas1_ctrl2, meas1_data_sar) & 0x0FFF;
}

// Init “alla Arduino”, con forzature + prime
static void fadcInit_IDF()
{
    // alimenta SARADC
    periph_module_enable(PERIPH_SARADC_MODULE);

    // Config ufficiale ADC1: 12 bit, CH3 (GPIO4), atten 12 dB (il tuo IDF depreca 11 dB)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);

    // lato digitale: high-Z, nessun pull
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_DISABLE);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_FLOATING);

    // forzature controllo software + prime iniziale (come Arduino)
    SENS.sar_meas1_ctrl2.meas1_start_force = 1;   // controllo da CPU
    SENS.sar_meas1_ctrl2.sar1_en_pad_force = 1;   // selezione pad da CPU
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1U << 3); // CH3=GPIO4
    SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
}

// ====== GPTimer per scandire le conversioni (20 MHz / 298 ≈ 67.114 kHz) ======
static gptimer_handle_t s_adc_timer = nullptr;

// ISR produttore (duplicazione campioni come nel tuo sketch)
void IRAM_ATTR onTimer(void) {
    if (!fadcBusy()) {
        uint16_t s = fadcResult();
        datoadc = s;
        adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
        adc_buffer[i_interrupt & 0x3FFF] = s; i_interrupt += 1;
        fadcStart(3); // CH3 (GPIO4)
    }
}

static bool IRAM_ATTR adc_timer_cb(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
    onTimer();
    return true;
}

// ====== La tua analisi (identica) ======
static void media_correlazione_32() {
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

// ===== Task su core 1: init ADC, timer, poi analisi =====
static void rfid_task(void *pvParameters)
{
    // 1) Init ADC stile Arduino + prime
    fadcInit_IDF();
    fadcStart(3); // prima conversione

    // 2) GPTimer: APB 80 MHz / 4 = 20 MHz; allarme ogni 298 tick (~14.9 us)
    gptimer_config_t tcfg = {};
    tcfg.clk_src       = GPTIMER_CLK_SRC_DEFAULT; // APB 80 MHz
    tcfg.direction     = GPTIMER_COUNT_UP;
    tcfg.resolution_hz = 20000000;                // 20 MHz
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &s_adc_timer));

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = adc_timer_cb;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_adc_timer, &cbs, nullptr));

    gptimer_alarm_config_t alarm = {};
    alarm.reload_count = 0;
    alarm.alarm_count  = 298;                     // 20 MHz / 298 ≈ 67.114 kHz
    alarm.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_adc_timer, &alarm));

    ESP_ERROR_CHECK(gptimer_enable(s_adc_timer));
    ESP_ERROR_CHECK(gptimer_start(s_adc_timer));

    // 3) Analisi sullo stesso core (come volevi)
    media_correlazione_32();

    vTaskDelete(nullptr); // non ritorna
}

void start_rfid_task(void) {
    xTaskCreatePinnedToCore(rfid_task, "RFID_Task", 8192, nullptr, 5, nullptr, 1); // core 1
}
