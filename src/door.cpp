#include "comune.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <string.h>

// =============================================================
//  Porta – Step 1 (scheletro ESP‑IDF, senza stepper & sensori)
//  - Macchina a stati della porta (AUTO / ALWAYS_OPEN / ALWAYS_CLOSED)
//  - Solo SERVO (posizionamento diretto, senza rampa per ora)
//  - Stepper ignorato (placeholder no‑op)
//  - Placeholder per IR/Encoder (da implementare negli step successivi)
//  - Log: manteniamo i punti d’integrazione, ma di default non inviamo
// =============================================================

static const char *TAG = "DOOR";


// -----------------------------
// Stato locale
// -----------------------------
static bool s_door_open = false;                      // stato porta
static TickType_t s_door_timer_start = 0;             // in tick
static bool s_servo_inited = false;                   // LEDC inizializzato?

// -----------------------------
// Helper: init LEDC per SERVO (senza rampa per ora)
// -----------------------------
static void servo_init_once()
{
    if (s_servo_inited) return;

    // Timer 50 Hz (periodo 20 ms), risoluzione 15 bit (puoi adattare)
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK,
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
        .flags = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    s_servo_inited = true;
}

static void servo_set_us(uint32_t pulse_us)
{
    // duty = pulse_us / 20000 * (2^res - 1)
    const uint32_t max_duty = (1u << LEDC_TIMER_14_BIT) - 1u;
    uint32_t duty = (pulse_us * max_duty) / 20000u;
    if (duty > max_duty) duty = max_duty;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

// -----------------------------
// Placeholder Stepper (ignorato)
// -----------------------------
static inline void stepper_start(bool /*direction*/) { /* no‑op */ }
static inline void stepper_stop() { /* no‑op */ }

// -----------------------------
// Helper LED (opzionali)
// -----------------------------
static inline void led_set(gpio_num_t pin, int level)
{
    if (pin == GPIO_NUM_NC) return;
    gpio_set_level(pin, level);
}

// -----------------------------
// Apertura/chiusura porta via SERVO (senza rampa)
// -----------------------------
static void door_open_now()
{
    servo_init_once();
    servo_set_us(config.servo_open_us);
    s_door_open = true;
    s_door_timer_start = xTaskGetTickCount();
    led_set(LED_ROSSO, LED_ON);
    ESP_LOGI(TAG, "Porta APERTA (servo %u us)", config.servo_open_us);
}

static void door_close_now()
{
    servo_init_once();
    servo_set_us(config.servo_closed_us);
    s_door_open = false;
    led_set(LED_ROSSO, LED_OFF);
    ESP_LOGI(TAG, "Porta CHIUSA (servo %u us)", config.servo_closed_us);
}

// -----------------------------
// Inizializzazione GPIO opzionali
// -----------------------------
static void door_gpio_init_once()
{
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
    
    io.pin_bit_mask = 1ULL << LED_BLUE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(LED_BLUE, LED_OFF);
    

    // IR sensor (input, ISR verrà aggiunto nello step successivo)
    
    io.pin_bit_mask = 1ULL << INFRARED;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    
    

    // IR enable (output sempre alto per alimentare il sensore)
    
    io.pin_bit_mask = 1ULL << INFRARED_ENABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(INFRARED_ENABLE, 1); // attiva alimentazione IR 
    
}


// -----------------------------
// Task principale della porta (exportable)
// -----------------------------
extern "C" void door_task(void *pv)
{
    door_gpio_init_once();

    // Stato iniziale coerente con config.door_mode
    switch (config.door_mode) {
        case ALWAYS_OPEN:
            if (!s_door_open) door_open_now();
            break;
        case ALWAYS_CLOSED:
            if (s_door_open) door_close_now();
            else { // assicurati che sia chiusa
                servo_init_once();
                servo_set_us(config.servo_closed_us);
                s_door_open = false;
                led_set(LED_ROSSO, LED_OFF);
            }
            break;
        case AUTO:
        default:
            if (s_door_open) door_close_now();
            else {
                // porta chiusa all’avvio in AUTO
                servo_init_once();
                servo_set_us(config.servo_closed_us);
                s_door_open = false;
                led_set(LED_ROSSO, LED_OFF);
            }
            break;
    }

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); // 100 ms

    while (true) {
        // 1) Leggi modalita’ corrente
        DoorMode mode = config.door_mode; // se in futuro serve protezione, aggiungeremo mutex

        // 2) Reagisci ai cambi modalita’ (transizioni basilari)
        static DoorMode s_last_mode = AUTO; // di default
        if (mode != s_last_mode) {
            ESP_LOGI(TAG, "Cambio modalita’: %d -> %d", (int)s_last_mode, (int)mode);
            if (mode == ALWAYS_OPEN) {
                if (!s_door_open) door_open_now();
            } else if (mode == ALWAYS_CLOSED) {
                if (s_door_open) door_close_now();
                else { servo_init_once(); servo_set_us(config.servo_closed_us); }
            } else { // AUTO
                if (s_door_open) door_close_now();
                else { servo_init_once(); servo_set_us(config.servo_closed_us); }
            }
            s_last_mode = mode;
        }

        // 3) Placeholder rilevamenti (IR / Encoder / RFID …)
        //    Per Step 1 non apriamo/chiudiamo automaticamente in base ai sensori.
        //    Apriremo in Step 2/3 quando integreremo ISR IR e letture encoder/I2C.

        // 4) Timeout automatico in AUTO
        if (mode == AUTO && s_door_open) {
            TickType_t now = xTaskGetTickCount();
            if ((now - s_door_timer_start) >= pdMS_TO_TICKS(config.door_timeout)) {
                ESP_LOGI(TAG, "Timeout porta in AUTO: chiudo");
                door_close_now();
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

// =============================================================
// NOTE FUTURE (Step 2 / Step 3)
// - Step 2: rampa servo (aggiornamento duty ogni 10 ms, durata config.servo_transition_ms)
// - Step 3: ISR su PIN_INFRARED che alza un flag (interrupt‑safe), letture encoder via I2C
// - Facoltativo: wrapper per inviare log/JSON via WS (compat con index.html attuale)
// =============================================================
