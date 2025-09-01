// src/main.cpp
#include "comune.h"
#include "wifi.h"
#include "console.h"
#include "time_sync.h"

#include <esp_log.h>
#include <esp_system.h>
#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gptimer.h>
#include <ctime>   // time_t, localtime_r, strftime
#include <inttypes.h>
#include <cstdio>  // <-- aggiunto per printf
#include "core1.h"

// --- opzionale: wrapper SPIFFS che abbiamo definito in comune.cpp ---
extern "C" esp_err_t spiffs_mount_boot(void);
extern "C" esp_err_t spiffs_get_info(size_t* total, size_t* used);

// ========================= RMT v2: Portante ~134.2 kHz =========================
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "esp_check.h"

extern "C" void door_task(void *pv);

// Dichiarazioni per il logging unificato
extern "C" void unified_log_raw(const char* fmt, ...);

static const char* TAG = "MAIN_APP";
static const char* TAG_RMT = "RMT134";

static rmt_channel_handle_t s_rmt_chan = nullptr;
static rmt_encoder_handle_t s_copy_encoder = nullptr;

// 40 MHz => 25 ns per tick
static constexpr uint32_t RMT_RES_HZ   = 40'000'000;
static constexpr uint32_t PERIOD_TICKS = 298;                    // 7.45 us
static constexpr uint16_t HALF_TICKS   = PERIOD_TICKS / 2;       // 149

volatile uint16_t lastRawAngle = 0;
volatile uint16_t lastMagnitude = 0;
volatile uint16_t lastCorrectedAngle = 0;

// ---------- FDX-B test sequence (128 bit), identica ad Arduino ----------
static const uint8_t fdx_b_sequence[128] = {
    0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,1,
    0,0,1,0,0,1,0,0,1,1,1,1,0,0,1,1,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,
    1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,1,1,1,0,0,0,1,1,0,0,1,
    0,0,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1,1
};

// Avvia la portante in loop infinito (duty 50%)
static esp_err_t pwm134_start(gpio_num_t pin = (gpio_num_t)F134KHZ)
{
    if (s_rmt_chan) {
        ESP_LOGI(TAG_RMT, "Carrier giÃ  avviata");
        return ESP_OK;
    }

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags = { .invert_out = 0, .with_dma = 0 },
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &s_rmt_chan), TAG_RMT, "tx_channel");

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&enc_cfg, &s_copy_encoder), TAG_RMT, "copy_encoder");

    ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_chan), TAG_RMT, "enable");

    static rmt_symbol_word_t sym;
    sym.level0    = 1;  sym.duration0 = HALF_TICKS;  // HIGH
    sym.level1    = 0;  sym.duration1 = HALF_TICKS;  // LOW

    rmt_transmit_config_t tx_trans_cfg = {
        .loop_count = -1,          // infinito
        .flags = { .eot_level = 0 }
    };

    ESP_RETURN_ON_ERROR(
        rmt_transmit(s_rmt_chan, s_copy_encoder, &sym, sizeof(sym), &tx_trans_cfg),
        TAG_RMT, "transmit"
    );

    ESP_LOGI(TAG_RMT, "Carrier avviata su GPIO %d @ ~134.228 kHz (298 tick @ 40MHz)", (int)pin);
    return ESP_OK;
}

// Ferma e rilascia le risorse (se/quando servirÃ )
static void pwm134_stop(void)
{
    if (!s_rmt_chan) return;

    // Se stavi trasmettendo in loop infinito, non chiamare "wait all done":
    // disabilitare il canale ferma l'output immediatamente.
    (void) rmt_disable(s_rmt_chan);

    if (s_copy_encoder) {
        (void) rmt_del_encoder(s_copy_encoder);
        s_copy_encoder = nullptr;
    }
    (void) rmt_del_channel(s_rmt_chan);
    s_rmt_chan = nullptr;

    ESP_LOGI(TAG_RMT, "Carrier fermata");
}

// ============================================================================

// ---------- FDX-B test (GPTimer) ----------

// In Arduino: timer @ 2.5 MHz, HALF_BIT_TICKS=298 → 119.2 µs
// Qui replichiamo identico: GPTimer @ 2.5 MHz, allarme ogni 298 tick
#define FDX_TIMER_RES_HZ    2500000
#define FDX_HALF_TICKS      298

static gptimer_handle_t s_fdx_timer = nullptr;
static volatile uint32_t s_fdx_bit_index = 0;      // 0..255 (2 half per bit)
static volatile bool     s_fdx_last_half_value = false;

static bool IRAM_ATTR fdx_on_alarm_cb(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*)
{
    // stesso schema Arduino:
    // - ogni interrupt è un "half-bit"
    // - bit = fdx_b_sequence[index/2]
    // - Manchester: la seconda metà dipende dal bit (inverti o ripeti la semionda)
    uint32_t idx = s_fdx_bit_index;
    uint8_t bit = fdx_b_sequence[idx >> 1]; // /2

    bool first_half_value = !s_fdx_last_half_value;
    if ((idx & 1) == 0) {
        // prima metà 
        gpio_set_level(FDX_B_PIN, first_half_value);
    } else {
        // seconda metà 
        bool second_half_value = (bit == 0) ? !first_half_value : first_half_value;
        gpio_set_level(FDX_B_PIN, second_half_value);
        s_fdx_last_half_value = second_half_value;
    }
    s_fdx_bit_index = (idx + 1) & 0xFF; // %256
    return true; // riattiva l'allarme auto-reload
}

static esp_err_t start_fdx_test_if_enabled(void)
{
    if (config.config_01 == 0) {
        ESP_LOGI(TAG, "FDX-B test disabilitato (config_01=0)");
        return ESP_OK;
    }

    // GPIO output
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << FDX_B_PIN);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(FDX_B_PIN, 0);

    // GPTimer @ 2.5 MHz
    gptimer_config_t tcfg = {};
    tcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    tcfg.direction = GPTIMER_COUNT_UP;
    tcfg.resolution_hz = FDX_TIMER_RES_HZ;
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &s_fdx_timer));

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = fdx_on_alarm_cb;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_fdx_timer, &cbs, nullptr));
    ESP_ERROR_CHECK(gptimer_enable(s_fdx_timer));

    gptimer_alarm_config_t acfg = {};
    acfg.reload_count = 0;
    acfg.alarm_count = FDX_HALF_TICKS; // 298 tick @2.5MHz = 119.2 µs
    acfg.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_fdx_timer, &acfg));
    ESP_ERROR_CHECK(gptimer_start(s_fdx_timer));

    ESP_LOGI(TAG, "FDX-B test attivo (half-bit=%u tick @ %u Hz)", FDX_HALF_TICKS, FDX_TIMER_RES_HZ);
    return ESP_OK;
}

// --- implementazione task ---
void print_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000);

    uint32_t last_i_interrupt = 0;
    uint32_t freq = 0;

    for (;;) {
        // Frequenza di campionamento "effettiva", come nel codice Arduino
        freq = i_interrupt - last_i_interrupt;
        last_i_interrupt = i_interrupt;

        // Timestamp locale "umano"
        time_t now = 0;
        time(&now);
        struct tm lt;
        localtime_r(&now, &lt);
        char time_str[20];
        strftime(time_str, sizeof(time_str), "%H:%M:%S", &lt);

        // Copia sicura dell'ultima sequenza (evita tearing durante la stampa)
        uint8_t seq[10];
        for (int i = 0; i < 10; ++i) seq[i] = last_sequence[i];

        // MODIFICA PRINCIPALE: Usa unified_log_raw invece di std::printf
        unified_log_raw(
            "[%s] Sync: %u, OK: %u, Last Seq: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X], "
            "DC: %llu, CC: %u, diff: %u, freq: %u, a_s: %ld, lastADC: %u,Angle:%u/%u,mag:%u\n",
            time_str,
            (unsigned)sync_count,
            (unsigned)display_sync_count,
            seq[0], seq[1], seq[2], seq[3], seq[4], seq[5], seq[6], seq[7], seq[8], seq[9],
            (unsigned long long)last_device_code,
            (unsigned)last_country_code,
            (unsigned)(i_interrupt - ia),
            (unsigned)freq,
            (long)available_samples,
            (unsigned)datoadc,
            lastRawAngle, 
            lastCorrectedAngle, 
            lastMagnitude
        );

        // Reset contatori "per stampa" come sul vecchio sketch
        sync_count = 0;
        display_sync_count = 0;

        vTaskDelayUntil(&last_wake_time, interval);
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Avvio del sistema Pet Door");

    // --- SPIFFS (wrapper) ---
    if (spiffs_mount_boot() == ESP_OK) {
        size_t total=0, used=0;
        if (spiffs_get_info(&total, &used) == ESP_OK) {
            ESP_LOGI(TAG, "SPIFFS: Totale: %u bytes, Usato: %u bytes", (unsigned)total, (unsigned)used);
        }
    } else {
        ESP_LOGE(TAG, "SPIFFS mount fallito");
    }

    // Carica configurazione
    ESP_ERROR_CHECK(load_config());

    // LED Wi-Fi come prima
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << WIFI_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(WIFI_LED, LED_OFF);

    // Wi-Fi + console
    setup_wifi();
    console_start();

    // Time sync (se presente il modulo)
    time_sync_setup_timezone();     // es. Europe/Rome inside
    ESP_ERROR_CHECK(time_sync_start());

    // === Avvia la portante RFID 134.2 kHz (RMT v2, hardware, jitter ~0) ===
    ESP_ERROR_CHECK(pwm134_start());

    // Avvia FDX-B test (GPTimer) se abilitato da config.config_01
    ESP_ERROR_CHECK(start_fdx_test_if_enabled());

    start_rfid_task();   // crea task su core 1 e avvia il GPTimer per l'ADC

    // Task di stampa sul CORE 0
    xTaskCreatePinnedToCore(print_task, "Print_Task", 4096, nullptr, 1, nullptr, 0);

    // Task porta su CORE 0 (RFID rimane su core 1 dentro start_rfid_task)
    xTaskCreatePinnedToCore(door_task, "Door_Task", 4096, nullptr, 5, nullptr, 0);
}