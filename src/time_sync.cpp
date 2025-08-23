#include "time_sync.h"

#include <string.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_sntp.h"   // wrapper IDF per le API esp_sntp_*
#include "esp_system.h"

static const char* TAG = "TIME_SYNC";

// Server NTP cablati (senza DHCP)
static const char* kServers[] = {
    "pool.ntp.org",
    "it.pool.ntp.org",
    "time.google.com"
};
static constexpr int kServerCount = sizeof(kServers) / sizeof(kServers[0]);

static volatile bool   s_started   = false;
static volatile bool   s_valid     = false;
static volatile time_t s_last_sync = 0;

// Callback chiamata da SNTP quando l’ora viene aggiornata
static void time_sync_notification_cb(struct timeval* tv)
{
    (void)tv;
    time_t now = 0;
    time(&now);
    s_last_sync = now;
    s_valid = true;
    ESP_LOGI(TAG, "Ora sincronizzata. Epoch=%lld", (long long)now);
}

extern "C" esp_err_t time_sync_start(void)
{
    if (s_started) {
        ESP_LOGD(TAG, "SNTP già avviato");
        return ESP_OK;
    }

    // Modalità polling, allineamento immediato ("jump")
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);

    // Server cablati
    for (int i = 0; i < kServerCount; ++i) {
        esp_sntp_setservername(i, kServers[i]);
    }

    // Callback di notifica
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    // Intervallo di sync: default ~1h; se vuoi, abilita questa riga:
    // sntp_set_sync_interval(3600 * 1000UL);

    // Avvio
    esp_sntp_init();
    s_started = true;
    ESP_LOGI(TAG, "SNTP avviato (jump). Server[0]=%s", kServers[0]);
    return ESP_OK;
}

extern "C" void time_sync_stop(void)
{
    if (!s_started) return;
    esp_sntp_stop();
    s_started = false;
    ESP_LOGI(TAG, "SNTP fermato");
}

extern "C" void time_sync_setup_timezone(void)
{
    // Europe/Rome: CET-1CEST-2, cambio ora: ultima dom. di Mar/Oct
    setenv("TZ", "CET-1CEST-2,M3.5.0/2,M10.5.0/3", 1);
    tzset();
    ESP_LOGI(TAG, "Timezone impostata: Europe/Rome (CET/CEST)");
}

extern "C" bool time_sync_is_valid(void)
{
    // "Valido" se almeno un sync e epoch sensato (> 2019-01-01)
    const time_t MIN_VALID = 1546300800;
    return s_valid && (s_last_sync > MIN_VALID);
}

extern "C" time_t time_sync_last_epoch(void)
{
    return s_last_sync;
}

extern "C" esp_err_t time_sync_force(void)
{
    // Restart semplice del client SNTP
    time_sync_stop();
    return time_sync_start();
}

extern "C" const char* time_sync_server_used(void)
{
    const char* s0 = esp_sntp_getservername(0);
    return s0 ? s0 : kServers[0];
}
