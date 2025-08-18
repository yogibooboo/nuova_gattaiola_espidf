// src/buffers_psram.cpp
#include "buffers_psram.h"
#include <string.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static const char* TAG = "BUFPSRAM";

// Struttura generica di un pool
typedef struct {
    const char* name;
    uint8_t**   slots;       // puntatori ai blocchi
    bool*       in_use;      // bitmap uso
    size_t      slot_size;
    int         count;
    SemaphoreHandle_t mutex; // protezione bitmap
    SemaphoreHandle_t sem;   // counting: #slot liberi
} bufpool_t;

// --- Dichiarazioni dei tre pool ---
static uint8_t* g_file_slots[FILE_CHUNK_COUNT];
static bool     g_file_used[FILE_CHUNK_COUNT];
static bufpool_t g_file_pool = {
    .name = "FILE",
    .slots = g_file_slots,
    .in_use = g_file_used,
    .slot_size = FILE_CHUNK_SIZE,
    .count = FILE_CHUNK_COUNT,
    .mutex = nullptr,
    .sem = nullptr
};

static uint8_t* g_post_slots[POST_BODY_COUNT];
static bool     g_post_used[POST_BODY_COUNT];
static bufpool_t g_post_pool = {
    .name = "POST",
    .slots = g_post_slots,
    .in_use = g_post_used,
    .slot_size = POST_BODY_SIZE,
    .count = POST_BODY_COUNT,
    .mutex = nullptr,
    .sem = nullptr
};

static uint8_t* g_ws_slots[WS_FRAME_COUNT];
static bool     g_ws_used[WS_FRAME_COUNT];
static bufpool_t g_ws_pool = {
    .name = "WS",
    .slots = g_ws_slots,
    .in_use = g_ws_used,
    .slot_size = WS_FRAME_SIZE,
    .count = WS_FRAME_COUNT,
    .mutex = nullptr,
    .sem = nullptr
};

static void pool_init(bufpool_t* p) {
    p->mutex = xSemaphoreCreateMutex();
    p->sem   = xSemaphoreCreateCounting(p->count, p->count);
    for (int i = 0; i < p->count; ++i) {
        p->slots[i] = (uint8_t*) heap_caps_malloc(p->slot_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!p->slots[i]) {
            ESP_LOGE(TAG, "%s: alloc fallita per slot %d (%u B)", p->name, i, (unsigned)p->slot_size);
            // In casi estremi potresti fallbackare in DRAM:
            // p->slots[i] = (uint8_t*) heap_caps_malloc(p->slot_size, MALLOC_CAP_8BIT);
        }
        p->in_use[i] = false;
    }
    ESP_LOGI(TAG, "%s: %d slot x %u B allocati in PSRAM", p->name, p->count, (unsigned)p->slot_size);
}

void buffers_psram_init(void) {
    static bool inited = false;
    if (inited) return;
    inited = true;
    pool_init(&g_file_pool);
    pool_init(&g_post_pool);
    pool_init(&g_ws_pool);
}

static void* pool_get(bufpool_t* p, uint32_t timeout_ms) {
    if (!p->sem || !p->mutex) return nullptr;
    if (xSemaphoreTake(p->sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return nullptr; // timeout: nessuno slot libero
    }
    void* ret = nullptr;
    xSemaphoreTake(p->mutex, portMAX_DELAY);
    for (int i = 0; i < p->count; ++i) {
        if (!p->in_use[i] && p->slots[i]) {
            p->in_use[i] = true;
            ret = p->slots[i];
            break;
        }
    }
    xSemaphoreGive(p->mutex);
    if (!ret) {
        // non dovrebbe accadere (sem garantisce disponibilità)
        xSemaphoreGive(p->sem);
    }
    return ret;
}

static void pool_put(bufpool_t* p, void* ptr) {
    if (!ptr || !p->mutex || !p->sem) return;
    bool released = false;
    xSemaphoreTake(p->mutex, portMAX_DELAY);
    for (int i = 0; i < p->count; ++i) {
        if (p->slots[i] == ptr) {
            if (p->in_use[i]) {
                p->in_use[i] = false;
                released = true;
            }
            break;
        }
    }
    xSemaphoreGive(p->mutex);
    if (released) {
        xSemaphoreGive(p->sem);
    } else {
        ESP_LOGW(TAG, "%s: put di puntatore non appartenente al pool (%p)", p->name, ptr);
    }
}

// API specifiche
void* buf_get_file_chunk(uint32_t timeout_ms) { return pool_get(&g_file_pool, timeout_ms); }
void  buf_put_file_chunk(void* p)             { pool_put(&g_file_pool, p); }

void* buf_get_post_body(uint32_t timeout_ms)  { return pool_get(&g_post_pool, timeout_ms); }
void  buf_put_post_body(void* p)              { pool_put(&g_post_pool, p); }

void* buf_get_ws_frame(uint32_t timeout_ms)   { return pool_get(&g_ws_pool, timeout_ms); }
void  buf_put_ws_frame(void* p)               { pool_put(&g_ws_pool, p); }
