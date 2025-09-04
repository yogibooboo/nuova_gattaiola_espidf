// system_diag.cpp - Diagnostiche complete per ESP32-S3 - ESP-IDF 5.5 compatible
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <esp_image_format.h>
#include <esp_app_format.h>



// =====================================
// 3. DIAGNOSTICA TASK E PERFORMANCE COMPLETA
// =====================================
// Sostituisci la funzione get_task_info nel tuo system_diag.cpp

extern "C" void get_task_info(char* buffer, size_t buffer_size, const char* subcommand) {
    //if (strlen(subcommand) == 0) {
        // === VERSIONE UNIFICATA: LISTA + CPU STATS ===
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        TaskStatus_t* task_array = (TaskStatus_t*)malloc(task_count * sizeof(TaskStatus_t));
        
        if (!task_array) {
            snprintf(buffer, buffer_size, "Errore: impossibile allocare memoria per task info\n");
            return;
        }
        
        uint32_t total_runtime = 0;
        UBaseType_t actual_count = uxTaskGetSystemState(task_array, task_count, &total_runtime);
        
        int pos = snprintf(buffer, buffer_size,
            "=== TASK ATTIVI (%u) + CPU STATS ===\n"
            "Nome            Prio Core Stack Stato  CPU%%    Tempo Abs\n"
            "------------------------------------------------------\n",
            actual_count
        );
        
        // Ordina per utilizzo CPU decrescente
        for (int i = 0; i < actual_count - 1; i++) {
            for (int j = i + 1; j < actual_count; j++) {
                if (task_array[i].ulRunTimeCounter < task_array[j].ulRunTimeCounter) {
                    TaskStatus_t temp = task_array[i];
                    task_array[i] = task_array[j];
                    task_array[j] = temp;
                }
            }
        }
        
        for (int i = 0; i < actual_count && pos < buffer_size - 200; i++) {
            const char* state_str;
            switch (task_array[i].eCurrentState) {
                case eRunning: state_str = "RUN"; break;
                case eReady: state_str = "RDY"; break;
                case eBlocked: state_str = "BLK"; break;
                case eSuspended: state_str = "SUS"; break;
                case eDeleted: state_str = "DEL"; break;
                default: state_str = "???"; break;
            }
            
            // Calcola percentuale CPU
            float cpu_percent = 0.0f;
            if (total_runtime > 0) {
                cpu_percent = (float)task_array[i].ulRunTimeCounter * 100.0f / total_runtime;
            }
            
            // Core ID - gestisci il valore speciale per task non pinnati
            char core_str[12];
            if (task_array[i].xCoreID == 2147483647) {
                strcpy(core_str, "ANY");
            } else {
                snprintf(core_str, sizeof(core_str), "%u", task_array[i].xCoreID);
            }
            
            pos += snprintf(buffer + pos, buffer_size - pos,
                "%-14s  %2u  %-3s %4lu  %s   %4.1f%% %10lu\n",
                task_array[i].pcTaskName,
                task_array[i].uxCurrentPriority,
                core_str,
                (unsigned long)task_array[i].usStackHighWaterMark,
                state_str,
                cpu_percent,
                (unsigned long)task_array[i].ulRunTimeCounter
            );
        }
        
        // === MEMORIA DETTAGLIATA ===
        pos += snprintf(buffer + pos, buffer_size - pos,
            "\nMEMORIA SISTEMA:\n"
            "SRAM interna: %u / %u bytes (%.1f%% libera)\n"
            "PSRAM esterna: %u / %u bytes (%.1f%% libera)\n"
            "DMA capable: %u / %u bytes (%.1f%% libera)\n"
            "\nLEGENDA STATI:\n"
            "RUN=In esecuzione, RDY=Pronto, BLK=Bloccato, SUS=Sospeso\n"
            "ANY=Task non pinnato a core specifico\n",
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
            heap_caps_get_total_size(MALLOC_CAP_INTERNAL) > 0 ?
                (float)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) * 100.0f / heap_caps_get_total_size(MALLOC_CAP_INTERNAL) : 0.0f,
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
            heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
            heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0 ?
                (float)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) * 100.0f / heap_caps_get_total_size(MALLOC_CAP_SPIRAM) : 0.0f,
            heap_caps_get_free_size(MALLOC_CAP_DMA),
            heap_caps_get_total_size(MALLOC_CAP_DMA),
            heap_caps_get_total_size(MALLOC_CAP_DMA) > 0 ?
                (float)heap_caps_get_free_size(MALLOC_CAP_DMA) * 100.0f / heap_caps_get_total_size(MALLOC_CAP_DMA) : 0.0f
        );
        
        free(task_array);
        
    //} 
}

// =====================================
// COMANDO SYSTEM UNIFICATO (info + reset + watchdog)
// =====================================
extern "C" void get_system_unified(char* buffer, size_t buffer_size) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    
    const esp_app_desc_t* app_desc = esp_app_get_description();
    
    // === DETECTION PSRAM REALE ===
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    char psram_info[64];
    if (psram_size > 0) {
        snprintf(psram_info, sizeof(psram_info), "PSRAM: %.1f MB (%.1f MB liberi)", 
                psram_size / (1024.0 * 1024.0), psram_free / (1024.0 * 1024.0));
    } else {
        strcpy(psram_info, "PSRAM: assente");
    }
    
    // === DETECTION FREQUENZA CPU ===
    char cpu_info[32];
    #ifdef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
    snprintf(cpu_info, sizeof(cpu_info), "%d cores a %d MHz", chip_info.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    #else
    snprintf(cpu_info, sizeof(cpu_info), "%d cores", chip_info.cores);
    #endif
    
    // === ULTIMO RESET ===
    esp_reset_reason_t reset_reason = esp_reset_reason();
    const char* reset_str;
    const char* reset_desc;
    const char* reset_action;
    
    switch (reset_reason) {
        case ESP_RST_POWERON:
            reset_str = "POWERON"; reset_desc = "Accensione normale";
            reset_action = "Avvio normale - tutto OK";
            break;
        case ESP_RST_PANIC:
            reset_str = "PANIC"; reset_desc = "Crash/panic sistema";
            reset_action = "ATTENZIONE: Controlla 'system task' per stack overflow";
            break;
        case ESP_RST_TASK_WDT:
            reset_str = "TASK_WDT"; reset_desc = "Watchdog task timeout";
            reset_action = "ATTENZIONE: Task bloccato - controlla loop senza delay";
            break;
        case ESP_RST_BROWNOUT:
            reset_str = "BROWNOUT"; reset_desc = "Tensione insufficiente";
            reset_action = "ATTENZIONE: Verifica alimentazione (min 3.0V)";
            break;
        case ESP_RST_INT_WDT:
            reset_str = "INT_WDT"; reset_desc = "Interrupt watchdog timeout";
            reset_action = "ATTENZIONE: Interrupt troppo lungo o disabilitato";
            break;
        default:
            reset_str = "OTHER"; reset_desc = "Altro motivo";
            reset_action = "Consultare documentazione ESP-IDF";
            break;
    }
    
    // === MEMORIA BREAKDOWN ===
    size_t sram_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t sram_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    
    int pos = snprintf(buffer, buffer_size,
        "=== PANORAMICA SISTEMA ===\n"
        "Chip: ESP32-S3 rev %d\n"
        "CPU: %s\n"
        "Flash: %.1f MB | %s\n"
        "\n"
        "FIRMWARE:\n"
        "Nome: %s v%s\n"
        "Build: %s %s (IDF %s)\n"
        "Uptime: %llu sec (%.1f giorni)\n"
        "\n"
        "ULTIMO RESET:\n"
        "Tipo: %s (%s)\n"
        "Azione: %s\n"
        "\n"
        "MEMORIA:\n"
        "SRAM: %.1f KB liberi / %.1f KB (%.1f%% libera)\n"
        "PSRAM: %.1f MB liberi / %.1f MB (%.1f%% libera)\n"
        "Heap totale: %.1f KB | Largest block: %.1f KB\n"
        "\n"
        "WATCHDOG:\n"
        "Task WDT: ATTIVO (timeout %d sec)\n"
        "Task corrente: %s (prio %u, stack %u words)\n",
        
        // Sistema base
        chip_info.revision,
        cpu_info,
        flash_size / (1024.0 * 1024.0), psram_info,
        
        // Firmware
        app_desc->project_name, app_desc->version,
        app_desc->date, app_desc->time, app_desc->idf_ver,
        esp_timer_get_time() / 1000000,
        (float)(esp_timer_get_time() / 1000000) / (24.0 * 3600.0),
        
        // Reset
        reset_str, reset_desc, reset_action,
        
        // Memoria
        sram_free / 1024.0, sram_total / 1024.0,
        sram_total > 0 ? (float)sram_free * 100.0f / sram_total : 0.0f,
        psram_free / (1024.0 * 1024.0), psram_size / (1024.0 * 1024.0),
        psram_size > 0 ? (float)psram_free * 100.0f / psram_size : 0.0f,
        esp_get_free_heap_size() / 1024.0,
        heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT) / 1024.0,
        
        // Watchdog
        CONFIG_ESP_TASK_WDT_TIMEOUT_S,
        pcTaskGetName(NULL), uxTaskPriorityGet(NULL), uxTaskGetStackHighWaterMark(NULL)
    );
    
    // Aggiungi note se ci sono stati reset problematici
    if (reset_reason == ESP_RST_PANIC || reset_reason == ESP_RST_TASK_WDT || 
        reset_reason == ESP_RST_BROWNOUT || reset_reason == ESP_RST_INT_WDT) {
        pos += snprintf(buffer + pos, buffer_size - pos,
            "\nSUGGERIMENTI DEBUG:\n"
            "- Usa 'system task' per analisi task dettagliata\n"
            "- Usa 'system flash' per controllo spazio storage\n"
            "- Aggiungi vTaskDelay(1) nei loop lunghi\n"
            "- Controlla stack overflow nella lista task\n"
        );
    }
}


// =====================================
// SISTEMA FLASH CON CALCOLO UTILIZZO
// =====================================
static size_t get_partition_used_size(const esp_partition_t* partition) {
    if (!partition || partition->type != ESP_PARTITION_TYPE_APP) {
        return 0; // Solo per partizioni APP possiamo calcolare l'utilizzo
    }
    
    // Leggi l'header dell'app per determinare la dimensione reale
    esp_image_header_t img_header;
    esp_image_segment_header_t seg_header;
    
    if (esp_partition_read(partition, 0, &img_header, sizeof(img_header)) != ESP_OK) {
        return 0;
    }
    
    // Verifica magic number
    if (img_header.magic != ESP_IMAGE_HEADER_MAGIC) {
        return 0;
    }
    
    size_t total_size = sizeof(img_header);
    size_t offset = sizeof(img_header);
    
    // Leggi tutti i segmenti per calcolare la dimensione totale
    for (int i = 0; i < img_header.segment_count && i < 16; i++) {
        if (esp_partition_read(partition, offset, &seg_header, sizeof(seg_header)) != ESP_OK) {
            break;
        }
        
        offset += sizeof(seg_header) + seg_header.data_len;
        total_size += sizeof(seg_header) + seg_header.data_len;
        
        // Protezione contro header corrotti
        if (seg_header.data_len > partition->size || offset > partition->size) {
            return 0;
        }
    }
    
    return total_size;
}

extern "C" void get_flash_info_extended(char* buffer, size_t buffer_size) {
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* boot = esp_ota_get_boot_partition();
    
    int pos = snprintf(buffer, buffer_size,
        "=== INFO FLASH DETTAGLIATA ===\n"
        "Dimensione totale: %.2f MB\n"
        "Partizione boot: %s\n"
        "Partizione running: %s\n"
        "\n"
        "MAPPA PARTIZIONI CON UTILIZZO:\n"
        "Nome           Tipo    Alloc   Usato   Libero  %% Uso\n"
        "------------------------------------------------------\n",
        flash_size / (1024.0 * 1024.0),
        boot ? boot->label : "N/A",
        running ? running->label : "N/A"
    );
    
    // Calcola spazio totale utilizzato
    size_t total_used = 0;
    size_t total_allocated = 0;
    
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it && pos < buffer_size - 200) {
        const esp_partition_t* part = esp_partition_get(it);
        
        const char* type_str = "DATA";
        if (part->type == ESP_PARTITION_TYPE_APP) type_str = "APP";
        
        // Calcola utilizzo reale
        size_t used_size = 0;
        float usage_percent = 0.0f;
        char usage_info[32] = "N/A";
        
        if (part->type == ESP_PARTITION_TYPE_APP) {
            used_size = get_partition_used_size(part);
            if (used_size > 0) {
                usage_percent = (float)used_size * 100.0f / part->size;
                snprintf(usage_info, sizeof(usage_info), "%.1f%%", usage_percent);
            }
        }
        
        // Indicators
        char indicators[16] = "";
        if (part == running) strcat(indicators, " RUN");
        if (part == boot) strcat(indicators, " BOOT");
        
        pos += snprintf(buffer + pos, buffer_size - pos,
            "%-12s   %-4s   %6.1fK %6.1fK %6.1fK  %s%s\n",
            part->label, type_str,
            part->size / 1024.0,
            used_size / 1024.0,
            (part->size - used_size) / 1024.0,
            usage_info, indicators
        );
        
        total_allocated += part->size;
        total_used += used_size;
        
        it = esp_partition_next(it);
    }
    if (it) esp_partition_iterator_release(it);
    
    // Riassunto finale
    pos += snprintf(buffer + pos, buffer_size - pos,
        "\nRIASSUNTO FLASH:\n"
        "Flash totale: %.1f MB\n"
        "Partizioni allocate: %.1f MB (%.1f%%)\n"
        "Firmware utilizzato: %.1f MB\n"
        "Spazio libero flash: %.1f MB\n",
        flash_size / (1024.0 * 1024.0),
        total_allocated / (1024.0 * 1024.0),
        (float)total_allocated * 100.0f / flash_size,
        total_used / (1024.0 * 1024.0),
        (flash_size - total_allocated) / (1024.0 * 1024.0)
    );
}

// =====================================
// COMANDO UNIFICATO DIAGNOSTICA SISTEMA
// =====================================


// =====================================
// DISPATCHER PRINCIPALE
// =====================================
extern "C" void get_system_diagnostics(char* buffer, size_t buffer_size, const char* subcommand) {
    if (strcmp(subcommand, "task") == 0) {
        get_task_info(buffer, buffer_size, "");
    } else if (strcmp(subcommand, "flash") == 0) {
        get_flash_info_extended(buffer, buffer_size);
    } else {
        // Comando principale unificato (system senza parametri)
        get_system_unified(buffer, buffer_size);
    }
}