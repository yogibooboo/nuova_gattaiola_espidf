#include "ota_handlers.h"
#include "comune.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>

static const char *TAG = "OTA_HANDLERS";

// =====================================
// OTA Firmware Handler
// =====================================

esp_err_t ota_firmware_handler(httpd_req_t* req) {
    if (req->method != HTTP_POST) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
        return ESP_FAIL;
    }

    // Partizione di destinazione
    const esp_partition_t* update_part = esp_ota_get_next_update_partition(nullptr);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Nessuna partizione OTA");
        return ESP_FAIL;
    }

    // Controllo Content-Length
    if (req->content_len <= 0 || req->content_len > (int)update_part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Dimensione non valida");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Inizio OTA firmware, dimensione: %d bytes", req->content_len);

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin fallita: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin fallita");
        return ESP_FAIL;
    }

    // Buffer in PSRAM se disponibile, altrimenti heap normale
    const size_t CHUNK = 16 * 1024;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t*)malloc(CHUNK);
        if (!buf) {
            esp_ota_end(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Alloc buffer fallita");
            return ESP_FAIL;
        }
        ESP_LOGW(TAG, "Buffer OTA allocato in heap interno (PSRAM non disponibile)");
    } else {
        ESP_LOGI(TAG, "Buffer OTA allocato in PSRAM");
    }

    int remaining = req->content_len;
    int bytes_written = 0;
    
    while (remaining > 0) {
        int to_read = (remaining > (int)CHUNK) ? (int)CHUNK : remaining;
        int r = httpd_req_recv(req, (char*)buf, to_read);
        
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Timeout ricezione OTA, riprovo...");
                continue;
            }
            ESP_LOGE(TAG, "Errore ricezione dati OTA: %d", r);
            free(buf);
            esp_ota_end(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ricezione fallita");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota, buf, r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write fallita: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_end(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Scrittura OTA fallita");
            return ESP_FAIL;
        }

        remaining -= r;
        bytes_written += r;
        
        // Log progresso ogni 64KB
        if (bytes_written % (64 * 1024) == 0) {
            ESP_LOGI(TAG, "OTA progresso: %d/%d bytes (%.1f%%)", 
                     bytes_written, req->content_len, 
                     100.0f * bytes_written / req->content_len);
        }
        
        // Lascia respirare il WDT e lo stack TCP
        vTaskDelay(1);
    }

    free(buf);

    ESP_LOGI(TAG, "Scrittura OTA completata, validazione...");

    if ((err = esp_ota_end(ota)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end fallita: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validazione OTA fallita");
        return ESP_FAIL;
    }

    if ((err = esp_ota_set_boot_partition(update_part)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition fallita: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition fallita");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA firmware completata con successo, riavvio...");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"OTA firmware completato. Riavvio...\"}");

    // Breve delay per permettere al client di ricevere la risposta
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    
    return ESP_OK; // Non raggiunto mai
}

// =====================================
// OTA SPIFFS Handler  
// =====================================

esp_err_t ota_spiffs_handler(httpd_req_t* req) {
    if (req->method != HTTP_POST) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Metodo non permesso");
        return ESP_FAIL;
    }

    // Trova partizione SPIFFS
    const esp_partition_t* part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if (!part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Partizione SPIFFS non trovata");
        return ESP_FAIL;
    }

    // Controllo dimensione
    if (req->content_len <= 0 || req->content_len > (int)part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Dimensione immagine non valida");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Inizio OTA SPIFFS, dimensione: %d bytes, partizione: %u bytes", 
             req->content_len, part->size);

    // 1) Smonta il filesystem per evitare accessi concorrenti
    ESP_LOGI(TAG, "Smonto SPIFFS...");
    spiffs_unmount();

    // 2) Preparazione buffer
    const size_t CHUNK = 16 * 1024;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t*)malloc(CHUNK);
        if (!buf) {
            spiffs_mount_strict(); // Tenta di rimontare
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Alloc buffer fallita");
            return ESP_FAIL;
        }
        ESP_LOGW(TAG, "Buffer SPIFFS allocato in heap interno");
    } else {
        ESP_LOGI(TAG, "Buffer SPIFFS allocato in PSRAM");
    }

    // 3) Ricezione e scrittura con erase incrementale
    size_t offset = 0;
    int remaining = req->content_len;
    size_t erased_until = 0;
    const size_t ERASE_STEP = 64 * 1024; // Cancella in step da 64KB

    ESP_LOGI(TAG, "Inizio ricezione e scrittura...");

    while (remaining > 0) {
        int to_read = (remaining > (int)CHUNK) ? (int)CHUNK : remaining;
        int r = httpd_req_recv(req, (char*)buf, to_read);
        
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Timeout ricezione SPIFFS, riprovo...");
                continue;
            }
            ESP_LOGE(TAG, "Errore ricezione dati SPIFFS: %d", r);
            free(buf);
            spiffs_mount_strict();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ricezione fallita");
            return ESP_FAIL;
        }

        // Assicura che l'area da scrivere sia cancellata
        size_t need_end = offset + r;
        if (need_end > erased_until) {
            // Calcola quanto cancellare (allineato a 4KB)
            size_t erase_to = ((need_end + 0xFFF) & ~0xFFF);
            if (erase_to > part->size) erase_to = part->size;
            
            // Cancella a step per non bloccare troppo
            while (erased_until < erase_to) {
                size_t step = ERASE_STEP;
                if (erased_until + step > erase_to) step = erase_to - erased_until;
                
                // Allinea start a 4KB
                size_t aligned_start = erased_until & ~0xFFF;
                size_t aligned_len = ((erased_until - aligned_start + step + 0xFFF) & ~0xFFF);
                
                if (aligned_start + aligned_len > part->size) {
                    aligned_len = part->size - aligned_start;
                }
                
                if (esp_partition_erase_range(part, aligned_start, aligned_len) != ESP_OK) {
                    ESP_LOGE(TAG, "Erase fallito a offset 0x%X, len 0x%X", aligned_start, aligned_len);
                    free(buf);
                    spiffs_mount_strict();
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase fallito");
                    return ESP_FAIL;
                }
                
                erased_until = aligned_start + aligned_len;
                
                // Log progresso erase
                ESP_LOGD(TAG, "Erased fino a 0x%X", erased_until);
                
                vTaskDelay(1); // WDT feed
            }
        }

        // Scrivi il chunk
        if (esp_partition_write(part, offset, buf, r) != ESP_OK) {
            ESP_LOGE(TAG, "Scrittura SPIFFS fallita a offset 0x%X", offset);
            free(buf);
            spiffs_mount_strict();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Scrittura SPIFFS fallita");
            return ESP_FAIL;
        }

        offset += r;
        remaining -= r;

        // Log progresso ogni 32KB
        if (offset % (32 * 1024) == 0) {
            ESP_LOGI(TAG, "SPIFFS progresso: %u/%d bytes (%.1f%%)", 
                     offset, req->content_len, 100.0f * offset / req->content_len);
        }

        vTaskDelay(1); // Feed cooperativo
    }

    // 4) Pulizia della "coda" per evitare residui che confondano SPIFFS
    if (offset < part->size) {
        size_t tail_start = (offset + 0xFFF) & ~0xFFF; // Allinea a 4KB
        if (tail_start < part->size) {
            size_t tail_len = part->size - tail_start;
            ESP_LOGI(TAG, "Pulizia coda da 0x%X, lunghezza 0x%X", tail_start, tail_len);
            
            // Erase della coda a step
            size_t done = 0;
            while (done < tail_len) {
                size_t step = ERASE_STEP;
                if (done + step > tail_len) step = tail_len - done;
                
                if (esp_partition_erase_range(part, tail_start + done, step) != ESP_OK) {
                    ESP_LOGW(TAG, "Pulizia coda fallita (non critico)");
                    break;
                }
                done += step;
                vTaskDelay(1);
            }
        }
    }

    free(buf);

    ESP_LOGI(TAG, "OTA SPIFFS completata con successo, riavvio...");

    // 5) Risposta e riavvio
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"SPIFFS aggiornato. Riavvio...\"}");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    
    return ESP_OK; // Non raggiunto mai
}

// =====================================
// Registrazione handler
// =====================================

esp_err_t register_ota_handlers(httpd_handle_t server) {
    if (!server) {
        ESP_LOGE(TAG, "Server handle nullo");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Handler OTA firmware
    httpd_uri_t ota_fw_post = {
        .uri = "/ota_firmware",
        .method = HTTP_POST,
        .handler = ota_firmware_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    ret = httpd_register_uri_handler(server, &ota_fw_post);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Registrazione /ota_firmware fallita: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Handler registrato: POST /ota_firmware");

    // Handler OTA SPIFFS
    httpd_uri_t ota_fs_post = {
        .uri = "/ota_spiffs",
        .method = HTTP_POST,
        .handler = ota_spiffs_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = NULL,
        .supported_subprotocol = NULL
    };
    ret = httpd_register_uri_handler(server, &ota_fs_post);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Registrazione /ota_spiffs fallita: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Handler registrato: POST /ota_spiffs");

    return ESP_OK;
}