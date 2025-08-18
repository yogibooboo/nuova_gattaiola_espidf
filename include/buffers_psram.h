// include/buffers_psram.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

// --- Taglie e quantità (puoi ritoccarle qui) ---
#ifndef FILE_CHUNK_SIZE
#define FILE_CHUNK_SIZE   2048   // chunk per servire file statici
#endif
#ifndef FILE_CHUNK_COUNT
#define FILE_CHUNK_COUNT  2      // 2 richieste GET contemporanee
#endif

#ifndef POST_BODY_SIZE
#define POST_BODY_SIZE    (16*1024) // payload massimo per /config_data POST
#endif
#ifndef POST_BODY_COUNT
#define POST_BODY_COUNT   1         // una POST per volta
#endif

#ifndef WS_FRAME_SIZE
#define WS_FRAME_SIZE     1024   // frame testo max gestito lato server
#endif
#ifndef WS_FRAME_COUNT
#define WS_FRAME_COUNT    4      // fino a 3–4 client WS contemporanei
#endif

// Inizializza il pool (alloca in PSRAM una tantum)
void buffers_psram_init(void);

// Ottieni/rilascia un buffer per FILE
void* buf_get_file_chunk(uint32_t timeout_ms);
void  buf_put_file_chunk(void* p);

// Ottieni/rilascia un buffer per POST body
void* buf_get_post_body(uint32_t timeout_ms);
void  buf_put_post_body(void* p);

// Ottieni/rilascia un buffer per WS frame
void* buf_get_ws_frame(uint32_t timeout_ms);
void  buf_put_ws_frame(void* p);

#ifdef __cplusplus
}
#endif
