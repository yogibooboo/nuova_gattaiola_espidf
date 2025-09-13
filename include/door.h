#ifndef DOOR_H
#define DOOR_H

// Dichiarazione della funzione per l'inizializzazione del motore e dei sensori
void setup_door();


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
void get_door_status(char* buffer, size_t buffer_size, const char* subcommand);
void get_buffer_extended_status(char* buffer, size_t buffer_size);
size_t get_buffer_window_for_timestamp(time_t center_timestamp, int duration_minutes, 
                                                  EncoderDataExtended* output, size_t max_samples,
                                                  time_t* actual_start_timestamp);

#ifdef __cplusplus
}
#endif

#endif // DOOR_H