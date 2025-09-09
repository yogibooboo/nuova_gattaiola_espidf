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

#ifdef __cplusplus
}
#endif

#endif // DOOR_H