#ifndef CORE1_H
#define CORE1_H

// Dichiarazione delle funzioni per la gestione del RFID
void setup_rfid();
void start_rfid_task();
void core1_task(void *pvParameters);

#endif // CORE1_H