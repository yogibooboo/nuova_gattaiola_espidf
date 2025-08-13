#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

/*
 * La funzione app_main è il punto di ingresso principale per le applicazioni ESP-IDF.
 * È l'equivalente del setup() e loop() di Arduino combinati.
 */
void app_main(void)
{
    // Stampa il nostro messaggio sulla console seriale.
    printf("Hello, World! from ESP-IDF\n");

    // In ESP-IDF, la funzione app_main non deve mai terminare.
    // Se terminasse, il sistema operativo FreeRTOS pulirebbe il task
    // e potrebbe causare un riavvio del microcontrollore.
    // Un ciclo infinito con un ritardo è un modo comune per mantenere il task attivo.
    while (1) {
        // Mette in pausa questo task per 1000 millisecondi.
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
