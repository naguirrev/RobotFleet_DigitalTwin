#include "Localization.h"
#include <shared.h>
#include <RFIDSensor.h>
#include <Arduino.h>

#define SS_PIN 5    // Slave Select Pin
#define RST_PIN 22  // Reset Pin

RFIDSensor rfidSensor(SS_PIN, RST_PIN);

/**
 * @brief Task to read RFID tags and update the localization data structure.
 * 
 * This task periodically checks for RFID tags and updates the localization data structure
 * with the detected coordinates. The task ensures thread safety by using a semaphore to protect
 * access to the shared localization data.
 * 
 * @param pvParameters Pointer to task parameters (not used in this implementation).
 * 
 * The task runs in an infinite loop with a periodic delay of 200 milliseconds. This delay is chosen
 * to balance the frequency of RFID detection updates and the processing load on the system. 
 * A 200 ms delay ensures that the task runs five times per second, which is typically sufficient 
 * for real-time localization updates in many robotic applications.
 */
void LocalizationTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        int row, col;
        if (rfidSensor.readCoordinateFromTag(row, col)) {
            Serial.printf("[Localization] Detected coordinates: (%d, %d)\n", row, col);

            // Attempt to take the localizationMutex semaphore, waiting up to 10 milliseconds
            if (xSemaphoreTake(localizationMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Update the localization structure with the detected coordinates
                localization.row = row;
                localization.col = col;

                // Release the semaphore after updating the localization data
                xSemaphoreGive(localizationMutex);
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
    }
}

/**
 * @brief Initializes the localization system.
 * 
 * This function initializes the RFID sensor and creates a FreeRTOS task for localization.
 * The task runs in a separate core to ensure that it does not block other tasks in the system.
 */
void initLocalization() {
    rfidSensor.begin();

    // Create the localization task on core 1
    xTaskCreatePinnedToCore(
        LocalizationTask,   // Task function
        "Localization",     // Name of the task
        2048,               // Stack size in bytes
        NULL,               // Task input parameter
        1,                  // Task priority
        NULL,               // Task handle
        1                   // Core ID (1 for core 1)
    );
}
