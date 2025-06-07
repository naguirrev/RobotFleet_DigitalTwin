#include "Telemetry.h"
#include <shared.h>
#include <MqttManager.h>
#include <ArduinoJson.h>
#include <Arduino.h>

#define TELEMETRY_PERIOD_MS 200 

/**
 * @brief Task to publish telemetry data to the MQTT broker.
 * 
 * This task periodically collects telemetry data from various sensors and publishes it
 * to the MQTT broker. The data includes motor speed, angular position, navigation order,
 * and localization data. The task ensures thread safety by using semaphores to protect
 * access to shared data structures.
 * 
 * @param pvParameters Pointer to task parameters (not used in this implementation).
 */
void telemetryTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        MotorSpeed speedCopy;
        AngularPosition angleCopy;
        LocalizationData locationCopy;

        
        if (xSemaphoreTake(motorSpeedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            speedCopy = motorSpeed;
            xSemaphoreGive(motorSpeedMutex);
        }

        if (xSemaphoreTake(angularPosMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            angleCopy = angularPos;
            xSemaphoreGive(angularPosMutex);
        }

        if (xSemaphoreTake(localizationMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            locationCopy = localization;
            xSemaphoreGive(localizationMutex);
        }

        // Empaquetar la información como JSON
        JsonDocument doc;
        doc["speed"]["left"] = speedCopy.left;
        doc["speed"]["right"] = speedCopy.right;

        doc["angle"]["left"] = angleCopy.left;
        doc["angle"]["right"] = angleCopy.right;


        doc["position"]["x"] = locationCopy.row;
        doc["position"]["y"] = locationCopy.col;

        char jsonBuffer[256];
        serializeJson(doc, jsonBuffer);

        // Publicar al topic MQTT
        MqttManager::getInstance().publish("rfm/telemetry", jsonBuffer);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

void initTelemetry() {

    // Create the localization task on core 1
    xTaskCreatePinnedToCore(
        telemetryTask,   // Task function
        "Telemetry",     // Name of the task
        2048,               // Stack size in bytes
        NULL,               // Task input parameter
        1,                  // Task priority
        NULL,               // Task handle
        1                   // Core ID (1 for core 1)
    );
}