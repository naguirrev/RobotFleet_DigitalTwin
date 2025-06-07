#include "ObstacleDetection.h"
#include <shared.h>
#include <UltrasonicSensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <MqttManager.h>

#define TRIG_PIN 17
#define ECHO_PIN 16
#define MIN_DISTANCE_CM 3
#define MAX_DISTANCE_CM 7

UltrasonicSensor sensor(TRIG_PIN, ECHO_PIN);
const char* mqtt_topic_obstacle_pub = "rfm/obstacle";

/**
 * @brief Function to publish obstacle data to the MQTT broker.
 */
void publishObstacle(ObstacleData obstacle) {
    JsonDocument doc;
    doc["x"] = obstacle.x;
    doc["y"] = obstacle.y;
    doc["distance"] = obstacle.distance;

    char buffer[128];
    serializeJson(doc, buffer, sizeof(buffer));
    MqttManager::getInstance().publish(mqtt_topic_obstacle_pub, buffer);
}

/**
 * @brief Task to detect obstacles using a distance sensor and update the obstacle data structure.
 * 
 * This task periodically checks the distance to the nearest obstacle using a distance sensor.
 * If an obstacle is detected within the specified range, it updates the obstacle data structure
 * with the detected distance and placeholder coordinates. The task ensures thread safety by using
 * a semaphore to protect access to the shared obstacle data.
 * 
 * @param pvParameters Pointer to task parameters (not used in this implementation).
 * 
 * The task runs in an infinite loop with a periodic delay of 200 milliseconds. This delay is chosen
 * to balance the frequency of obstacle detection updates and the processing load on the system. 
 * A 200 ms delay ensures that the task runs five times per second, which is typically sufficient 
 * for real-time obstacle detection in many robotic applications.
 */
void obstacleDetectionTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    ObstacleData lastObstacle;

    for (;;) {
        float dist = sensor.getDistanceCM();

        if (dist >= MIN_DISTANCE_CM && dist <= MAX_DISTANCE_CM) {
            Serial.printf("[Obstacle] Detected at %.2f cm\n", dist);

            // Attempt to take the obstacleMutex semaphore, waiting up to 10 milliseconds
            if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Update the obstacle structure with the detected distance and placeholder coordinates
                obstacle.distance = dist;
                obstacle.x = 0; // Placeholder for the current X location (use current location in real code)
                obstacle.y = 0; // Placeholder for the current Y location
           
               // Release the semaphore after updating the obstacle data
                xSemaphoreGive(obstacleMutex);
            }

         
            // Publish the obstacle data to the MQTT broker
            publishObstacle(obstacle);

        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
    }
}

/**
 * @brief Initializes the obstacle detection system.
 * 
 * This function initializes the ultrasonic sensor and creates a FreeRTOS task for obstacle detection.
 * The task runs in a separate core to ensure that it does not block other tasks in the system.
 */
void initObstacleDetection() {
    sensor.begin();

    xTaskCreatePinnedToCore(
        obstacleDetectionTask, // Task function to execute
        "ObstacleDetection",   // Name of the task
        2048,                  // Stack size in bytes
        NULL,                  // Task parameters (none in this case)
        3,                     // Task priority (medium priority)
        NULL,                  // Task handle (not storing it)
        1                      // Core to run the task on (core 1)
    );
}
