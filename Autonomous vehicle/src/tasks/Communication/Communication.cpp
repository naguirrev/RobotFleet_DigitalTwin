#include "Communication.h"
#include <MqttManager.h>
#include <WifiManager.h>
#include <Arduino.h>
#include <shared.h>

const char *wifiConfigFile = "/wifi.json";
const char *mqttConfigFile = "/mqtt.json";

const char *mqtt_topic_map = "rfm/map";
const char *mqtt_topic_nav_path = "rfm/nav/path";
const char *mqtt_topic_nav_command = "rfm/nav/command";
const char *mqtt_topic_obstacle = "rfm/obstacle";

static const std::vector<const char*> topics = {
    mqtt_topic_map,
    mqtt_topic_nav_path,
    mqtt_topic_nav_command,
    mqtt_topic_obstacle   
};


JsonDocument readTopicAsJson(char message[]) {
    JsonDocument jsonDoc; // Create a JSON document with sufficient size
    DeserializationError error = deserializeJson(jsonDoc, message);

    if (error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        // Return an empty document in case of an error
        return jsonDoc;
    }
    return jsonDoc;
} 


void processMqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("[Communication] Message arrived: ");
    Serial.print("Topic: ");
    Serial.print(topic);
    Serial.print(" Payload: ");
    
    // Convertir el payload en un String para mostrarlo
    String payloadStr = "";
    String topicStr(topic);
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    for (unsigned int i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
    }
    Serial.println(payloadStr);

    if (topicStr == mqtt_topic_nav_path) {
        JsonDocument jDoc = readTopicAsJson(message); 
    
        if (!jDoc.is<JsonArray>()) {
            Serial.println("[MQTT] Invalid JSON: expected array of actions");
            return;
        }
    
        JsonArray actionsArray = jDoc.as<JsonArray>();
    
        for (JsonObject actionObj : actionsArray) {
            if (actionObj["action"].is<const char*>() && actionObj["value"].is<float>()) {
                Action action;
                action.action = String(actionObj["action"].as<const char*>());
                action.value = actionObj["value"].as<float>();
                action.completed = false;
    
                if (xQueueSend(actionQueue, &action, pdMS_TO_TICKS(100)) != pdPASS) {
                    Serial.println("[MQTT] Failed to enqueue action");
                } else {
                    Serial.printf("[MQTT] Action enqueued: %s - %.2f\n", action.action.c_str(), action.value);
                }
            } else {
                Serial.println("[MQTT] Invalid action format inside array");
            }
        }
    }

    if (topicStr == mqtt_topic_nav_command) {
        String command = String(message);
        if (command.equalsIgnoreCase("stop")) {
            Serial.println("[MQTT] Stop command received");
            // Set the emergency stop flag
            if (xSemaphoreTake(emergencyStopMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                EmergencyStop = true;
                xSemaphoreGive(emergencyStopMutex);
            }
        } else {
            Serial.println("[MQTT] Unrecognized command");
        }    
    }
}

/**
 * @brief Task to handle MQTT communication in a loop.
 * 
 * This task runs in an infinite loop, calling the MQTT client's loop method
 * to process incoming messages and maintain the connection. It uses a delay
 * to prevent blocking other tasks.
 * 
 * @param parameter Pointer to task parameters (not used in this implementation).
 */
void CommunicationTask(void* parameter) {
    while (true) {
        MqttManager::getInstance().loop();
        vTaskDelay(pdMS_TO_TICKS(10));  
    }
}

/**
 * @brief Initializes the task communication system.
 * 
 * This function sets up the WiFi connection and MQTT client, and creates a
 * FreeRTOS task for handling MQTT communication. The task runs in a separate
 * core to ensure that it does not block other tasks in the system.
 */
void initCommunication() {

    // Load the wifi configuration file and connect to the WiFi network
    WifiManager::getInstance().loadConfig(wifiConfigFile);
    WifiManager::getInstance().connect();
    // Load the MQTT configuration file and connect to the MQTT broker
    MqttManager::getInstance().loadConfig(mqttConfigFile);
    MqttManager::getInstance().connect(topics, processMqttCallback);

    xTaskCreatePinnedToCore(
        CommunicationTask,      
        "Communication",
        4096,
        NULL,
        1,
        NULL,
        1  
    );
}
