#include "Communication.h"
#include <MqttManager.h>
#include <WifiManager.h>
#include <Arduino.h>

const char *wifiConfigFile = "/wifi.json";
const char *mqttConfigFile = "/mqtt.json";

static const std::vector<const char*> topics = {
    "robot/test"
};


void processMqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("[Custom] Message arrived: ");
    Serial.print("Topic: ");
    Serial.print(topic);
    Serial.print(" Payload: ");
    
    // Convertir el payload en un String para mostrarlo
    String payloadStr = "";
    for (unsigned int i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
    }
    Serial.println(payloadStr);

    // Aquí puedes añadir más lógica, como responder o realizar acciones basadas en el contenido del mensaje
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
        "TaskCommunicationLoop",
        4096,
        NULL,
        1,
        NULL,
        1  
    );
}
