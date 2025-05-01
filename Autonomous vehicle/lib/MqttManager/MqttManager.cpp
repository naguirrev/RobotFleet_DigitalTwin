#include "MqttManager.h"
#include <FilesManager.h>  

MqttManager& MqttManager::getInstance() {
    static MqttManager instance;
    return instance;
}

void MqttManager::loadConfig(const char* configPath) {
    JsonDocument doc;
    if (FilesManager::readFileAsJson(doc, configPath)) {
        if (doc["server"].is<const char*>() && 
            doc["port"].is<int>() &&
            doc["user"].is<const char*>() &&
            doc["password"].is<const char*>()) {
            mqttSettings.server = doc["server"].as<const char*>();
            mqttSettings.port = doc["port"].as<int>();
            mqttSettings.user = doc["user"].as<const char*>();
            mqttSettings.password = doc["password"].as<const char*>();
            Serial.println("MQTT configuration loaded successfully.");
        } else {
            Serial.println("JSON missing required keys: 'server', 'port', 'user', and/or 'password'");
        }
    } else {
        Serial.println("Failed to load MQTT configuration!");
    }
}

void MqttManager::connect(const std::vector<const char*>& topics) {
    Serial.print("Connecting to MQTT server: ");
    Serial.println(mqttSettings.server);

    client.setServer(mqttSettings.server, mqttSettings.port);
    client.setCallback(MqttManager::messageReceivedCallback);

    while (!client.connected()) {
        Serial.println("Attempting MQTT connection...");
        Serial.print("Server: ");
        Serial.println(mqttSettings.server);
        Serial.print("Port: ");
        Serial.println(mqttSettings.port);
        Serial.print("User: ");
        Serial.println(mqttSettings.user);
        Serial.print("Password: ");
        Serial.println(mqttSettings.password);

        // Intento de conexión con el servidor MQTT
        if (client.connect("ESP32Client", "walle", "walle3000")) {
            Serial.println("Connected to MQTT server.");
        } else {
            Serial.print("Error connecting to MQTT broker, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        } 
    }

    for(const char* topic : topics) {
        MqttManager::subscribe(topic);
    }
}

void MqttManager::subscribe(const char* topic) {
    if (client.connected()) {
        bool success = client.subscribe(topic);
        Serial.print(success ? "Subscribed to: " : "Failed to subscribe to: ");
        Serial.println(topic);
    } else {
        Serial.println("Client not connected. Cannot subscribe to:");
        Serial.println(topic);
    }
}


void MqttManager::publish(const char* topic, const char* payload) {
    if (client.connected()) {
        client.publish(topic, payload);
        Serial.print("Published to topic: ");
        Serial.println(topic);
    }
}

void MqttManager::loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

void MqttManager::reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");

        if (client.connect("ESP32Client", mqttSettings.user, mqttSettings.password)) {
            Serial.println("Reconnected to MQTT server.");
        } else {
            Serial.print("Failed to connect, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void MqttManager::messageReceivedCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived: ");
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