#include "MqttManager.h"
#include <FilesManager.h>  

MqttManager& MqttManager::getInstance() {
    static MqttManager instance;
    return instance;
}

void MqttManager::loadConfig(const char* configPath) {
    JsonDocument doc;
    if (FilesManager::readFileAsJson(doc, configPath)) {
        if (doc["server"].is<String>() && 
            doc["port"].is<int>() &&
            doc["user"].is<String>() &&
            doc["password"].is<String>()) {
            mqttSettings.server = doc["server"].as<String>();
            mqttSettings.port = doc["port"].as<int>();
            mqttSettings.user = doc["user"].as<String>();
            mqttSettings.password = doc["password"].as<String>();
            Serial.println("MQTT configuration loaded successfully.");
        } else {
            Serial.println("JSON missing required keys: 'server', 'port', 'user', and/or 'password'");
        }
    } else {
        Serial.println("Failed to load MQTT configuration!");
    }
}

void MqttManager::connect(const std::vector<const char*>& topics, MQTT_CALLBACK_SIGNATURE) {
    Serial.print("Connecting to MQTT server: ");
    Serial.println(mqttSettings.server);

    client.setServer(mqttSettings.server.c_str(), mqttSettings.port);
    client.setCallback(callback);

    while (!client.connected()) {
        Serial.println("Attempting MQTT connection...");

        // Intento de conexión con el servidor MQTT
        Serial.print("Connecting to MQTT broker at ");
        Serial.print(mqttSettings.server);
        Serial.print(":");
        Serial.println(mqttSettings.port);
        Serial.print("User: ");
        Serial.println(mqttSettings.user);
        Serial.print("Password: ");
        Serial.println(mqttSettings.password);
        if (client.connect("ESP32Client", mqttSettings.user.c_str(), mqttSettings.password.c_str())) {
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

        if (client.connect("ESP32Client", mqttSettings.user.c_str(), mqttSettings.password.c_str())) {
            Serial.println("Reconnected to MQTT server.");
        } else {
            Serial.print("Failed to connect, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}