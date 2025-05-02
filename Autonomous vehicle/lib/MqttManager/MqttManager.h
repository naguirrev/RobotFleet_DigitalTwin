#pragma once

#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

struct MqttSettings {
    String server;
    int port;
    String user;
    String password;
};

class MqttManager {
public:
    static MqttManager& getInstance();

    void loadConfig(const char* configPath);
    void connect(const std::vector<const char*>& topics, MQTT_CALLBACK_SIGNATURE);
    void subscribe(const char* topic);
    void publish(const char* topic, const char* payload);
    void loop();


private:
    MqttManager() = default;                                // Private constructor
    MqttManager(const MqttManager&) = delete;                // Delete copy constructor
    MqttManager& operator=(const MqttManager&) = delete;     // Delete assignment operator

    void reconnect();

    WiFiClient espClient;
    PubSubClient client{espClient};
    MqttSettings mqttSettings;
};
