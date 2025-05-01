#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>

struct WifiCredentials {
    const char* ssid;
    const char* password;
};

class WifiManager {
public:
    static WifiManager& getInstance();

    void loadConfig(const char* configPath);
    void connect();

    const WifiCredentials& getCredentials() const;

private:
    WifiManager() = default;                             //Private constructor
    WifiManager(const WifiManager&) = delete;            // It's a singleton, so we delete the copy constructor
    WifiManager& operator=(const WifiManager&) = delete; // Delete the assignment operator

    WifiCredentials credentials;
};
