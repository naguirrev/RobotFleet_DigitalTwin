#include "WifiManager.h"
#include <FilesManager.h>

WifiManager& WifiManager::getInstance() {
    static WifiManager instance;
    return instance;
}

void WifiManager::loadConfig(const char* configPath) {
    JsonDocument doc;
    if (FilesManager::readFileAsJson(doc, configPath)) {
        if (!doc["ssid"].isNull() && !doc["password"].isNull()) {
            credentials.ssid = strdup(doc["ssid"]);
            credentials.password = strdup(doc["password"]);
            Serial.println("WiFi configuration loaded successfully.");
        } else {
            Serial.println("JSON missing required keys: 'ssid' and/or 'password'");
        }
    } else {
        Serial.println("Failed to load WiFi configuration!");
    }
}

void WifiManager::connect() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(credentials.ssid);

    WiFi.begin(credentials.ssid, credentials.password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
        Serial.print("Connecting to WiFi: ");
        Serial.println(credentials.ssid);
    }
    Serial.println("\nWiFi connected.");
}

const WifiCredentials& WifiManager::getCredentials() const {
    return credentials;
}
