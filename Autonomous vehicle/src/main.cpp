#include <Arduino.h>
#include <FilesManager.h>
#include <ObstacleDetection/ObstacleDetection.h>
#include <Localization/Localization.h>
#include <Communication/Communication.h>

void setup() {
    Serial.begin(115200);
    FilesManager::initLittleFS();

    initObstacleDetection();
    initLocalization();
    initCommunication();
}

void loop(){}