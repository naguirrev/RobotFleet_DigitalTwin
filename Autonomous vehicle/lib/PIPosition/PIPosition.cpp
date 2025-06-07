#include "PIPosition.h"
#include <shared.h>
#include <ArduinoJson.h>
#include <FilesManager.h>
#include <PIController.h>

PIPosition::PIPosition() {
    currentPosition = 0;
    setPoint = 0;
    kp = 0;
    ki = 0;
    minSpeed = 0;
    integralError = 0;
}

void PIPosition::loadConfig(const char* configFile, const char* piName) {
    // Load configuration from JSON file
    JsonDocument doc;
    bool result = FilesManager::readFileAsJson(doc, configFile);
    if (result) {
        kp = doc[piName]["kp"].as<float>();
        ki = doc[piName]["ki"].as<float>();
        minSpeed = doc[piName]["minSpeed"].as<float>();
        Serial.print("PI position configuration ");
        Serial.print(piName);
        Serial.println(" loaded");
    } else {
        Serial.println("Failed to load PI position ");
        Serial.print(piName);
        Serial.println(" configuration!");
    }
}


float PIPosition::update() {
    PIControllerResult result = updatePIController(setPoint, currentPosition, integralError, kp, ki);
    //Anti windup (acciones integrales para evitar que se desborde-si se desborda entra en condición error y es muy lento para recuperarse)
    integralError = constrain(result.integralError, -10, 10);

    float speedSetPoint = constrain(result.value + minSpeed*sgn(result.value), -PI, PI);
    // Error que se acepta 0.01 rad en el que no hace control (para que no oscile cerca del punto, porque no va a ser exacto)
    return speedSetPoint;
}