#include "PISpeed.h"
#include <ArduinoJson.h>
#include <shared.h>
#include <FilesManager.h>
#include <PIController.h>

PISpeed::PISpeed(RateLimiter* rl)
    : rateLimiter(rl), kp(-1), ki(-1), coulombFriction(-1), integralError(0), previousError(0), previousIntegralOutput(0), currentSpeed(0), setPoint(0) {}


void PISpeed::loadConfig(const char *configFile, const char *piName)
{
    // Load the configuration file
    JsonDocument doc;
    bool result = FilesManager::readFileAsJson(doc, configFile);
    if (result) {
        // Load PI parameters from the JSON file
        kp = doc[piName]["kp"].as<float>();
        ki = doc[piName]["ki"].as<float>();
        coulombFriction = doc[piName]["coulombFriction"].as<int>();
    } else {
        Serial.println("Failed to load PI configuration!");
    }
}

float PISpeed::update(float deltaTime) {
    
    float setPointLimited = rateLimiter->updateRateLimiter(setPoint, deltaTime); // Limit the slope of the setpoint

    PIControllerResult result = updatePIController(setPointLimited, currentSpeed, integralError, kp, ki);
    
    float dutyCycle = constrain(result.value + coulombFriction*sgn(setPoint),-100,100);

    integralError = result.integralError;
    if(currentSpeed == 0) {
        integralError = 0; // Reset integral error if speed is zero
    }

    return dutyCycle;
}

