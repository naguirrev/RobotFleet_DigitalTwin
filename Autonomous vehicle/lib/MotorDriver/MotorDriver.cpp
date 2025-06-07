#include "MotorDriver.h"
#include <ArduinoJson.h>
#include <FilesManager.h>

#define PWM_FREQ 30000
#define PWM_RESOLUTION 8

MotorDriver::MotorDriver()
    : in1(-1), in2(-1), ena(-1), channel(-1), currentDuty(0), direction(true) {}

void MotorDriver::loadConfig(const char* configPath, const char* motorName) {
    // Load motor configuration from JSON file
    JsonDocument doc;
    if (FilesManager::readFileAsJson(doc, configPath)) {
        if (!doc[motorName]["in1"].isNull() && !doc[motorName]["in2"].isNull() && 
            !doc[motorName]["ena"].isNull() && !doc[motorName]["channel"].isNull()) {

            in1 = doc[motorName]["in1"];
            in2 = doc[motorName]["in2"];
            ena = doc[motorName]["ena"];
            channel = doc[motorName]["channel"];

            Serial.println("Motor configuration loaded successfully.");
        } else {
            Serial.println("JSON missing required keys.");
        }
    } else {
        Serial.print("Failed to load motor ");
        Serial.print(motorName);
        Serial.println("configuration!");
    }
}
void MotorDriver::begin() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    ledcAttachChannel(ena, PWM_FREQ, PWM_RESOLUTION, channel);
}

int MotorDriver::dutyToPWM(float duty) {
    float d = constrain(abs(duty), 0.0f, 100.0f);
    return static_cast<int>((d / 100.0f) * 255.0f); // 8-bit PWM
}

void MotorDriver::setDutyCycle(float dutyCycle) {
    currentDuty = constrain(dutyCycle, -100.0f, 100.0f);
    
    if (currentDuty > 0) {
        // Move forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (currentDuty < 0) {
        // Move backward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        // Stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    int pwmValue = dutyToPWM(currentDuty);
    ledcWrite(ena, pwmValue); 
}

void MotorDriver::stop() {
    setDutyCycle(0);
}
