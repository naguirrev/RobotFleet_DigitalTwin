#pragma once
#include <Arduino.h>

class MotorDriver {
public:
    MotorDriver();
    
    void loadConfig(const char* configPath, const char* motorName);
    void begin();
    void setDutyCycle(float dutyCycle); // -100 to 100 (%)
    void stop();

private:
    uint8_t in1, in2, ena, channel;
    float currentDuty;
    bool direction;

    int dutyToPWM(float duty);
};
