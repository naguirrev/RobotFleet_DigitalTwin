#pragma once

struct PIControllerResult {
    float value;
    float integralError;
};

PIControllerResult updatePIController(float setPoint, float currentValue, float integralError, float kp, float ki);
