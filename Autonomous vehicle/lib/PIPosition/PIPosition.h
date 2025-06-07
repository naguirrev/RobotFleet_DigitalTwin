#pragma once

class PIPosition {
    public:
        PIPosition();
        void loadConfig(const char* configFile, const char* piName);
        float update();
        
        float currentPosition;          // Current position 
        float setPoint;                 // Desired position (setpoint)

    private:

        float kp;                     // Proportional gain
        float ki;                     // Integral gain
        float minSpeed;               // Minimum speed to apply correction
        float integralError;          // Accumulated integral error for correction
};