#pragma once
#include <RateLimiter.h>
#include <MotorDriver.h>

class PISpeed {
    public:
        PISpeed(RateLimiter* rl);
        void loadConfig(const char* configFile, const char* piName);
        float update(float deltaTime); // Update the PI controller and return the duty cycle
        
        float currentSpeed;          // Current speed
        float setPoint;              // Desired speed (setpoint)

    private:

        float kp;                     // Proportional gain
        float ki;                     // Integral gain
        int coulombFriction;          // Coulomb friction compensation value
        float integralError;          // Accumulated integral error for correction
        float previousError;          // Previous error value (for derivative calculation)
        float previousIntegralOutput; // Previous integral output (for stability)
        RateLimiter* rateLimiter;     // Pointer to the rate limiter object
};
