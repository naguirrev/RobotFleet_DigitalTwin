#include <Arduino.h>

// Robot components
struct Robot
{
    String id;
    float wheelRadius;
    Motor leftMotor, rightMotor;
    Encoder leftEncoder, rightEncoder;
    PIspeed leftPIDspeed, rightPIDspeed;
    PIposition leftPIDposition, rightPIDposition;   
};

struct Motor
{
    int in1;                      // Control pin 1 for motor driver
    int in2;                      // Control pin 2 for motor driver
    int ena;                      // Enable pin (PWM control)
    boolean direction;            // Rotation direction (true = forward, false = backward)
    float dutyCycle;              // Percentage (%) of power applied to the motor
};

struct Encoder
{
    int pinA;                      // Encoder signal pin A
    int pinB;                      // Encoder signal pin B
    volatile byte pinALastState;   // Last recorded state of pin A (for edge detection)
    int pulses;                    // Number of pulses counted (encoder position)
    int totalPulses;               // Total pulses counted since power-on
};

struct PIspeed
{
    float kp;                     // Proportional gain
    float ki;                     // Integral gain
    float integralError;          // Accumulated integral error for correction
    
    int coulombFriction;          // Coulomb friction compensation value
    float setPosition;            // Desired position (setpoint)
    float previousError;          // Previous error value (for derivative calculation)
    float previousIntegralOutput; // Previous integral output (for stability)
};

struct PIposition
{
    float kp;                     // Proportional gain
    float ki;                     // Integral gain
    float minSpeed;               // Minimum speed to apply correction
    float integralError;          // Accumulated integral error for correction
};

struct RateLimiter {
    float rateUp;                 // Maximum increment limit (R_up)
    float rateDown;               // Maximum decrement limit (R_down)
    float previousOutput;         // Previous output (y(t-Δt))
    float deltaTime;              // Time interval (Δt)
};


struct PIControllerResult {
    float value;                  // Output value of the PI controller
    float integralError;          // Updated integral error after computation
};


