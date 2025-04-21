#include <Arduino.h>
#include <globals.h>

// Setting PWM properties
#define PWM_FREQ 30000
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8

const float loopTime = 0.01; // The time it takes for one complete loop cycle in seconds

// Robot states
struct Pose
{
    int row;
    int col;
    Orientation orientation;

};
// Robot components
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
    int ppr;
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

    int coulombFriction;          // Coulomb friction compensation value

    float integralError;          // Accumulated integral error for correction
    float previousError;          // Previous error value (for derivative calculation)
    float previousIntegralOutput; // Previous integral output (for stability)
    
    float currentSpeed;
    float setPoint;            // Desired position (setpoint)
};

struct PIposition
{
    float kp;                     // Proportional gain
    float ki;                     // Integral gain
    
    float minSpeed;               // Minimum speed to apply correction
    
    float integralError;          // Accumulated integral error for correction

    float currentPosition;
    float setPoint;
};

struct RateLimiter {
    float rateUp;                 // Maximum increment limit (R_up)
    float rateDown;               // Maximum decrement limit (R_down)
    float previousOutput;         // Previous output (y(t-Δt))
};

struct PIControllerResult {
    float value;                  // Output value of the PI controller
    float integralError;          // Updated integral error after computation
};

struct Robot
{
    String id;
    float wheelRadius;
    Motor leftMotor, rightMotor;
    Encoder leftEncoder, rightEncoder;
    PIspeed leftPIDspeed, rightPIDspeed;
    PIposition leftPIDposition, rightPIDposition;
    RateLimiter leftRateLimiter, rightRateLimiter;   
    Pose pose;
    RobotState state;
    ControlMode mode;
};

//Variables 
extern Robot robot;

//Read config
void initRobot();

//Motors
void motorsInit();
void leftMotorControl();
void rightMotorControl();
void stop();

//Encoders
void encodersInit();
void leftPulsesCounter();
void rightPulsesCounter();
void resetEncodersPulses();

//Odometry
float calculateAngularPosition(int pulsesCount, int ppr);
float calculateLinearPosition(float angularPosition);
float calculateAngularSpeed(int pulsesCount, float timeCount, int ppr);
float calculateLinearSpeed(float angularSpeed);
void updateAngularSpeed(float deltaTime);
void updateAngularPosition();

// PI Control
PIControllerResult updatePIController(float setPoint, float currentValue, float integralError, float kp, float ki);
void updatePIspeedController();
void updatePIpositionController();
float rateLimiter(RateLimiter *rl, float input);
int sgn(float value); 

//Plot
void logRobotControlInfoToSerial();

