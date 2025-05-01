#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <motors.h>

Robot robot;
const char *configFile = "/robot.json";
void initRobot() {    
    JsonDocument doc;  
    bool result = readFileAsJson(doc, configFile);
    if(result){
  
    // Load data at Robot struct
    robot.id = doc["id"].as<String>();
    robot.wheelRadius = doc["wheelRadius"].as<float>();
    robot.wheelBase = doc["wheelBase"].as<float>();
  
    // Motors
    robot.leftMotor = { doc["motorPin"]["left"]["in1"], doc["motorPin"]["left"]["in2"], doc["motorPin"]["left"]["ena"], 0, 0 };
    robot.rightMotor = { doc["motorPin"]["right"]["in1"], doc["motorPin"]["right"]["in2"], doc["motorPin"]["right"]["ena"], 0, 0 };
  
    // Encoders
    robot.leftEncoder = { doc["encoder"]["left"]["ppr"], doc["encoder"]["left"]["pinA"], doc["encoder"]["left"]["pinB"], 0, 0, 0 };
    robot.rightEncoder = { doc["encoder"]["right"]["ppr"], doc["encoder"]["right"]["pinA"], doc["encoder"]["right"]["pinB"], 0, 0, 0 };
  
    // PID Speed
    robot.leftPIDspeed = { doc["pid"]["speed"]["left"]["kp"], doc["pid"]["speed"]["left"]["ki"],
                           doc["pid"]["speed"]["left"]["coulombFriction"], 0, 0, 0, 0, 0 };
    robot.rightPIDspeed = { doc["pid"]["speed"]["right"]["kp"], doc["pid"]["speed"]["right"]["ki"],
                            doc["pid"]["speed"]["right"]["coulombFriction"], 0, 0, 0, 0, 0 };
  
    // PID Position
    robot.leftPIDposition = { doc["pid"]["position"]["left"]["kp"], doc["pid"]["position"]["left"]["ki"], doc["pid"]["position"]["left"]["minSpeed"], 0, 0, 0 };
    robot.rightPIDposition = { doc["pid"]["position"]["right"]["kp"], doc["pid"]["position"]["right"]["ki"], doc["pid"]["position"]["right"]["minSpeed"], 0, 0, 0 };
  
    //RateLimiter
    robot.leftRateLimiter = {doc["rateLimiter"]["left"]["rateUp"], doc["rateLimiter"]["left"]["rateDown"], 0 };
    robot.rightRateLimiter = {doc["rateLimiter"]["right"]["rateUp"], doc["rateLimiter"]["right"]["rateDown"], 0 };

    Serial.println("Robot motors configuration loaded succesfully!");
    }else {
    Serial.println("Failed to load robot motors configuration!");
    }
  }

  //Motor control
int dutyCycleToPWM(float dutyCycle) {
    float d = constrain(abs(dutyCycle), 0, 100);
    // Convertir el porcentaje a un valor PWM
    return static_cast<int>((d / 100.0f) * 255.0f);
}
void motorsInit(){
//Left motor
  pinMode(robot.leftMotor.in1, OUTPUT);
  pinMode(robot.leftMotor.in2, OUTPUT);
  pinMode(robot.leftMotor.ena, OUTPUT);

  //Right motor
  pinMode(robot.rightMotor.in1, OUTPUT);
  pinMode(robot.rightMotor.in2, OUTPUT);
  pinMode(robot.rightMotor.ena, OUTPUT);
  
  // configure LEDC PWM functionalities
  ledcAttachChannel(robot.leftMotor.ena, PWM_FREQ, PWM_RESOLUTION, 0);
  ledcAttachChannel(robot.rightMotor.ena, PWM_FREQ, PWM_RESOLUTION, 1);

}
void motorControl(int pin1, int pin2, int pinEnable, int dutyCycle){

 if(dutyCycle > 0){
    //Move forward
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2, LOW);
  } else if(dutyCycle < 0) {
    //Move backwards
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else
  {
    // Stop
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
  int pwm = dutyCycleToPWM(dutyCycle);
  ledcWrite(pinEnable, pwm);

}
void leftMotorControl(){
  motorControl(robot.leftMotor.in1, robot.leftMotor.in2, robot.leftMotor.ena, robot.leftMotor.dutyCycle);
}
void rightMotorControl(){
  motorControl(robot.rightMotor.in1, robot.rightMotor.in2, robot.rightMotor.ena, robot.rightMotor.dutyCycle);
}
void stop(){
  leftMotorControl();
  rightMotorControl();
}

//Encoders
void encodersInit()
{
  robot.leftMotor.direction = true;//default -> Forward
  robot.rightMotor.direction =true;

  pinMode(robot.leftEncoder.pinA, INPUT);
  pinMode(robot.leftEncoder.pinB,INPUT);
  pinMode(robot.rightEncoder.pinA, INPUT);
  pinMode(robot.rightEncoder.pinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(robot.leftEncoder.pinA), leftPulsesCounter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.rightEncoder.pinA), rightPulsesCounter, CHANGE);  
}

void leftPulsesCounter()
{
  int currentState = digitalRead(robot.leftEncoder.pinA);
  if (robot.leftEncoder.pinALastState == LOW && currentState == HIGH) {
    robot.leftMotor.direction = (digitalRead(robot.leftEncoder.pinB) == HIGH);
  }
  robot.leftEncoder.pinALastState = currentState;

  // Ajustamos el contador según la dirección
  robot.leftEncoder.pulses += robot.leftMotor.direction ? -1 : 1;
  // Contador para la posicion
  robot.leftEncoder.totalPulses += robot.leftMotor.direction ? -1 : 1;
}

void rightPulsesCounter()
{
  int currentState = digitalRead(robot.rightEncoder.pinA);
  if(robot.rightEncoder.pinALastState ==LOW && currentState == HIGH){
    robot.rightMotor.direction = (digitalRead(robot.rightEncoder.pinB) == HIGH);
  }
  robot.rightEncoder.pinALastState = currentState;
  robot.rightEncoder.pulses += robot.rightMotor.direction ? 1 : -1;
  //Contador para la posicion
  robot.rightEncoder.totalPulses += robot.rightMotor.direction ? 1 : -1;

}

int sgn(float value){

  if(value > 0) return 1;
  if (value < 0) return -1;
  
  return 0;
}

void resetEncodersPulses(){
  robot.leftEncoder.pulses = 0;
  robot.rightEncoder.pulses = 0;
}


// Odometry
float calculateAngularPosition(int pulsesCount, int ppr){
  float radiansByStep = (2 * PI) / ppr; // 360º = 2PIrad / pulsesEncoder --> radianes que avanza cada pulso
  return pulsesCount * radiansByStep;
}

float calculateLinearPosition(float angularPosition){
  return angularPosition * robot.wheelRadius;
}

float linearPositionToAngularPosition(float meters){
  return meters / robot.wheelRadius;
}

float calculateAngularSpeed(int pulsesCount, float timeCount, int ppr){
  float radiansByStep = (2 * PI) / ppr;
  return (pulsesCount * radiansByStep) / timeCount; // angular speed
}

float calculateLinearSpeed(float angularSpeed){
  return angularSpeed * robot.wheelRadius; // m/s

}

void updateAngularSpeed(float deltaTime){
  robot.leftPIDspeed.currentSpeed = calculateAngularSpeed(robot.leftEncoder.pulses/2, deltaTime, robot.leftEncoder.ppr);
  robot.rightPIDspeed.currentSpeed = calculateAngularSpeed(robot.rightEncoder.pulses/2, deltaTime, robot.rightEncoder.ppr);
}

void updateAngularPosition(){
  robot.leftPIDposition.currentPosition = calculateAngularPosition(robot.leftEncoder.totalPulses/2, robot.leftEncoder.ppr);
  robot.rightPIDposition.currentPosition =  calculateAngularPosition(robot.rightEncoder.totalPulses/2, robot.rightEncoder.ppr);
}


//PIController
PIControllerResult updatePIController(float setPoint, float currentValue, float integralError, float kp, float ki) {
  // Calculate error
  float error = setPoint - currentValue;

  // Accumulate integral error
  integralError += error;

  // Calculate control output
  float controlOutput = (kp * error) + (ki * integralError);

  return {controlOutput, integralError};
}

void updatePIpositionController(){
  //Update Position controller
  PIControllerResult leftPosPI = updatePIController(robot.leftPIDposition.setPoint, robot.leftPIDposition.currentPosition, robot.leftPIDposition.integralError, robot.leftPIDposition.kp, robot.leftPIDposition.ki);
  PIControllerResult rightPosPI = updatePIController(robot.rightPIDposition.setPoint, robot.rightPIDposition.currentPosition, robot.rightPIDposition.integralError, robot.rightPIDposition.kp, robot.rightPIDposition.ki);
  //Anti windup (acciones integrales para evitar que se desborde -si se desborda entra en condición error y es muy lento para recuperarse)
  robot.leftPIDposition.integralError = constrain(leftPosPI.integralError,-10,10);
  robot.rightPIDposition.integralError = constrain(rightPosPI.integralError,-10, 10);

  robot.leftPIDspeed.setPoint = constrain(leftPosPI.value + robot.leftPIDposition.minSpeed*sgn(leftPosPI.value), -PI, PI);
  // Error que se acepta 0.01 rad en el que no hace control (para que no oscile cerca del punto, porque no va a ser exacto)
  if(abs(robot.leftPIDposition.setPoint-robot.leftPIDposition.currentPosition) < 0.1){
    robot.leftPIDspeed.setPoint=0; //Si esta dentro del error aceptable se debe detener.
    //Reset current position y set point (trabajamos con posiciones relativas a casa paso)
    robot.leftPIDposition.setPoint = 0;
    robot.leftPIDposition.currentPosition = 0;
    robot.leftEncoder.totalPulses = 0;
  }
  
  robot.rightPIDspeed.setPoint = constrain(rightPosPI.value + robot.rightPIDposition.minSpeed*sgn(leftPosPI.value), -PI, PI);
  if(abs(robot.rightPIDposition.setPoint-robot.rightPIDposition.currentPosition) < 0.1){
    robot.rightPIDspeed.setPoint=0;
    robot.rightPIDposition.setPoint= 0;
    robot.rightPIDposition.currentPosition = 0;
    robot.rightEncoder.totalPulses = 0;
  }
}

void updatePIspeedController(){
  //Limitate slope
  float leftSetPointLimited = rateLimiter(&robot.leftRateLimiter, robot.leftPIDspeed.setPoint); 
  float rightSetPointLimited = rateLimiter(&robot.rightRateLimiter, robot.rightPIDspeed.setPoint);

  // Update PI controller
  PIControllerResult leftPI = updatePIController(leftSetPointLimited, robot.leftPIDspeed.currentSpeed, robot.leftPIDspeed.integralError, robot.leftPIDspeed.kp, robot.leftPIDspeed.ki);
  PIControllerResult rightPI = updatePIController(rightSetPointLimited, robot.rightPIDspeed.currentSpeed, robot.rightPIDspeed.integralError, robot.rightPIDspeed.kp, robot.rightPIDspeed.ki);

  robot.leftMotor.dutyCycle = constrain(leftPI.value + robot.leftPIDspeed.coulombFriction*sgn(robot.leftPIDspeed.setPoint),-100,100);
  robot.rightMotor.dutyCycle = constrain(rightPI.value + robot.leftPIDspeed.coulombFriction*sgn(robot.rightPIDspeed.setPoint),-100,100);
  // Compensar las oscilaciones de la rueda cuando la velocidad es 0
  robot.leftPIDspeed.integralError = leftPI.integralError;
  if(robot.leftPIDspeed.currentSpeed==0)robot.leftPIDspeed.integralError = 0;
  robot.rightPIDspeed.integralError = rightPI.integralError;
  if(robot.rightPIDspeed.currentSpeed==0)robot.rightPIDspeed.integralError = 0;

  //Serial.printf("LIE: %.2f RIE %.2f ", leftIntegralError, rightIntegralError);
  //Update speed
  leftMotorControl(); //Cosas raras
  rightMotorControl();
}

//Rate limiter
// Function to apply a rate limiter to an input signal
// It ensures that the output does not change faster than the allowed rate
float rateLimiter(RateLimiter *rl, float input) {
  // Calculate the allowed change based on the rate limits
  float maxIncrease = rl->rateUp * loopTime;   // Maximum allowed increase per loop cycle
  float maxDecrease = rl->rateDown * loopTime; // Maximum allowed decrease per loop cycle

  // Compute the proposed change
  float proposedChange = input - rl->previousOutput;

  // Limit the change to the allowed range
  if (proposedChange > maxIncrease) {
      rl->previousOutput += maxIncrease;  // Restrict increase to max allowed rate
  } else if (proposedChange < -maxDecrease) {
      rl->previousOutput -= maxDecrease;  // Restrict decrease to max allowed rate
  } else {
      rl->previousOutput = input;  // Apply input directly if within limits
  }

  // Return the constrained output value
  return rl->previousOutput;
}

//NAVIGATION
float calculateRadiansRotation(float angle){
  //Transformar grados a radianes
  float theta = abs(angle) * PI / 180;

  // Calcular distancia que debe recorrer cada rueda
  float distance = (robot.wheelBase / 2.0) * theta; // (L/2)*theta

  // Convertir distancia a radianes de rueda
  float angularDisplacement = distance / robot.wheelRadius; // radianes que deben girar las ruedas

  return angularDisplacement;

}

void forward(float meters){
  float angularPos = linearPositionToAngularPosition(meters);
  robot.leftPIDposition.setPoint = angularPos;
  robot.rightPIDposition.setPoint = angularPos;
}
void turn(float angle){
  float angularDisplacement = calculateRadiansRotation(angle);
  robot.leftPIDposition.setPoint = sgn(angle) * angularDisplacement;
  robot.rightPIDposition.setPoint = sgn(angle) * -angularDisplacement;
}
void processAction(String action, float value){
  if (action.equalsIgnoreCase("forward")){
    forward(value);
  }
  else if(action.equalsIgnoreCase("turn")){
    turn(value);
  }
  else if(action.equalsIgnoreCase("stop")){
    stop();
  }else{
    Serial.println("Unrecognized action");
  }

}


//Plot
void logRobotControlInfoToSerial(){
  //Duty cycle
  Serial.print(">LDuty:");
  Serial.println(robot.leftMotor.dutyCycle);
  Serial.print(">RDuty:");
  Serial.println(robot.rightMotor.dutyCycle);
  //Speed control info
  Serial.print(">LSpeed:");
  Serial.println(robot.leftPIDspeed.currentSpeed);
  Serial.print(">RSpeed:");
  Serial.println(robot.rightPIDspeed.currentSpeed);
  Serial.print(">LSpeedSetPoint:");
  Serial.println(robot.leftPIDspeed.setPoint);
  Serial.print(">RSpeedSetPoint:");
  Serial.println(robot.rightPIDspeed.setPoint);
  Serial.print(">LSpeedKp:");
  Serial.println(robot.leftPIDspeed.kp);
  Serial.print(">LSpeedKi:");
  Serial.println(robot.leftPIDspeed.ki);
  Serial.print(">RSpeedKp:");
  Serial.println(robot.rightPIDspeed.kp);
  Serial.print(">RSpeedKi:");
  Serial.println(robot.rightPIDspeed.ki);
  //Position control info
  Serial.print(">LAngularPos:");
  Serial.println(robot.leftPIDposition.currentPosition);
  Serial.print(">RAngularPos:");
  Serial.println(robot.rightPIDposition.currentPosition);
  Serial.print(">LPosSetPoint:");
  Serial.println(robot.leftPIDposition.setPoint);
  Serial.print(">RPosSetPoint:");
  Serial.println(robot.rightPIDposition.setPoint);
  Serial.print(">LPosKp:");
  Serial.println(robot.leftPIDposition.kp);
  Serial.print(">LPosKi:");
  Serial.println(robot.leftPIDposition.ki);
  Serial.print(">RPosKp:");
  Serial.println(robot.rightPIDposition.kp);
  Serial.print(">RPosKi:");
  Serial.println(robot.rightPIDposition.ki);
}
//Create a method to log robot info after read configFile

