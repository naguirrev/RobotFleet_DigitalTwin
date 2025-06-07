#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <motors.h>
#include <mqtt.h>

// --- Timers ---
unsigned long lastControlTime = 0;
unsigned long currentTime = 0;
//Mode
int mode = 3;
//Plot
bool publish = true; 

// --- Functions declaration ---
void setControlMode(int mode);

// --- MAIN: setup and loop ---
void setup() {
  
  Serial.begin(115200);
  //Mount mqtt
  if (!LittleFS.begin(true)) {
      Serial.println("LittleFS mount failed!");
      return;
  }
  //Load config from files
  initRobot();
  initMqtt();
  
  //Init sensors and actuators
  motorsInit();
  encodersInit();
  //Wifi
  setupWifi();
  //MQTT
  setupMqtt();

  Serial.print("Initializing system...");
}

void loop() {
  //Initialize mqtt
  ensureMqttConnected();

  // Calculate elapsed time for speed calculation
  currentTime = millis();
  float deltaTime = (currentTime - lastControlTime) / 1000.0; // Convert to seconds

  // Calculate current angular speed
  updateAngularSpeed(deltaTime);
  // Calculate current angular position
  updateAngularPosition();
  //Control mode
  setControlMode(mode);
  
  // Reset encoder pulses for the next angular speed calculation
  resetEncodersPulses();
  lastControlTime = currentTime;

  //Publish to mqtt topics
  if (publish){
    logRobotControlInfoToSerial();
    publishRobotState(deltaTime);
    //publishControllerSpeedData();
  }
  
  delay(10); // Small delay for stability 

}


// --- Functions definition ---
void setControlMode(int mode){
  
  switch (mode)
  {
    case 0:{
      //Stop the car and data publication
      stop();
      publish = false;
      break;
    }
    case 1:{
      //Open-loop control: read duty cycle and set car pwm
      publish = true;
      leftMotorControl();
      rightMotorControl();
      break;
      }
    case 2:{
      //Speed controller: read angular speed set point and use speed PI controller
      publish = true;
      updatePIspeedController();
      break;
    } 
    case 3:{
      //Position controller: read angular position set point and use position PI controller
      publish = true;
      updatePIpositionController();
      updatePIspeedController();
      break;
    }
    default:{
      //Stop the car
      publish =true;
      stop();
      break;
    }
  }
}


  








