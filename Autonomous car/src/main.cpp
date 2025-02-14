#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <motors.h>
#include <LittleFS.h>

//------------Settings-------------
Robot robot;

bool loadConfig(const char *filename, Robot &robot) {
  File file = LittleFS.open(filename, "r");
  if (!file) {
      Serial.println("Error opening config file!");
      return false;
  }

  StaticJsonDocument<1024> doc;  
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
      Serial.println("JSON parsing failed!");
      return false;
  }

  // Cargar datos en el struct Robot
  robot.id = doc["id"].as<String>();
  robot.wheelRadius = doc["wheelRadius"].as<float>();

  // Motores
  robot.leftMotor = { doc["motorPin"]["left"]["in1"], doc["motorPin"]["left"]["in2"], doc["motorPin"]["left"]["ena"] };
  robot.rightMotor = { doc["motorPin"]["right"]["in1"], doc["motorPin"]["right"]["in2"], doc["motorPin"]["right"]["ena"] };

  // Encoders
  robot.leftEncoder = { doc["encoder"]["left"]["pinA"], doc["encoder"]["left"]["pinB"] };
  robot.rightEncoder = { doc["encoder"]["right"]["pinA"], doc["encoder"]["right"]["pinB"] };

  // PID Speed
  robot.leftPIDspeed = { doc["pid"]["speed"]["left"]["kp"], doc["pid"]["speed"]["left"]["ki"], 0,
                         doc["pid"]["speed"]["left"]["coulombFriction"], 0, 0, 0 };
  robot.rightPIDspeed = { doc["pid"]["speed"]["right"]["kp"], doc["pid"]["speed"]["right"]["ki"], 0,
                          doc["pid"]["speed"]["right"]["coulombFriction"], 0, 0, 0 };

  // PID Position
  robot.leftPIDposition = { doc["pid"]["position"]["left"]["kp"], doc["pid"]["position"]["left"]["ki"], doc["pid"]["position"]["left"]["minSpeed"], 0 };
  robot.rightPIDposition = { doc["pid"]["position"]["right"]["kp"], doc["pid"]["position"]["right"]["ki"], doc["pid"]["position"]["right"]["minSpeed"], 0 };

  return true;
}

//---------WIFI----------------- 
const char* ssid = "DIGIFIBRA-kDuE";
const char* password = "N6HAFdFPSfDd";

//---------MQTT-------------------
const char* mqtt_server = "192.168.1.136"; // IP del servidor RabbitMQ
const int mqtt_port = 1883; // Puerto MQTT
const char* mqtt_user = "test"; // Usuario MQTT
const char* mqtt_password = "test"; // Contraseña MQTT

//Mqtt topics 

//Read topics
const char* mqtt_topic_speed = "smr/speed/car1"; //car speed


//Write topics
//Speed control
const char* mqtt_topic_leftKp = "smr/lKp/car1";
const char* mqtt_topic_leftKi = "smr/lKi/car1";
const char* mqtt_topic_rightKp = "smr/rKp/car1";
const char* mqtt_topic_rightKi = "smr/rKi/car1";
const char* mqtt_topic_duty= "smr/duty/car1";
const char* mqtt_topic_angularSpeed = "smr/angularSpeed/car1";

//Position control
const char* mqtt_topic_leftPosKp = "smr/lPosKp/car1";
const char* mqtt_topic_leftPosKi = "smr/lPosKi/car1";
const char* mqtt_topic_rightPosKp = "smr/rPosKp/car1";
const char* mqtt_topic_rightPosKi = "smr/rPosKi/car1";
const char* mqtt_topic_angularPos = "smr/angularPos/car1";

const char* mqtt_topic_mode = "smr/mode/car1";



WiFiClient espClient;
PubSubClient client(espClient);

void setupWifi();
void reconnectMqtt();
void callbackMqtt(char* topic, byte* payload, unsigned int length);

// ------- MOTOR CONTROL AND ENCODERS -------------
//Car properties
#define WHEEL_RADIUS 0.03 // Radio de la rueda en metros (ajusta según tu mecanismo)

//Left motor
#define LEFT_MOTOR_IN1 2
#define LEFT_MOTOR_IN2 15
#define LEFT_MOTOR_ENA 4

//Right motor
#define RIGHT_MOTOR_IN3 14
#define RIGHT_MOTOR_IN4 12
#define RIGHT_MOTOR_ENB 13 

//Encoder
#define PPR 960 // Pulsos por vuelta del encoder (ajusta según tu encoder)

//Left encoder
#define LEFT_ENCODER_A 35 //A pin 
#define LEFT_ENCODER_B 34 //B pin
volatile byte leftEncoderALastState = LOW; // Inicializar adecuadamente
boolean leftMotorDirection;//the rotation leftMotorDirection
int leftEncoderPulses = 0;
int leftPositionEncoderPulses = 0;

//Right encoder
#define RIGHT_ENCODER_A 33 //A pin 
#define RIGHT_ENCODER_B 32 //B pin
volatile byte rightEncoderALastState = LOW; 
boolean rightMotorDirection;
int rightEncoderPulses = 0;
int rightPositionEncoderPulses = 0;

// Setting PWM properties
#define PWM_FREQ 30000
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
float leftDutyCycle = 0.0; // % of PWM
float rightDutyCycle = 0.0;

// --- PI Controller ---
float leftCoulombFriction = 40; //Value 45, pero requiere un suavizado
float rightCoulombFriction = 40;
float leftSetPoint = PI; // Velocidad angular deseada (rad/s)
float rightSetPoint = PI;
float leftKp = 13.2;   // Ganancia proporcional 
float leftKi = 5;   // Ganancia integral 
float rightKp = 13.2;  // Ganancia proporcional 
float rightKi = 5;  // Ganancia integral 
float leftIntegralError = 0;
float rightIntegralError = 0;

float leftPreviousError = 0;
float leftPreviousIntegralOutput = 0;

float rateUp = 8;
float rateDown = 8;
float deltaTime = 0.01;


// Position controller
float leftMinSpeed = 0.5; //Value 0.5, pero requiere un suavizado
float rightMinSpeed = 0.5;
float leftAngularPos = 0;
float rightAngularPos = 0;
float leftPosSetPoint = 0;
float rightPosSetPoint = 0;
float leftPosKp = 2.5;   // Ganancia proporcional 
float leftPosKi = 0;   // Ganancia integral 
float rightPosKp = 2.5;  // Ganancia proporcional 
float rightPosKi = 0;  // Ganancia integral 
float leftPosIntegralError = 0;
float rightPosIntegralError = 0;

// --- Timer ---
unsigned long lastControlTime = 0;

// --- Structs ---
RateLimiter leftRateLimiter;
RateLimiter rightRateLimiter;


// --- Functions ----
//Plot
bool publish = true; 
void publishSpeedMqtt(float leftCurrentSpeed, float rightCurrentSpeed);
void printSpeedSerial(float leftCurrentSpeed, float rightCurrentSpeed);
void processMqttMessage(char* topic, byte* payload, unsigned int length);
//Mode
int mode = 1;
//Motors
void motorsInit();
void leftMotorControl(float dutyCycle);
void rightMotorControl(float dutyCycle);
void leftMotorControl(int IN1, int IN2, float dutyCycle);
void rightMotorControl(int IN3, int IN4, float dutyCycle);
void forward();
void backward();
void stop();

//Encoders
void encodersInit();
void leftPulsesCounter();
void rightPulsesCounter();

//Odometry
float calculateAngularPosition(int pulsesCount);
float calculateLinearPosition(float angularPosition);
float calculateAngularSpeed(int pulsesCount, float timeCount);
float calculateLinearSpeed(float angularSpeed);

// PI Control
PIControllerResult updatePIController(float setPoint, float currentSpeed, float integralError, float kp, float ki);
PIControllerResult updatePIPositionController(float setPoint, float currentPosition, float integralError, float kp, float ki);
void initializeRateLimiter(RateLimiter *rl, float rateUp, float rateDown, float deltaTime);
float rateLimiter(RateLimiter *rl, float input);
int sgn(float value); 

void setup() {
  
  Serial.begin(115200);
  if (!LittleFS.begin(true)) {
      Serial.println("LittleFS mount failed!");
      return;
  }
  Serial.println("Loading Robot Configuration...");
  if (loadConfig("/robot.json", robot)) {
      Serial.println("Robot configuration loaded successfully!");
  } else {
      Serial.println("Failed to load configuration!");
  }

  Serial.print("Robot ID: "); Serial.println(robot.id);
  motorsInit();
  encodersInit();
  //RateLimiters
  initializeRateLimiter(&leftRateLimiter, rateUp, rateDown, deltaTime);
  initializeRateLimiter(&rightRateLimiter, rateUp, rateDown, deltaTime);

  //Wifi
  setupWifi();
  //MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callbackMqtt);

  Serial.print("Initializing system...");
}


void loop() {
  Serial.print("Robot ID: "); Serial.println(robot.id);
  //Initialize mqtt
  if (!client.connected()) {
    reconnectMqtt();
  }
  client.loop();

  // Calculate elapsed time for speed calculation
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastControlTime) / 1000.0; // Convert to seconds

  // Calculate current angular speed
  float leftCurrentSpeed = calculateAngularSpeed(leftEncoderPulses/2, deltaTime);
  float rightCurrentSpeed = calculateAngularSpeed(rightEncoderPulses/2, deltaTime);

  // Calculate angular position
  leftAngularPos = calculateAngularPosition(leftPositionEncoderPulses/2);
  rightAngularPos =  calculateAngularPosition(rightPositionEncoderPulses/2);
 
  //Control mode
  switch (mode)
  {
  case 0:{
    //stop the car and mqtt data publication
    stop();
    publish = false;
    break;
  }
  case 1:{
    //read duty cycle and set car pwm
    publish = true;
    leftMotorControl(leftDutyCycle);
    rightMotorControl(rightDutyCycle);
    break;
    }
  case 2:{
    //read angular speed set point and use pi controller
    publish = true;

    //Limitate slope
    float leftSetPointLimited = rateLimiter(&leftRateLimiter, leftSetPoint);
    float rightSetPointLimited = rateLimiter(&rightRateLimiter, rightSetPoint);

    // Update PI controller
    PIControllerResult leftPI = updatePIController(leftSetPointLimited, leftCurrentSpeed, leftIntegralError, leftKp, leftKi);
    PIControllerResult rightPI = updatePIController(rightSetPointLimited, rightCurrentSpeed, rightIntegralError, rightKp, rightKi);

    leftDutyCycle = constrain(leftPI.value + leftCoulombFriction*sgn(leftSetPoint),-100,100);
    rightDutyCycle = constrain(rightPI.value + rightCoulombFriction*sgn(rightSetPoint),-100,100);
    // Compensar las oscilaciones de la rueda cuando la velocidad es 0
    leftIntegralError = leftPI.integralError;
    if(leftCurrentSpeed==0)leftIntegralError = 0;
    rightIntegralError = rightPI.integralError;
    if(rightCurrentSpeed==0)rightIntegralError = 0;

    //Serial.printf("LIE: %.2f RIE %.2f ", leftIntegralError, rightIntegralError);
    //Update speed
    leftMotorControl(leftDutyCycle); //Cosas raras
    rightMotorControl(rightDutyCycle);
    break;
  } 
  case 3:{
    //Control angular position
    publish = true;

    //Update Position controller
    PIControllerResult leftPosPI = updatePIPositionController(leftPosSetPoint, leftAngularPos, leftPosIntegralError, leftPosKp, leftPosKi);
    PIControllerResult rightPosPI = updatePIPositionController(rightPosSetPoint, rightAngularPos, rightPosIntegralError, rightPosKp, rightPosKi);
    leftPosIntegralError = constrain(leftPosPI.integralError,-10,10);
    rightPosIntegralError = constrain(rightPosPI.integralError,-10, 10);

    leftSetPoint = constrain(leftPosPI.value + leftMinSpeed*sgn(leftPosPI.value), -PI, PI);
    // Error que se acepta 0.04 rad en el que no hace control (para que no oscile cerca del punto, porque no va a ser exacto)
    if(abs(leftPosSetPoint-leftAngularPos) < 0.1)leftSetPoint=0;
    rightSetPoint = constrain(rightPosPI.value + rightMinSpeed*sgn(leftPosPI.value), -PI, PI);
    if(abs(rightPosSetPoint-rightAngularPos) < 0.1)rightSetPoint=0;

     //Limitate slope
    float leftSetPointLimited = rateLimiter(&leftRateLimiter, leftSetPoint);
    float rightSetPointLimited = rateLimiter(&rightRateLimiter, rightSetPoint);

    // Update PI controller
    PIControllerResult leftPI = updatePIController(leftSetPointLimited, leftCurrentSpeed, leftIntegralError, leftKp, leftKi);
    PIControllerResult rightPI = updatePIController(rightSetPointLimited, rightCurrentSpeed, rightIntegralError, rightKp, rightKi);

    leftDutyCycle = constrain(leftPI.value + leftCoulombFriction*sgn(leftSetPoint),-100,100);
    rightDutyCycle = constrain(rightPI.value + rightCoulombFriction*sgn(rightSetPoint),-100,100);
    leftIntegralError = leftPI.integralError;
    if(leftCurrentSpeed==0)leftIntegralError = 0;
    rightIntegralError = rightPI.integralError;
    if(rightCurrentSpeed==0)rightIntegralError = 0;
  
    //Serial.printf("LIE: %.2f RIE %.2f ", leftIntegralError, rightIntegralError);
    //Update speed
    leftMotorControl(leftDutyCycle); //Cosas raras
    rightMotorControl(rightDutyCycle);
    break;
  }
  default:{
    //stop the car
    publish =true;
    stop();
    break;
  }
  }
  

  // Reset encoder pulses for the next angular speed calculation
  leftEncoderPulses = 0;
  rightEncoderPulses = 0;
  lastControlTime = currentTime;

  // Debugging output
  //Serial.printf("LSP: %.2f RSP:%.2f | LS: %.2f DC: %d | RS: %.2f DC: %d\n", leftSetPint, rightSetPoint, leftCurrentSpeed, leftDutyCycle, rightCurrentSpeed, rightDutyCycle);

  
  //Publish to mqtt topics
  if (publish){
    printSpeedSerial(leftCurrentSpeed, rightCurrentSpeed);
    publishSpeedMqtt(leftCurrentSpeed, rightCurrentSpeed);
  }
  

  delay(10); // Small delay for stability (adjust as necessary)
}

//Plot
void printSpeedSerial(float leftCurrentSpeed, float rightCurrentSpeed){
  Serial.print(">LSpeed:");
  Serial.println(leftCurrentSpeed);
  Serial.print(">RSpeed:");
  Serial.println(rightCurrentSpeed);
  Serial.print(">LKp:");
  Serial.println(leftKp);
  Serial.print(">LKi:");
  Serial.println(leftKi);
  Serial.print(">RKp:");
  Serial.println(rightKp);
  Serial.print(">RKi:");
  Serial.println(rightKi);
  Serial.print(">LDuty:");
  Serial.println(leftDutyCycle);
  Serial.print(">RDuty:");
  Serial.println(rightDutyCycle);
  Serial.print(">LSetPoint:");
  Serial.println(leftSetPoint);
  Serial.print(">RSetPoint:");
  Serial.println(rightSetPoint);
  Serial.print(">LAngularPos:");
  Serial.println(leftAngularPos);
  Serial.print(">RAngularPos:");
  Serial.println(rightAngularPos);
  Serial.print(">LPosSetPoint:");
  Serial.println(leftPosSetPoint);
  Serial.print(">RPosSetPoint:");
  Serial.println(rightPosSetPoint);
  Serial.print(">LPosKp:");
  Serial.println(leftPosKp);
  Serial.print(">LPosKi:");
  Serial.println(leftPosKi);
  Serial.print(">RPosKp:");
  Serial.println(rightPosKp);
  Serial.print(">RPosKi:");
  Serial.println(rightPosKi);
}

void publishSpeedMqtt(float leftCurrentSpeed, float rightCurrentSpeed){
  String speedMessage = "{";
  speedMessage += "\"leftSpeed\": " + String(leftCurrentSpeed, 2) + ",";
  speedMessage += "\"rightSpeed\": " + String(rightCurrentSpeed, 2);
  speedMessage += "}";

  // Publish JSON message
  client.publish(mqtt_topic_speed, speedMessage.c_str());
}

//Motor control
int dutyCycleToPWM(float dutyCycle) {
    float d = constrain(abs(dutyCycle), 0, 100);
    // Convertir el porcentaje a un valor PWM
    return static_cast<int>((d / 100.0f) * 255.0f);
}
void motorsInit(){
//Left motor
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENA, OUTPUT);

  //Right motor
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  pinMode(RIGHT_MOTOR_ENB, OUTPUT);
  
  // configure LEDC PWM functionalities
  ledcAttachChannel(LEFT_MOTOR_ENA, PWM_FREQ, PWM_RESOLUTION, 0);
  ledcAttachChannel(RIGHT_MOTOR_ENB, PWM_FREQ, PWM_RESOLUTION, 1);

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

void leftMotorControl(float dutyCycle){
  motorControl(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_ENA, dutyCycle);
}

void rightMotorControl(float dutyCycle){
  motorControl(RIGHT_MOTOR_IN3, RIGHT_MOTOR_IN4, RIGHT_MOTOR_ENB, dutyCycle);
}

void leftMotorControl(int IN1, int IN2, float dutyCycle){
  digitalWrite(LEFT_MOTOR_IN1, IN1);
  digitalWrite(LEFT_MOTOR_IN2, IN2);
  int pwm = dutyCycleToPWM(dutyCycle);
  ledcWrite(LEFT_MOTOR_ENA, pwm);
}
void rightMotorControl(int IN3, int IN4, float dutyCycle){
  digitalWrite(RIGHT_MOTOR_IN3, IN3);
  digitalWrite(RIGHT_MOTOR_IN4, IN4);
  int pwm = dutyCycleToPWM(dutyCycle);
  ledcWrite(RIGHT_MOTOR_ENB, pwm);
}
void forward() {
    leftMotorControl(HIGH, LOW, leftDutyCycle);
    rightMotorControl(HIGH, LOW, rightDutyCycle);

}
void backward() {
    leftMotorControl(LOW, HIGH, leftDutyCycle);
    rightMotorControl(LOW, HIGH, rightDutyCycle);
}
void stop(){
  leftMotorControl(LOW, LOW, 0);
  rightMotorControl(LOW, LOW, 0);
}

//Encoders
void encodersInit()
{
  leftMotorDirection = true;//default -> Forward
  rightMotorDirection =true;

  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B,INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftPulsesCounter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightPulsesCounter, CHANGE);  
}

void leftPulsesCounter()
{
  int currentState = digitalRead(LEFT_ENCODER_A);
  if (leftEncoderALastState == LOW && currentState == HIGH) {
    leftMotorDirection = (digitalRead(LEFT_ENCODER_B) == HIGH);
  }

  leftEncoderALastState = currentState;

  // Ajustamos el contador según la dirección
  leftEncoderPulses += leftMotorDirection ? -1 : 1;
  // Contador para la posicion
  leftPositionEncoderPulses += leftMotorDirection ? -1 : 1;
}

void rightPulsesCounter()
{
  int currentState = digitalRead(RIGHT_ENCODER_A);
  if(rightEncoderALastState ==LOW && currentState == HIGH){
    rightMotorDirection = (digitalRead(RIGHT_ENCODER_B) == HIGH);
  }
  rightEncoderALastState = currentState;
  rightEncoderPulses += rightMotorDirection ? 1 : -1;
  //Contador para la posicion
  rightPositionEncoderPulses += rightMotorDirection ? 1 : -1;

}

int sgn(float value){

  if(value > 0) return 1;
  if (value < 0) return -1;
  
  return 0;
}

// Odometry
float calculateAngularPosition(int pulsesCount){
  float radiansByStep = (2 * PI) / PPR; // 360º = 2PIrad / pulsesEncoder --> radianes que avanza cada pulso
  return pulsesCount * radiansByStep;
}

float calculateLinearPosition(float angularPosition){
  return angularPosition * WHEEL_RADIUS;
}

float calculateAngularSpeed(int pulsesCount, float timeCount){
  float radiansByStep = (2 * PI) / PPR;
  return (pulsesCount * radiansByStep) / timeCount; // angular speed
}

float calculateLinearSpeed(float angularSpeed){
  return angularSpeed * WHEEL_RADIUS; // m/s

}

// Speed control
PIControllerResult updatePIController(float setPoint, float currentSpeed, float integralError, float kp, float ki) {
  // Calculate error
  float error = setPoint - currentSpeed;

  // Accumulate integral error
  integralError += error;

  // Calculate control output
  float controlOutput = (kp * error) + (ki * integralError);
  //float dutyCycle = constrain(controlOutput, 0, 100);
  float dutyCycle = controlOutput;
  // Convert to duty cycle
  return {dutyCycle, integralError};
}

//Position control
PIControllerResult updatePIPositionController(float setPoint, float currentPosition, float integralError, float kp, float ki){

  //Calculate error
  float error = setPoint - currentPosition;

  //Accumulate integral error
  integralError += error;

  //Calculate control output 
  float controlOutput = (kp * error) + (ki * integralError);
  // Convert to duty cycle
  return {controlOutput, integralError};
}

//-----WIFI------
void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
}

// ------MQTT-------
void reconnectMqtt() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Connecting...");
      //Subscribe to required topics
      client.subscribe(mqtt_topic_leftKp);
      client.subscribe(mqtt_topic_leftKi);
      client.subscribe(mqtt_topic_rightKp);
      client.subscribe(mqtt_topic_rightKi);
      client.subscribe(mqtt_topic_mode);
      client.subscribe(mqtt_topic_duty);
      client.subscribe(mqtt_topic_angularSpeed);
      client.subscribe(mqtt_topic_leftPosKp);
      client.subscribe(mqtt_topic_leftPosKi);
      client.subscribe(mqtt_topic_rightPosKp);
      client.subscribe(mqtt_topic_rightPosKi);
      client.subscribe(mqtt_topic_angularPos);
    } else {
      Serial.print("Error connecting to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }
}
void callbackMqtt(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received at topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  processMqttMessage(topic, payload, length);
}

StaticJsonDocument<128> readAsJson(char message[]) {
    StaticJsonDocument<128> jsonDoc; // Create a JSON document with sufficient size
    DeserializationError error = deserializeJson(jsonDoc, message);

    if (error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        // Return an empty document in case of an error
        return StaticJsonDocument<128>();
    }
    return jsonDoc;
}


void processMqttMessage(char* topic, byte* payload, unsigned int length){
  // Convertir el payload a una cadena para facilitar la conversión posterior
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0'; // Añadir terminador nulo para que sea una cadena válida

  if (strcmp(topic, mqtt_topic_mode) == 0) {
    // Leer payload como int y establecer el modo
    int m = atoi(message); // Convertir cadena a entero
    mode = m;
    Serial.print("Mode set to: ");
    Serial.println(m);
  }
  else if (strcmp(topic, mqtt_topic_duty) == 0) {
    // Interpretar el payload como JSON y extraer valores de "left" y "right"
    StaticJsonDocument<128> jsonDoc = readAsJson(message);
    // Leer los valores de "left" y "right" del JSON
    float left = jsonDoc["left"];
    float right = jsonDoc["right"];
    leftDutyCycle = left;
    rightDutyCycle = right;
    Serial.print("Duty cycle set to - Left: ");
    Serial.print(left);
    Serial.print(", Right: ");
    Serial.println(right);
  }
  else if (strcmp(topic, mqtt_topic_angularSpeed) == 0) {
    // Interpretar el payload como JSON y extraer valores de "left" y "right"
    StaticJsonDocument<128> jsonDoc = readAsJson(message);
    // Leer payload como float y establecer la velocidad angular
    float left = jsonDoc["left"];
    float right = jsonDoc["right"];
    leftSetPoint = left;
    rightSetPoint = right;
    Serial.print("Angular speed set to - Left: ");
    Serial.print(leftSetPoint);
    Serial.print(", Right: ");
    Serial.println(right);
  }else if (strcmp(topic, mqtt_topic_leftKp) == 0) {
    float value = atof(message);
    leftKp = value;
    Serial.print("Left Kp set to: ");
    Serial.println(leftKp);
  }else if (strcmp(topic, mqtt_topic_leftKi) == 0) {
    float value = atof(message);
    leftKi = value;
    Serial.print("Left Ki set to: ");
    Serial.println(leftKi);
  }else if (strcmp(topic, mqtt_topic_rightKp) == 0) {
    float value = atof(message);
    rightKp = value;
    Serial.print("Right Kp set to: ");
    Serial.println(rightKp);
  }else if (strcmp(topic, mqtt_topic_rightKi) == 0) {
    float value = atof(message);
    rightKi = value;
    Serial.print("Right Ki set to: ");
    Serial.println(rightKi);
  }else if (strcmp(topic, mqtt_topic_angularPos) == 0) {
    // Interpretar el payload como JSON y extraer valores de "left" y "right"
    StaticJsonDocument<128> jsonDoc = readAsJson(message);
    // Leer payload como float y establecer la velocidad angular
    float left = jsonDoc["left"];
    float right = jsonDoc["right"];
    leftPosSetPoint = left;
    rightPosSetPoint = right;
    Serial.print("Angular position set to - Left: ");
    Serial.print(leftPosSetPoint);
    Serial.print(", Right: ");
    Serial.println(rightPosSetPoint);
  }else if (strcmp(topic, mqtt_topic_leftPosKp) == 0) {
    float value = atof(message);
    leftPosKp = value;
    Serial.print("Left Kp set to: ");
    Serial.println(leftPosKp);
  }else if (strcmp(topic, mqtt_topic_leftPosKi) == 0) {
    float value = atof(message);
    leftPosKi = value;
    Serial.print("Left Ki set to: ");
    Serial.println(leftPosKi);
  }else if (strcmp(topic, mqtt_topic_rightPosKp) == 0) {
    float value = atof(message);
    rightPosKp = value;
    Serial.print("Right Kp set to: ");
    Serial.println(rightPosKp);
  }else if (strcmp(topic, mqtt_topic_rightPosKi) == 0) {
    float value = atof(message);
    rightPosKi = value;
    Serial.print("Right Ki set to: ");
    Serial.println(rightPosKi);
  }
  else {
    Serial.println("Unknown topic received.");
  }
}


//Rate limiter
// Inicializa el Rate Limiter
void initializeRateLimiter(RateLimiter *rl, float rateUp, float rateDown, float deltaTime) {
    rl->rateUp = rateUp;
    rl->rateDown = rateDown;
    rl->deltaTime = deltaTime;
    rl->previousOutput = 0.0; // Inicializa la salida anterior en 0
}

// Función para limitar la señal
float rateLimiter(RateLimiter *rl, float input) {
    // Calcula el cambio permitido basado en los límites
    float maxIncrease = rl->rateUp * rl->deltaTime;
    float maxDecrease = rl->rateDown * rl->deltaTime;

    // Calcula el cambio propuesto
    float proposedChange = input - rl->previousOutput;

    // Limita el cambio
    if (proposedChange > maxIncrease) {
        rl->previousOutput += maxIncrease;
    } else if (proposedChange < -maxDecrease) {
        rl->previousOutput -= maxDecrease;
    } else {
        rl->previousOutput = input;   
    }

    // Retorna la salida limitada
    return rl->previousOutput;
}
  








