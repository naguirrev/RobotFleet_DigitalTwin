#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <mqtt.h>
#include <globals.h>
#include <motors.h>

Mqtt mqtt;
WifiCredentials wifi;
WiFiClient espClient;
PubSubClient client(espClient);
const char *wifiConfigFile = "/wifi.json";
const char *mqttConfigFile = "/mqtt.json";
// -----WIFI-----
void initWifi()
  {
    JsonDocument doc;
    bool result = readFileAsJson(doc, wifiConfigFile);
    if (result) {
      wifi = {strdup(doc["ssid"]), strdup(doc["password"])};
      Serial.println("Wifi configuration loaded succesfully");
    }else{
      Serial.println("Failed to load Wifi configuration!");
    }
  }
void setupWifi()
{
  initWifi();
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi.ssid);

  WiFi.begin(wifi.ssid, wifi.password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
}
// ------MQTT-------
// private methods
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
String getTopic (const char* value){
  return mqtt_base + robot.id + value;
}

void reconnectMqtt() {
    while (!client.connected()) {
      Serial.print("Connecting to MQTT...");
      if (client.connect("ESP32Client", mqtt.user, mqtt.password)) {
        Serial.println("Connecting...");
        //Subscribe to required topics
        
        client.subscribe(getTopic(mqtt_mode).c_str());
        client.subscribe(getTopic(mqtt_encoder_left_set).c_str());
        client.subscribe(getTopic(mqtt_encoder_right_set).c_str());
        client.subscribe(getTopic(mqtt_ultrasonic_set).c_str());
        client.subscribe(getTopic(mqtt_rfid_set).c_str());
        client.subscribe(getTopic(mqtt_controller_speed_left_set).c_str());
        client.subscribe(getTopic(mqtt_controller_speed_right_set).c_str());
        client.subscribe(getTopic(mqtt_controller_speed_command_setpoint).c_str());
        client.subscribe(getTopic(mqtt_controller_position_left_set).c_str());
        client.subscribe(getTopic(mqtt_controller_position_right_set).c_str());
        client.subscribe(getTopic(mqtt_controller_position_command_setpoint).c_str());
        client.subscribe(getTopic(mqtt_actuator_motor_set).c_str());
        client.subscribe(getTopic(mqtt_navigation_path_set).c_str());
        client.subscribe(getTopic(mqtt_navigation_command).c_str());
        
      } else {
        Serial.print("Error connecting to MQTT broker, rc=");
        Serial.print(client.state());
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
      }
    }
  }

JsonDocument readAsJson(char message[]) {
    JsonDocument jsonDoc; // Create a JSON document with sufficient size
    DeserializationError error = deserializeJson(jsonDoc, message);

    if (error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        // Return an empty document in case of an error
        return jsonDoc;
    }
    return jsonDoc;
} 
// public methods
void initMqtt()
  {
    JsonDocument doc;  
    bool result = readFileAsJson(doc, mqttConfigFile);
    if (result){
      mqtt = {strdup(doc["server"]), doc["port"].as<int>(), strdup(doc["user"]), strdup(doc["password"])};
      Serial.println("Mqtt configuration loaded succesfully!");
    } else {
      Serial.println("Failed to load wifi configuration!");
    }
  }
void setupMqtt(){
  client.setServer(mqtt.server, mqtt.port);
  client.setCallback(callbackMqtt);
}
void ensureMqttConnected(){
  if (!client.connected()) {
    reconnectMqtt();
  }
  client.loop();
}

void processMqttMessage(char* topic, byte* payload, unsigned int length){
    // Convertir el payload a una cadena para facilitar la conversión posterior
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0'; // Añadir terminador nulo para que sea una cadena válida

    //Robot: Control mode
    if (strcmp(topic, getTopic(mqtt_mode).c_str()) == 0) {
      int m = atoi(message);
      mode = m;
      Serial.print("Mode set to: ");
      Serial.println(m);
    } 

    //Encoder
    else if(strcmp(topic, getTopic(mqtt_encoder_left_set).c_str()) == 0){

    }
    else if(strcmp(topic, getTopic(mqtt_encoder_right_set).c_str()) == 0){

    }
    //Ultrasonic
    else if(strcmp(topic, getTopic(mqtt_ultrasonic_set).c_str()) == 0){

    }
    //RFID
    else if(strcmp(topic, getTopic(mqtt_rfid_set).c_str()) == 0){

    }
    //Controller: speed (set, command)
    else if (strcmp(topic, getTopic(mqtt_controller_speed_left_set).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      if (jsonDoc["kp"].is<float>()){
        float kp = jsonDoc["kp"];
        robot.leftPIDspeed.kp = kp;
      }
      if (jsonDoc["ki"].is<float>()){
        float ki = jsonDoc["ki"];
        robot.leftPIDspeed.ki = ki;
      }
      if (jsonDoc["coulombFriction"].is<float>()){
        float coulombFriction = jsonDoc["coulombFriction"];
        robot.leftPIDspeed.coulombFriction = coulombFriction;
      }
      if (jsonDoc["rateUp"].is<float>()){
        float rateUp = jsonDoc["rateUp"];
        robot.leftRateLimiter.rateUp = rateUp;
      }
      if (jsonDoc["rateDown"].is<float>()){
        float rateDown = jsonDoc["rateDown"];
        robot.leftRateLimiter.rateDown = rateDown;
      }

    }
    else if (strcmp(topic, getTopic(mqtt_controller_speed_right_set).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      if (jsonDoc["kp"].is<float>()){
        float kp = jsonDoc["kp"];
        robot.rightPIDspeed.kp = kp;
      }
      if (jsonDoc["ki"].is<float>()){
        float ki = jsonDoc["ki"];
        robot.rightPIDspeed.ki = ki;
      }
      if (jsonDoc["coulombFriction"].is<float>()){
        float coulombFriction = jsonDoc["coulombFriction"];
        robot.rightPIDspeed.coulombFriction = coulombFriction;
      }
      if (jsonDoc["rateUp"].is<float>()){
        float rateUp = jsonDoc["rateUp"];
        robot.rightRateLimiter.rateUp = rateUp;
      }
      if (jsonDoc["rateDown"].is<float>()){
        float rateDown = jsonDoc["rateDown"];
        robot.rightRateLimiter.rateDown = rateDown;
      }
    }
    else if (strcmp(topic, getTopic(mqtt_controller_speed_command_setpoint).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      float leftSetPoint = jsonDoc["left"];
      float rightSetPoint = jsonDoc["right"];
      robot.leftPIDspeed.setPoint = leftSetPoint;
      robot.rightPIDspeed.setPoint = rightSetPoint;
    
    //Controller: position (set, command)
    }
    else if (strcmp(topic, getTopic( mqtt_controller_position_left_set).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      
      if (jsonDoc["kp"].is<float>()){
        float kp = jsonDoc["kp"];
        robot.leftPIDposition.kp = kp;
      }
      if (jsonDoc["ki"].is<float>()){
        float ki = jsonDoc["ki"];
        robot.leftPIDposition.ki = ki;
      }
    }
    else if (strcmp(topic, getTopic(mqtt_controller_position_right_set).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      if (jsonDoc["kp"].is<float>()){
        float kp = jsonDoc["kp"];
        robot.rightPIDposition.kp = kp;
      }
      if (jsonDoc["ki"].is<float>()){
        float ki = jsonDoc["ki"];
        robot.rightPIDposition.ki = ki;
      }
    }
    else if (strcmp(topic, getTopic(mqtt_controller_position_command_setpoint).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      float leftSetPoint = jsonDoc["left"];
      float rightSetPoint = jsonDoc["right"];
      robot.leftPIDposition.setPoint = leftSetPoint;
      robot.rightPIDposition.setPoint = rightSetPoint;
    }
    //Actuator: motor (set duty cycle)
    else if (strcmp(topic, getTopic(mqtt_actuator_motor_set).c_str()) == 0) {
      JsonDocument jsonDoc = readAsJson(message);
      // Leer los valores de "left" y "right" del JSON
      float left = jsonDoc["left"]["dutyCycle"];
      float right = jsonDoc["right"]["dutyCycle"];
      robot.leftMotor.dutyCycle = left;
      robot.rightMotor.dutyCycle = right;
    }
    //Navigation: path, command
    else if(strcmp(topic, getTopic(mqtt_navigation_path_set).c_str()) == 0){

    }
    else if(strcmp(topic, getTopic(mqtt_navigation_command).c_str()) == 0){
      JsonDocument jsonDoc = readAsJson(message);
      if (jsonDoc["action"].is<String>() && jsonDoc["value"].is<float>()){
        String action = jsonDoc["action"];
        float value = jsonDoc["value"];
        Serial.println(value);
        processAction(action, value);
      } else {
        Serial.println("Bad request. Invalid message definition");
      }

    }
    else {
      Serial.println("Unknown topic received.");
    }
  }

// Publishing methods
char* floatToChar(float value, char* buffer){
  dtostrf(value, 5, 2, buffer);
  return buffer;
}

char* intTochar(int value, char* buffer){
  itoa(value, buffer, 10);
  return buffer;
}

char* poseToJson(Pose pose, char* buffer){
  snprintf(buffer, sizeof(buffer),
           "{\"row\":%d,\"col\":%d,\"orientation\":\"%c\"}",
           pose.row, pose.col, pose.orientation);
  return buffer;
}

char* PISpeedDataToJson(PIspeed piSpeed, char* buffer, size_t buffer_size){
  snprintf(buffer, buffer_size,
           "{\"setPoint\":%.2f,\"currentSpeed\":%.2f,\"integralError\":%.2f}",
           piSpeed.setPoint, piSpeed.currentSpeed, piSpeed.integralError);
  return buffer;
}

char* PISpeedStateToJson(PIspeed piSpeed, RateLimiter rateLimiter, char* buffer) {
  snprintf(buffer, sizeof(buffer),
           "{\"kp\":%.2f, \"ki\":%.2f, \"coulombFriction\":%d, \"rateUp\":%.2f, \"rateDown\":%.2f}",
           piSpeed.kp, piSpeed.ki, piSpeed.coulombFriction, rateLimiter.rateUp, rateLimiter.rateDown);
  return buffer;
}

char* PIPositionDataToJson(PIposition piPos, char* buffer){
  snprintf(buffer, sizeof(buffer),
           "{\"setPoint\":%.2f,\"currentPosition\":%.2f,\"integralError\":%.2f}",
           piPos.setPoint, piPos.currentPosition, piPos.integralError);
  return buffer;
}

char* PIPositionStateToJson(PIposition piPos, char* buffer) {
  snprintf(buffer, sizeof(buffer),
           "{\"kp\":%.2f, \"ki\":%.2f, \"minSpeed\":%d}",
           piPos.kp, piPos.ki, piPos.minSpeed);
  return buffer;
}
//Encoder: state, odometry 
void publishEncoderState(){

}
void publishEncoderOdometry(){

}
//Ultrasonic: state, data raw, obstacles
void publishUltrasonicstate(){

}

void publishUltrasonicObstacles(){

}
//RFID: state, data, raw 
void publishRfidState(){}
void publisRfidData(){}
void publishRfidRaw(){}

//Controller: speed (state, data)
void publishControllerSpeedState(){
  char leftBuffer [100];
  char rightBuffer [100];
  client.publish(getTopic(mqtt_controller_speed_left_state).c_str(), PISpeedStateToJson(robot.leftPIDspeed, robot.leftRateLimiter, leftBuffer));
  client.publish(getTopic(mqtt_controller_speed_right_state).c_str(), PISpeedStateToJson(robot.rightPIDspeed, robot.rightRateLimiter, rightBuffer));

}
void publishControllerSpeedData(){
  char leftBuffer [150];
  char rightBuffer [150];
  client.publish(getTopic(mqtt_controller_speed_left_data).c_str(), PISpeedDataToJson(robot.leftPIDspeed, leftBuffer, sizeof(leftBuffer)));
  client.publish(getTopic(mqtt_controller_speed_right_data).c_str(), PISpeedDataToJson(robot.rightPIDspeed, rightBuffer, sizeof(rightBuffer)));
}
//Controller: position (state, data)
void publishControllerPositionState(){
  char leftBuffer [100];
  char rightBuffer [100];
  client.publish(getTopic(mqtt_controller_position_left_state).c_str(), PIPositionStateToJson(robot.leftPIDposition, leftBuffer));
  client.publish(getTopic(mqtt_controller_position_right_state).c_str(), PIPositionStateToJson(robot.rightPIDposition, rightBuffer));
}
void publishControllerPositionData(){
  char leftBuffer [100];
  char rightBuffer [100];
  client.publish(getTopic(mqtt_controller_position_left_state).c_str(), PIPositionDataToJson(robot.leftPIDposition, leftBuffer));
  client.publish(getTopic(mqtt_controller_position_right_state).c_str(), PIPositionDataToJson(robot.rightPIDposition, rightBuffer));

}
//Navigation: state
void publishNavState(Pose pose){
  char buffer[50];
  client.publish(getTopic(mqtt_navigation_path_state).c_str(), poseToJson(pose, buffer));
}

//Robot: pose, state, mode
void publishCurrentMode(){
  char buffer[50];
  client.publish(getTopic(mqtt_mode).c_str(), intTochar(robot.mode, buffer));
}
void publishCurrentPose(){
  char buffer[50];
  client.publish(getTopic(mqtt_pose).c_str(), poseToJson(robot.pose, buffer));
}

void publishState(){
  char buffer[3];
  client.publish(getTopic(mqtt_state).c_str(), intTochar(robot.state, buffer));
}
  
  
  