#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <globals.h>
#include <mqtt.h>


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
      wifi = {doc["ssid"].as<char*>(), doc["password"].as<char*>()};
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
void initMqtt()
  {
    JsonDocument doc;  
    bool result = readFileAsJson(doc, mqttConfigFile);
    if (result){
      mqtt = {doc["server"].as<char*>(), doc["port"].as<int>(), doc["user"].as<char*>(), doc["password"].as<char*>()};
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
void reconnectMqtt() {
    while (!client.connected()) {
      Serial.print("Connecting to MQTT...");
      if (client.connect("ESP32Client", mqtt.user, mqtt.password)) {
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
      JsonDocument jsonDoc = readAsJson(message);
      // Leer los valores de "left" y "right" del JSON
      float left = jsonDoc["left"];
      float right = jsonDoc["right"];
      robot.leftMotor.dutyCycle = left;
      robot.rightMotor.dutyCycle = right;
    }
    else if (strcmp(topic, mqtt_topic_angularSpeed) == 0) {
      // Interpretar el payload como JSON y extraer valores de "left" y "right"
      JsonDocument jsonDoc = readAsJson(message);
      // Leer payload como float y establecer la velocidad angular
      float left = jsonDoc["left"];
      float right = jsonDoc["right"];
      robot.leftPIDspeed.setPoint = left;
      robot.rightPIDspeed.setPoint = right;
    }else if (strcmp(topic, mqtt_topic_leftKp) == 0) {
      float value = atof(message);
      robot.leftPIDspeed.kp = value;
    }else if (strcmp(topic, mqtt_topic_leftKi) == 0) {
      float value = atof(message);
      robot.leftPIDspeed.ki = value;
    }else if (strcmp(topic, mqtt_topic_rightKp) == 0) {
      float value = atof(message);
      robot.rightPIDspeed.kp = value;
    }else if (strcmp(topic, mqtt_topic_rightKi) == 0) {
      float value = atof(message);
      robot.rightPIDspeed.ki = value;
    }else if (strcmp(topic, mqtt_topic_angularPos) == 0) {
      // Interpretar el payload como JSON y extraer valores de "left" y "right"
      JsonDocument jsonDoc = readAsJson(message);
      // Leer payload como float y establecer la velocidad angular
      float left = jsonDoc["left"];
      float right = jsonDoc["right"];
      robot.leftPIDposition.setPoint = left;
      robot.rightPIDposition.setPoint = right;
    }else if (strcmp(topic, mqtt_topic_leftPosKp) == 0) {
      float value = atof(message);
      robot.leftPIDposition.kp = value;
    }else if (strcmp(topic, mqtt_topic_leftPosKi) == 0) {
      float value = atof(message);
      robot.leftPIDposition.ki = value;
    }else if (strcmp(topic, mqtt_topic_rightPosKp) == 0) {
      float value = atof(message);
      robot.rightPIDposition.kp = value;
    }else if (strcmp(topic, mqtt_topic_rightPosKi) == 0) {
      float value = atof(message);
      robot.rightPIDposition.ki = value;
    }
    else {
      Serial.println("Unknown topic received.");
    }
  }
void publishSpeedMqtt(float leftCurrentSpeed, float rightCurrentSpeed){
    String speedMessage = "{";
    speedMessage += "\"leftSpeed\": " + String(leftCurrentSpeed, 2) + ",";
    speedMessage += "\"rightSpeed\": " + String(rightCurrentSpeed, 2);
    speedMessage += "}";
  
    // Publish JSON message
    client.publish(mqtt_topic_speed, speedMessage.c_str());
  }
  
  
  