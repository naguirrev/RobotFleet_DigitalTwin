#include<Arduino.h>

//STRUCTS
struct WifiCredentials {
    char* ssid;
    char* password;
};

struct Mqtt
{
    char* server;
    int port;
    char* user;
    char* password;
};

//CONSTANTS
//Mqtt topics 
//---Read topics
const char* mqtt_topc_config = "smr/config/car1";
const char* mqtt_topic_speed = "smr/speed/car1"; //car speed

//---Write topics
//-----Speed control
const char* mqtt_topic_leftKp = "smr/lKp/car1";
const char* mqtt_topic_leftKi = "smr/lKi/car1";
const char* mqtt_topic_rightKp = "smr/rKp/car1";
const char* mqtt_topic_rightKi = "smr/rKi/car1";
const char* mqtt_topic_duty= "smr/duty/car1";
const char* mqtt_topic_angularSpeed = "smr/angularSpeed/car1";

//-----Position control
const char* mqtt_topic_leftPosKp = "smr/lPosKp/car1";
const char* mqtt_topic_leftPosKi = "smr/lPosKi/car1";
const char* mqtt_topic_rightPosKp = "smr/rPosKp/car1";
const char* mqtt_topic_rightPosKi = "smr/rPosKi/car1";
const char* mqtt_topic_angularPos = "smr/angularPos/car1";

const char* mqtt_topic_mode = "smr/mode/car1";


//FUNCTIONS
void setupWifi();
void initMqtt();
void setupMqtt();
void ensureMqttConnected();
void publishSpeedMqtt(float leftCurrentSpeed, float rightCurrentSpeed);
void processMqttMessage(char* topic, byte* payload, unsigned int length);