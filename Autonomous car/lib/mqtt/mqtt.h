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
//Base topic
constexpr char* mqtt_base = "rfm/fleet"; //Robot fleet manager -> fleet


//---READ TOPICS
//Control mode
constexpr char* mqtt_mode = "/mode"; //Control mode - speed controller, position controller, navigation control

//Sensors
constexpr char* mqtt_encoder_left_set = "/encoder/left/set";
constexpr char* mqtt_encoder_right_set = "/encoder/right/set";
constexpr char* mqtt_ultrasonic_set = "/ultrasonic/set";
constexpr char* mqtt_rfid_set = "/rfid/set";

//Controllers
constexpr char* mqtt_controller_speed_left_set = "/controllers/speed/left/set";
constexpr char* mqtt_controller_speed_right_set = "/controllers/speed/right/set";
constexpr char* mqtt_controller_speed_command_setpoint = "/controllers/speed/command/setpoint";
constexpr char* mqtt_controller_position_left_set = "/controllers/position/left/set";
constexpr char* mqtt_controller_position_right_set = "/controllers/position/right/set";
constexpr char* mqtt_controller_position_command_setpoint = "/controllers/position/command/setpoint";

//Actuators
constexpr char* mqtt_actuator_motor_set = "/controller/actuator/motor/set";

//Navigation
constexpr char* mqtt_navigation_path_set = "/navigation/path/set";
constexpr char* mqtt_navigation_command = "/navigation/command";


//---WRITE TOPICS

//Encoder
constexpr char* mqtt_encoder_left_state = "/encoder/left/state";
constexpr char* mqtt_encoder_left_data_raw = "/encoder/left/data/raw";
constexpr char* mqtt_encoder_left_data_odometry = "/encoder/left/data/odometry";
constexpr char* mqtt_encoder_right_state = "/encoder/right/state";
constexpr char* mqtt_encoder_right_data_raw = "/encoder/right/data/raw";
constexpr char* mqtt_encoder_right_data_odometry = "/encoder/right/data/odometry";

//Ultrasonic
constexpr char* mqtt_ultrasonic_state = "/ultrasonic/state";
constexpr char* mqtt_ultrasonic_data_raw = "/ultrasonic/data/raw";
constexpr char* mqtt_ultrasonic_data_obstacle = "/ultrasonic/data/obstacle";

//RFID
constexpr char* mqtt_rfid_state = "/rfid/state";
constexpr char* mqtt_rfid_data_raw = "/rfid/data/raw";
constexpr char* mqtt_rfid_data_location = "/rfid/data/location";


//Speed controller
constexpr char* mqtt_controller_speed_left_state = "/controller/speed/left/state";
constexpr char* mqtt_controller_speed_left_data = "/controller/speed/left/data/raw";
constexpr char* mqtt_controller_speed_right_state = "/controller/speed/right/state";
constexpr char* mqtt_controller_speed_right_data = "/controller/speed/right/data/raw";

//Position controller
constexpr char* mqtt_controller_position_left_state = "/controller/position/left/state";
constexpr char* mqtt_controller_position_left_data_raw = "/controller/position/left/data/raw";
constexpr char* mqtt_controller_position_left_data_odometry = "/controller/position/left/data/odometry";
constexpr char* mqtt_controller_position_right_state = "/controller/position/right/state";
constexpr char* mqtt_controller_position_right_data_raw = "/controller/position/right/data/raw";
constexpr char* mqtt_controller_position_right_data_odometry = "/controller/position/right/data/odometry";

//Actuator
constexpr char* mqtt_actuator_motor_left_state = "/actuator/motor/left/state";
constexpr char* mqtt_actuator_motor_right_state = "/actuator/motor/right/state";

//High level functions
constexpr char* mqtt_pose = "/pose";
constexpr char* mqtt_state = "/state";
constexpr char* mqtt_navigation_path_state = "/navigation/path/state";


//FUNCTIONS
void setupWifi();
void initMqtt();
void setupMqtt();
void ensureMqttConnected();
//Read mqtt topics
void processMqttMessage(char* topic, byte* payload, unsigned int length);

//Publish on mqtt topics
void publishEncoderState();
void publishEncoderOdometry();
void publishUltrasonicstate();
void publishUltrasonicObstacles();
void publishRfidState();
void publisRfidData();
void publishRfidRaw();
void publishControllerSpeedState();
void publishControllerSpeedData();
void publishControllerPositionState();
void publishControllerPositionData();
void publishCurrentPose();
void publishState();