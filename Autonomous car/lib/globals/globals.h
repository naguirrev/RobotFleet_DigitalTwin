#include<ArduinoJson.h>

extern int mode;
extern bool publish;

//CONSTANTS
enum Orientation {
    NORTH,
    SOUTH,
    EAST,
    WEST
  };

enum RobotState {
    IDLE,
    NAVIGATING,
    PERFORMING_TASK,
    OBSTACLE_DETECTED,
    FAIL
};

enum ControlMode{
    STOP,
    OPEN_LOOP,
    SPEED_CONTROLLER,
    POSITION_CONTROLLER,
    NAVIGATION
};

enum BaseCell{
    EMPTY, 
    TASK,
    OBSTACLE
};

enum StateCell{
    FREE,
    RESERVED,
    OCUPPIED
};

enum TaskState{
    PENDING, 
    ASSIGNED, 
    IN_PROGRESS,
    COMPLETED,
    ERROR
};

//FUNCTIONS
void setMode(int m);

bool readFileAsJson(JsonDocument doc, const char* filename);