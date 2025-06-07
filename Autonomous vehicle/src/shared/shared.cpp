#include "shared.h"

ObstacleData obstacle = {0, 0, 0};
SemaphoreHandle_t obstacleMutex = xSemaphoreCreateMutex();

LocalizationData localization = {0, 0};
SemaphoreHandle_t localizationMutex = xSemaphoreCreateMutex();

MotorSpeed motorSpeed = {0, 0};
SemaphoreHandle_t motorSpeedMutex = xSemaphoreCreateMutex();

AngularPosition angularPos = {0, 0};
SemaphoreHandle_t angularPosMutex = xSemaphoreCreateMutex();

// Action queue for task communication and motor control task. 
QueueHandle_t actionQueue = xQueueCreate(MAX_ACTIONS, sizeof(Action));

//Emergency stop flag when stop command is received or obstacle detected
bool EmergencyStop = false;
SemaphoreHandle_t emergencyStopMutex = xSemaphoreCreateMutex();

int sgn(float value) {
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}


