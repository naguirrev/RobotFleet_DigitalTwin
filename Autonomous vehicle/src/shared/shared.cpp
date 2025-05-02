#include "shared.h"

ObstacleData obstacle = {0, 0, 0};
SemaphoreHandle_t obstacleMutex = xSemaphoreCreateMutex();

LocalizationData localization = {0, 0};
SemaphoreHandle_t localizationMutex = xSemaphoreCreateMutex();

MotorSpeed motorSpeed = {0, 0};
SemaphoreHandle_t motorSpeedMutex = xSemaphoreCreateMutex();

AngularPosition angularPos = {0, 0};
SemaphoreHandle_t angularPosMutex = xSemaphoreCreateMutex();

NavigationOrder currentOrder = STOP;
SemaphoreHandle_t orderMutex = xSemaphoreCreateMutex();


