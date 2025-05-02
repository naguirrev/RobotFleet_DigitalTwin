#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    float x;
    float y;
    float distance;
} ObstacleData;

extern ObstacleData obstacle;
extern SemaphoreHandle_t obstacleMutex;


typedef struct {
    int row;
    int col;
} LocalizationData;
extern LocalizationData localization;
extern SemaphoreHandle_t localizationMutex;

typedef struct {
    float left;
    float right;
} MotorSpeed;
extern MotorSpeed motorSpeed;
extern SemaphoreHandle_t motorSpeedMutex;

typedef struct {
    float left;
    float right;
} AngularPosition;
extern AngularPosition angularPos;
extern SemaphoreHandle_t angularPosMutex;

typedef enum {
    STOP,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} NavigationOrder;
extern NavigationOrder currentOrder;
extern SemaphoreHandle_t orderMutex;
