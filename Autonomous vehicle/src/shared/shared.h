#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define MAX_ACTIONS 100

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

typedef struct {
    String action;  
    float value;  
    bool completed = false;
} Action;
extern QueueHandle_t actionQueue;


extern bool EmergencyStop;
extern SemaphoreHandle_t emergencyStopMutex;

int sgn(float value);
