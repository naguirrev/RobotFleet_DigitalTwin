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
