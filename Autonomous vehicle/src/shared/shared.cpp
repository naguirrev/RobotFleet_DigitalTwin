#include "shared.h"

ObstacleData obstacle = {0, 0, 0};
SemaphoreHandle_t obstacleMutex = xSemaphoreCreateMutex();

LocalizationData localization = {0, 0};
SemaphoreHandle_t localizationMutex = xSemaphoreCreateMutex();
