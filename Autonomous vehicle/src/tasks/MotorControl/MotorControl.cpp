#include <motors.h>
#include <shared.h>

bool isExecuting = false;

TickType_t lastControlTime = xTaskGetTickCount();

void MotorControlTask(void* parameter){
    TickType_t lastWake = xTaskGetTickCount();
    Action currentAction;
    bool hasAction = false;

    for(;;) {
        TickType_t currentTime = xTaskGetTickCount();
        float deltaTime = (currentTime - lastControlTime) * portTICK_PERIOD_MS  / 1000.0; // Convert to seconds
        updateAngularSpeed(deltaTime);
        updateAngularPosition();
        //Control mode
        updatePIpositionController();
        updatePIspeedController();
        //Reset encoder pulses for the next angular speed calculation
        resetEncodersPulses();

        logRobotControlInfoToSerial();
        
        // Check for emergency stop
        if (xSemaphoreTake(emergencyStopMutex, 0) == pdTRUE) {
            if (EmergencyStop) {
                Serial.println("Emergency stop activated");
                processAction("stop", 0);
                EmergencyStop = false;
                //Reset action queue
                while (xQueueReceive(actionQueue, &currentAction, 0)) {
                    currentAction.completed = true;
                }
                isExecuting = false;
                hasAction = false;
            }
            xSemaphoreGive(emergencyStopMutex);
        }
        // Process action queue
        if(!isExecuting && !hasAction){
            if(xQueueReceive(actionQueue, &currentAction, 0)){
                processAction(currentAction.action, currentAction.value);
                isExecuting = true;
                hasAction = true;
            }
        } else if (isExecuting && actionCompleted()){
            isExecuting = false;
            hasAction = false;
        }     

        lastControlTime = currentTime;
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10))
    }
}

void initMotorControl(){
    
    initRobot();
    motorsInit();
    encodersInit();

   //Create the task
    xTaskCreatePinnedToCore(
        MotorControlTask,       // Function to implement the task
        "MotorControlTask",     // Name of the task
        2048,                   // Stack size in words
        NULL,                   // Task input parameter
        1,                      // Priority of the task
        NULL,                   // Task handle
        1                       // Core where the task should run
    );
}