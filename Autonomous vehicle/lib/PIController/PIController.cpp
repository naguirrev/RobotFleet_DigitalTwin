#include "PIController.h"

/**
 * This function updates the PI controller for a given setpoint and current value.
 * It calculates the error, accumulates the integral error, and computes the control output.
 * The integral error is also updated to prevent windup.
 */
PIControllerResult updatePIController(float setPoint, float currentValue, float integralError, float kp, float ki) {
    //Calculate error
    float error = setPoint - currentValue;
    //Accumulate integral error
    integralError += error;
    //Calculate control output
    float controlOutput = (kp * error) + (ki * integralError);
    return {controlOutput, integralError};
}