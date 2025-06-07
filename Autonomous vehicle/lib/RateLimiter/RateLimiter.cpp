#include "RateLimiter.h"
#include <ArduinoJson.h>
#include <FilesManager.h>


RateLimiter::RateLimiter() : rateUp(0), rateDown(0), previousOutput(0) {}

/**
 * This function loads the rate limiter configuration from a JSON file.
 * It reads the rate limits (rateUp and rateDown) for the specified rate limiter name.
 *
 * @param configFile The path to the configuration file.
 * @param rateLimiterName The name of the rate limiter in the JSON file.
 */
void RateLimiter::loadConfig(const char* configFile, const char* rateLimiterName) {
    JsonDocument doc;
    bool result = FilesManager::readFileAsJson(doc, configFile);
    if (result) {
        // Load rate limits from the JSON file
        rateUp = doc[rateLimiterName]["rateUp"].as<float>();
        rateDown = doc[rateLimiterName]["rateDown"].as<float>();
        Serial.print("Rate Limiter configuration ");
        Serial.print(rateLimiterName);
        Serial.println(" loaded");
    } else {
        Serial.println("Failed to load Rate Limiter ");
        Serial.print(rateLimiterName);
        Serial.println(" configuration!");
    }
}
/**
 * This function limits the rate of change of an input signal.
 * It ensures that the output does not increase or decrease faster
 * than the specified rate limits (rateUp and rateDown).
 *
 * @param rl A pointer to the RateLimiter structure containing rate limits and previous output.
 * @param input The current input value to be rate-limited.
 * @return The rate-limited output value.
 */
float RateLimiter::updateRateLimiter(float input, float loopTime) {
    // Calculate the maximum allowed increase and decrease based on the rate limits
    float maxIncrease = rateUp * loopTime;      // Maximum allowed increase per loop cycle
    float maxDecrease = rateDown * loopTime;    // Maximum allowed decrease per loop cycle
    
    // Compute the proposed change based on the input and previous output
    float proposedChange = input - previousOutput; 

    if (proposedChange > maxIncrease) {
        previousOutput += maxIncrease;  // Restrict increase to max allowed rate
    } else if (proposedChange < -maxDecrease) {
        previousOutput -= maxDecrease;  // Restrict decrease to max allowed rate
    } else {
        previousOutput = input;         // Apply input directly if within limits
    }

    return previousOutput;
}
