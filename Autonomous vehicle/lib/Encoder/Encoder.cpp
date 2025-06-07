#include "Encoder.h"
#include <FilesManager.h>

EncoderDriver* EncoderDriver::instances[NUM_DIGITAL_PINS] = { nullptr };

EncoderDriver::EncoderDriver()
    : pinA(-1), pinB(-1), ppr(-1), lastAState(0), direction(true) {
        mux = portMUX_INITIALIZER_UNLOCKED; // Initialize the mutex
    }


void EncoderDriver::loadConfig(const char* configPath, const char* encoderName) {
    // Load motor configuration from JSON file
    JsonDocument doc;
    if (FilesManager::readFileAsJson(doc, configPath)) {
        if (!doc[encoderName]["pinA"].isNull() && !doc[encoderName]["pinB"].isNull() && 
            !doc[encoderName]["ppr"].isNull()) {

            pinA = doc[encoderName]["pinA"];
            pinB = doc[encoderName]["pinB"];
            ppr = doc[encoderName]["ppr"];

            Serial.println("Encoder configuration loaded successfully.");
        } else {
            Serial.println("JSON missing required keys.");
        }
    } else {
        Serial.print("Failed to load encoder ");
        Serial.print(encoderName);
        Serial.println("configuration!");
    }
}    
void EncoderDriver::begin() {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    lastAState = digitalRead(pinA);

    instances[pinA] = this;

    attachInterrupt(digitalPinToInterrupt(pinA), EncoderDriver::globalInterruptHandler, CHANGE);
}

void EncoderDriver::handleInterrupt() {
    int aState = digitalRead(pinA);
    int bState = digitalRead(pinB);

    if (lastAState == LOW && aState == HIGH) {
        direction = (bState == HIGH);
    }
    lastAState = aState;
    pulses += direction ? 1 : -1; // Increment or decrement count based on direction
    accumulatedPulses += direction ? 1 : -1; // Increment or decrement total pulses    
}

void EncoderDriver::globalInterruptHandler() {
    // This will be called by *any* interrupt; dispatch appropriately.
    for (uint8_t pin = 0; pin < NUM_DIGITAL_PINS; ++pin) {
        if (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT && instances[pin]) {
            instances[pin]->handleInterrupt();
        }
    }
}

long EncoderDriver::getCount() const {
    portENTER_CRITICAL(&mux);
    long count = pulses;
    portEXIT_CRITICAL(&mux);
    return count;
}

void EncoderDriver::resetIterationPulses() {
    portENTER_CRITICAL(&mux);
    pulses = 0;
    portEXIT_CRITICAL(&mux);
}

void EncoderDriver::reset() {
    portENTER_CRITICAL(&mux);
    accumulatedPulses = 0;
    portEXIT_CRITICAL(&mux);
}

float EncoderDriver::getAngularPosition () const { // Quizas los pulses no funcionen bien aqui
    float radiansPerStep = (2 * PI) / ppr; //360º = 2PIrad / pulsesEncoder --> radians that advance each pulse
    // We divide by 2 because the encoder counts both edges of the signal
    long p;
    portENTER_CRITICAL(&mux);
    p = accumulatedPulses;
    portEXIT_CRITICAL(&mux);
    return (p/2 * radiansPerStep); 
}
float EncoderDriver::getAngularSpeed(float deltaTime) const {
    float radiansPerStep = (2 * PI) / ppr; // 360º = 2PIrad / pulsesEncoder --> radians that advance each pulse
    long p;
    portENTER_CRITICAL(&mux);
    p = pulses;
    portEXIT_CRITICAL(&mux);
    return (p/2*radiansPerStep) / deltaTime; // angular speed
}
