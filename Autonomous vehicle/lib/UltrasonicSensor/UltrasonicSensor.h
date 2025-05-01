#include <cstdint>
#pragma once

class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    void begin();
    float getDistanceCM();

private:
    uint8_t trig;  // Output pin for sending the ultrasonic pulse (TRIG)
    uint8_t echo;  // Input pin for receiving the echo signal (ECHO)

    // Speed of sound in air at room temperature (~20°C),
    // expressed in centimeters per microsecond (cm/µs).
    // Used to convert the time-of-flight of the pulse into distance.
    static constexpr float SPEED_OF_SOUND = 0.0343f;

    // Maximum wait time to receive the echo signal (in microseconds).
    // If exceeded, it is considered a timeout (no obstacle detected).
    static constexpr unsigned long PULSE_TIMEOUT = 25000;
};

