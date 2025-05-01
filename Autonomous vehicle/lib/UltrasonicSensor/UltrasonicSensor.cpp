#include <UltrasonicSensor.h>
#include <Arduino.h>

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
    : trig(trigPin), echo(echoPin) {}

void UltrasonicSensor::begin() {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
}

float UltrasonicSensor::getDistanceCM() {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, PULSE_TIMEOUT); 
    return (duration * SPEED_OF_SOUND) / 2;
}
