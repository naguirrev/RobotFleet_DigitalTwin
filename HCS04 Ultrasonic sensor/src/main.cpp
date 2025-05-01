#include <Arduino.h>

#define TRIG_PIN 17   // Pin TRIG
#define ECHO_PIN 16   // Pin ECHO
#define MIN_DISTANCE 3
#define MAX_DISTANCE 7

void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Enviar el pulso de activación
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Leer el pulso de retorno
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calcular la distancia
  long distance = (duration * 0.0343) / 2;

  // Detección de obstáculos
  if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
    Serial.print("Obstacle detected at ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(500);  // Esperar medio segundo antes de medir otra vez
}