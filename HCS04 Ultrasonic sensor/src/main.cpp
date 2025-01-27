#include <Arduino.h>

#define TRIG_PIN 19   // Pin TRIG
#define ECHO_PIN 18   // Pin ECHO

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

  // Calcular la distancia (velocidad del sonido = 343 m/s)
  long distance = (duration * 0.0343) / 2;  // Dividir entre 2 porque el pulso viaja de ida y vuelta

  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);  // Espera medio segundo antes de la siguiente medición
}

