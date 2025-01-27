#include <Arduino.h>

// put function declarations here:
float i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  i += 0.1;
  Serial.print(">Num:");
  Serial.println(i);

  delay(50);
}

