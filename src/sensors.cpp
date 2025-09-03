#include "sensors.h"
#include <Arduino.h>

void initSensors() {
  pinMode(3, INPUT); // bumper
  pinMode(A0, INPUT); // cliff sensor
}

int scanEnvironment() {
  // Simulated: add IR/ultrasonic later
  return 0;
}

bool bumperTriggered() {
  return digitalRead(3) == LOW;
}

bool cliffDetected() {
  return analogRead(A0) < 200;
}
