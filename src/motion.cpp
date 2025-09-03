#include "motion.h"
#include <Arduino.h>

void initMotors() {
  // pinMode setup for motor drivers
}

void forwardOneTick() {
  // digitalWrite to go forward briefly
}

void backwardOneTick() {
  // digitalWrite to go backward
}

void turnLeftOneTick() {
  // spin in place or curve left
}

void turnRightOneTick() {
  // spin in place or curve right
}

void stopAllMotors() {
  // all LOW or brake
}

void alertFreeze() {
  tone(9, 440, 100); // buzzer tone
}
