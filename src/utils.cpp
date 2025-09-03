#include "utils.h"
#include "motion.h"
#include <Arduino.h>

void initConnection() {
  // startup handshakes
}

void delayBriefly() {
  delay(100); // simple pause
}

void randomWiggle() {
  if (random(2) == 0) turnLeftOneTick();
  else turnRightOneTick();
}

void turnRandomly() {
  if (random(2) == 0) {
    turnLeftOneTick();
    delay(200);
  } else {
    turnRightOneTick();
    delay(200);
  }
}
