#include <Arduino.h>
#include "behavior.h"

void setup() {
  initializeBehavior();
}

void loop() {
  updateBehavior();  // one tick per loop
}
