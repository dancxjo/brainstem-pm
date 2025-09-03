#include <Arduino.h>
#include "behavior.h"
#include "leds.h"

void setup() {
  initializeBehavior();
  initLeds();
}

void loop() {
  updateBehavior();  // one tick per loop
  updateLeds();      // drive LED patterns non-blocking
}
