#include <Arduino.h>
#include "behavior.h"
#include "leds.h"
#include "sensors.h"
#include <utils.h>

void setup() {
  // Bring up USB CDC for debugging
  Serial.begin(115200);
  delay(50);
  Serial.println("[BOOT] brainstem-pm starting");
  initializeBehavior();
  initLeds();
}

void loop() {
  // Always poll OI sensor stream to keep caches fresh
  updateSensorStream();
  updateBehavior();    // one tick per loop
  enforceRobotWatchdog(); // ensure stop if control loop stalls
  updateLeds();      // drive LED patterns non-blocking
}
