// Minimal USB <-> iRobot Create 1 serial proxy with LED direction indicators
#include <Arduino.h>
#include "leds.h"

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// AutonomyLab create_1 drivers expect 57600 8N1 on the robot link
static const unsigned long CREATE_BAUD = 57600;

// iRobot Create Open Interface commands
static const uint8_t OI_START   = 128;
static const uint8_t OI_SAFE    = 131;   // not used anymore (conflicted with midbrain)
static const uint8_t OI_FULL    = 132;   // not used in proxy-only mode
static const uint8_t OI_SENSORS = 142;   // not used; avoid tickling robot

// Simple connect/retry state
static bool robotSeen = false;               // any bytes ever received from robot
static bool midbrainActive = false;          // any host->robot traffic observed
static bool startSent = false;               // sent a single OI_START after grace
static unsigned long bootMs = 0;             // millis at setup
static const unsigned long START_GRACE_MS = 4000; // wait before sending START once

void setup() {
  // Host-side USB CDC: baud often ignored, but set for clarity
  Serial.begin(CREATE_BAUD);
  // Robot-side UART
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  // LEDs start off
  initLeds();
  // Do not send anything immediately; give the midbrain a chance to init first.
  bootMs = millis();
}

void loop() {
  bool toRobot = false;
  bool fromRobot = false;
  // Host -> Robot
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    CREATE_SERIAL.write((uint8_t)c);
    toRobot = true;
  }
  // Robot -> Host
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
    fromRobot = true;
  }
  if (toRobot) midbrainActive = true;
  if (fromRobot) robotSeen = true;

  // After a short grace period with no host activity, send a single OI_START
  // to put the robot in passive OI mode without asserting SAFE/FULL.
  unsigned long now = millis();
  if (!startSent && !midbrainActive && (now - bootMs >= START_GRACE_MS)) {
    CREATE_SERIAL.write(OI_START);
    startSent = true;
    toRobot = true;
  }
  // LED policy: left = robot sending, right = robot receiving
  if (toRobot && fromRobot) {
    setLeds(true, true);
  } else if (toRobot) {
    setLeds(false, true);
  } else if (fromRobot) {
    setLeds(true, false);
  } else {
    setLeds(false, false);
  }
}
