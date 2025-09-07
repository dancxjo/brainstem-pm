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
static const uint8_t OI_SAFE    = 131;
static const uint8_t OI_FULL    = 132;
static const uint8_t OI_SENSORS = 142; // followed by packet id

// Simple connect/retry state
static bool robotSeen = false;               // any bytes ever received from robot
static unsigned long lastPokeMs = 0;        // last time we tried to init
static const unsigned long POKE_PERIOD_MS = 500; // retry interval until robotSeen

void setup() {
  // Host-side USB CDC: baud often ignored, but set for clarity
  Serial.begin(CREATE_BAUD);
  // Robot-side UART
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  // LEDs start off
  initLeds();
  // Immediately attempt to grab control so the robot is ready
  // Send START + FULL, and probe a simple sensor to solicit a response.
  CREATE_SERIAL.write(OI_START);
  CREATE_SERIAL.write(OI_FULL);
  CREATE_SERIAL.write(OI_SENSORS);
  CREATE_SERIAL.write((uint8_t)7); // bump/wheeldrop (1 byte)
  lastPokeMs = millis();
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
  if (fromRobot) robotSeen = true;

  // While we haven't heard anything from the robot yet, keep trying to
  // initialize and solicit a response at a gentle cadence.
  unsigned long now = millis();
  if (!robotSeen && (now - lastPokeMs >= POKE_PERIOD_MS)) {
    CREATE_SERIAL.write(OI_START);
    CREATE_SERIAL.write(OI_FULL);
    CREATE_SERIAL.write(OI_SENSORS);
    CREATE_SERIAL.write((uint8_t)7);
    lastPokeMs = now;
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
