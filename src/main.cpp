// Minimal USB <-> iRobot Create 1 serial proxy with LED direction indicators
#include <Arduino.h>
#include "leds.h"

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// AutonomyLab create_1 drivers expect 57600 8N1 on the robot link
static const unsigned long CREATE_BAUD = 57600;

// iRobot Create Open Interface opcodes (subset)
static const uint8_t OI_START   = 128; // enter passive
static const uint8_t OI_BAUD    = 129; // set baud (1 arg)
static const uint8_t OI_SAFE    = 131; // safe mode
static const uint8_t OI_FULL    = 132; // full mode
static const uint8_t OI_DRIVE_DIRECT = 145; // 4 args

void setup() {
  // Host-side USB CDC: baud often ignored, but set for clarity
  Serial.begin(CREATE_BAUD);
  // Robot-side UART
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  // LEDs start off
  initLeds();
  // Aggressively claim control so robot stays awake until midbrain boots.
  // Minimal init: START -> SAFE and stop motors.
  CREATE_SERIAL.write(OI_START);
  CREATE_SERIAL.write(OI_SAFE);
  CREATE_SERIAL.write(OI_DRIVE_DIRECT);
  CREATE_SERIAL.write((uint8_t)0); CREATE_SERIAL.write((uint8_t)0); // right = 0
  CREATE_SERIAL.write((uint8_t)0); CREATE_SERIAL.write((uint8_t)0); // left = 0
}

void loop() {
  bool toRobot = false;
  bool fromRobot = false;
  static uint8_t discardN = 0; // bytes to discard after an intercepted opcode
  // Host -> Robot
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    uint8_t b = (uint8_t)c;
    // Intercept host init/reset opcodes so we don't re-init the robot
    if (discardN > 0) { discardN--; continue; }
    if (b == OI_START || b == OI_SAFE || b == OI_FULL) {
      continue; // swallow
    }
    if (b == OI_BAUD) { discardN = 1; continue; }
    // Normal passthrough
    CREATE_SERIAL.write(b);
    toRobot = true;
  }
  // Robot -> Host
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
    fromRobot = true;
  }
  // No autonomous initialization; only act as a transparent proxy.
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
