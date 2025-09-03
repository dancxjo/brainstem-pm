#include "utils.h"
#include "motion.h"
#include <Arduino.h>

// Select the hardware serial used to talk to the Create.
// On ATmega32U4 boards (e.g. Pro Micro), Serial is USB-CDC and Serial1 is the UART pins.
#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// iRobot Create/Open Interface (OI) opcodes we'll use
static const uint8_t OI_START = 128; // Enter passive
static const uint8_t OI_SAFE  = 131; // Safe mode
static const uint8_t OI_FULL  = 132; // Full mode
static const uint8_t OI_DRIVE = 137; // Drive command

// Keepalive cadence (ms). Sending a no-op drive keeps OI from idling.
static const unsigned long KEEPALIVE_INTERVAL_MS = 1000;
static unsigned long lastKeepaliveMs = 0;

static inline void writeHighLow(uint8_t opcode, int16_t v1, int16_t v2) {
  CREATE_SERIAL.write(opcode);
  CREATE_SERIAL.write((uint8_t)((v1 >> 8) & 0xFF));
  CREATE_SERIAL.write((uint8_t)(v1 & 0xFF));
  CREATE_SERIAL.write((uint8_t)((v2 >> 8) & 0xFF));
  CREATE_SERIAL.write((uint8_t)(v2 & 0xFF));
}

void initConnection() {
  // Initialize UART to the Create. Default OI baud is typically 57600.
  CREATE_SERIAL.begin(57600);

  // Give the Create time to power its OI after boot/reset
  delay(1000);

  // Enter OI and take control
  CREATE_SERIAL.write(OI_START);
  delay(20);
  CREATE_SERIAL.write(OI_FULL); // Full control mode (vs SAFE)
  delay(20);

  // Send a stop drive to ensure motors are idle and start keepalive timer
  writeHighLow(OI_DRIVE, 0, 0);
  lastKeepaliveMs = millis();
}

void keepAliveTick() {
  unsigned long now = millis();
  if (now - lastKeepaliveMs >= KEEPALIVE_INTERVAL_MS) {
    // No-op drive (velocity 0, radius 0) acts as a benign keepalive
    writeHighLow(OI_DRIVE, 0, 0);
    lastKeepaliveMs = now;
  }
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
