// Read Create 1 sensors via the Open Interface (OI)
// Reference: OI opcode 142 (Sensors), with packet IDs such as 7 (Bumps/Wheel Drops)
// and 9-12 (Cliff Left, Front Left, Front Right, Right). Each of these packets
// returns one byte where non-zero means the event is active.

#include "sensors.h"
#include "utils.h"
#include <Arduino.h>

// Select the hardware serial used to talk to the Create OI.
#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

static const uint8_t OI_SENSORS = 142; // Query single sensor packet
static const uint8_t OI_STREAM  = 148; // Start data stream
static const uint8_t OI_PAUSE   = 150; // Pause/Resume data stream

// Optional external bumper interrupt (active-low microswitch on a GPIO)
#ifndef BUMPER_PIN
#define BUMPER_PIN 3
#endif
#ifndef BUMPER_ACTIVE_LOW
#define BUMPER_ACTIVE_LOW 1
#endif
static volatile bool bumperEventFlag = false;
static void bumperIsr() {
#if BUMPER_ACTIVE_LOW
  if (digitalRead(BUMPER_PIN) == LOW)
    bumperEventFlag = true;
#else
  if (digitalRead(BUMPER_PIN) == HIGH)
    bumperEventFlag = true;
#endif
}

// Cached sensor values from OI stream
static bool cachedBumpLeft = false;
static bool cachedBumpRight = false;
static bool cachedCliffL = false, cachedCliffFL = false, cachedCliffFR = false, cachedCliffR = false;

// Stream single-byte packets only to keep parsing simple:
//  - 7  = Bumps/Wheel Drops (1 byte)
//  - 9  = Cliff Left (1 byte)
//  - 10 = Cliff Front Left (1 byte)
//  - 11 = Cliff Front Right (1 byte)
//  - 12 = Cliff Right (1 byte)
//  - 18 = Buttons (1 byte)
//  - 8  = Wall (boolean, 1 byte)
static const uint8_t requestedPackets[] = { 7, 9, 10, 11, 12, 18, 8 };
static unsigned long lastStreamMs = 0;
// Pair parser state: expect value after seeing an ID
static bool expectValue = false;
static uint8_t currentId = 0;
// Cached wall and button edges
static bool cachedWall = false;
static uint8_t lastButtons = 0;
static volatile bool btnPlayEdge = false;
static volatile bool btnAdvEdge = false;
static uint16_t badChecksumCount = 0;
static unsigned long lastRecoverMs = 0;

static void recoverStreamIfNeeded() {
  unsigned long now = millis();
  if (badChecksumCount >= 8 && (now - lastStreamMs) > 250) {
    // Too many errors with no valid frame recently: reconfigure stream and poke OI
    Serial.println("[SENS] stream auto-recover: reconfig + pokeOI");
    pokeOI();
    // Reconfigure stream list and resume
    CREATE_SERIAL.write(OI_PAUSE); CREATE_SERIAL.write((uint8_t)0);
    CREATE_SERIAL.write(OI_STREAM);
    CREATE_SERIAL.write((uint8_t)(sizeof(requestedPackets)));
    for (uint8_t i = 0; i < sizeof(requestedPackets); ++i) CREATE_SERIAL.write(requestedPackets[i]);
    CREATE_SERIAL.write(OI_PAUSE); CREATE_SERIAL.write((uint8_t)1);
    // Reset parser state and counters
    spState = WAIT_HEADER; spLen = 0; spRead = 0; badChecksumCount = 0; lastRecoverMs = now;
    // Drain any residual bytes quickly
    while (CREATE_SERIAL.available()) { (void)CREATE_SERIAL.read(); }
  }
}
// Previous snapshot for change-detect logging
static bool prevBumpLeft = false, prevBumpRight = false;
static bool prevCliffL = false, prevCliffFL = false, prevCliffFR = false, prevCliffR = false;
static bool prevWall = false;
static uint8_t prevButtons = 0;

// Polling helpers removed in minimal stream parser build

static bool streamPaused = false;

void beginSensorStream() {
  // Pause any existing stream, configure, then resume
  CREATE_SERIAL.write(OI_PAUSE);
  CREATE_SERIAL.write((uint8_t)0); // pause
  CREATE_SERIAL.write(OI_STREAM);
  CREATE_SERIAL.write((uint8_t)(sizeof(requestedPackets)));
  for (uint8_t i = 0; i < sizeof(requestedPackets); ++i) {
    CREATE_SERIAL.write(requestedPackets[i]);
  }
  CREATE_SERIAL.write(OI_PAUSE);
  CREATE_SERIAL.write((uint8_t)1); // resume
  // Reset parser state and drain any stale bytes
  expectValue = false;
  currentId = 0;
  streamPaused = false;
  while (CREATE_SERIAL.available()) { (void)CREATE_SERIAL.read(); }
  // stream started
}

void pauseSensorStream() {
  if (!streamPaused) {
    // Pause any ongoing OI stream
    CREATE_SERIAL.write(OI_PAUSE);
    CREATE_SERIAL.write((uint8_t)0);
    streamPaused = true;
  }
}

void resumeSensorStream() {
  if (streamPaused) {
    // Resume an already configured OI stream
    CREATE_SERIAL.write(OI_PAUSE);
    CREATE_SERIAL.write((uint8_t)1);
    streamPaused = false;
  }
}

void updateSensorStream() {
  while (CREATE_SERIAL.available()) {
    int bi = CREATE_SERIAL.read();
    if (bi < 0) break;
    uint8_t b = (uint8_t)bi;
    // Skip frame header and immediate len if present
    if (!expectValue && b == 19) {
      // consume len if present; if not yet available, try next loop
      if (CREATE_SERIAL.available()) (void)CREATE_SERIAL.read();
      continue;
    }
    // Ignore lone len marker occasionally observed
    if (!expectValue && b == 14) {
      continue;
    }
    if (!expectValue) {
      switch (b) {
        case 7: case 8: case 9: case 10: case 11: case 12: case 18:
          currentId = b; expectValue = true; break;
        default: break; // ignore noise/opcodes
      }
    } else {
      uint8_t val = b;
      switch (currentId) {
        case 7:  cachedBumpRight = (val & 0x01) != 0; cachedBumpLeft = (val & 0x02) != 0; break;
        case 8:  cachedWall = (val != 0); break;
        case 9:  cachedCliffL  = (val != 0); break;
        case 10: cachedCliffFL = (val != 0); break;
        case 11: cachedCliffFR = (val != 0); break;
        case 12: cachedCliffR  = (val != 0); break;
        case 18: { uint8_t prev = lastButtons; lastButtons = val; if ((!(prev & 0x01)) && (val & 0x01)) btnPlayEdge = true; if ((!(prev & 0x04)) && (val & 0x04)) btnAdvEdge = true; break; }
        default: break;
      }
      lastStreamMs = millis();
      bool changed = (cachedBumpLeft != prevBumpLeft) || (cachedBumpRight != prevBumpRight) ||
                     (cachedCliffL != prevCliffL) || (cachedCliffFL != prevCliffFL) ||
                     (cachedCliffFR != prevCliffFR) || (cachedCliffR != prevCliffR) ||
                     (cachedWall != prevWall) || (lastButtons != prevButtons);
      if (changed) {
        // log suppressed to save flash
      }
      prevBumpLeft = cachedBumpLeft; prevBumpRight = cachedBumpRight;
      prevCliffL = cachedCliffL; prevCliffFL = cachedCliffFL; prevCliffFR = cachedCliffFR; prevCliffR = cachedCliffR;
      prevWall = cachedWall; prevButtons = lastButtons;
      expectValue = false; currentId = 0;
    }
  }
}

bool oiConnected() {
  // Consider connected if we saw a valid stream frame recently
  unsigned long now = millis();
  return (lastStreamMs != 0) && (now - lastStreamMs < 2000);
}

void initSensors() {
  // Nothing to initialize beyond the OI connection handled elsewhere.
  // Try to attach an external interrupt for a GPIO-wired bumper.
#ifdef ARDUINO
  pinMode(BUMPER_PIN, BUMPER_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
#  ifdef digitalPinToInterrupt
  int irq = digitalPinToInterrupt(BUMPER_PIN);
  if (irq != NOT_AN_INTERRUPT) {
    attachInterrupt(irq, bumperIsr, CHANGE);
  } else {
    // no interrupt available
  }
#  endif
#endif
}

int scanEnvironment() {
  // Placeholder for directional stimulus sensing (e.g., IR beacons).
  // For now, we return neutral (0). Extend by querying IR/opcode as needed.
  return 0;
}

bool bumperTriggered() {
  bool any = (cachedBumpLeft || cachedBumpRight);
  if (any) {
    // log suppressed
  }
  return any;
}

bool cliffDetected() {
  bool any = (cachedCliffL || cachedCliffFL || cachedCliffFR || cachedCliffR);
  if (any) { /* log suppressed */ }
  return any;
}

bool bumperEventTriggeredAndClear() {
  bool was = bumperEventFlag;
  bumperEventFlag = false;
  if (was) { /* log suppressed */ }
  return was;
}

bool wallDetected() { return cachedWall; }

bool playButtonPressedAndClear() {
  bool was = btnPlayEdge;
  btnPlayEdge = false;
  return was;
}

bool advanceButtonPressedAndClear() {
  bool was = btnAdvEdge;
  btnAdvEdge = false;
  return was;
}
