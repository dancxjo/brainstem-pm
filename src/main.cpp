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

// Buffer incoming host bytes; flush as a session after a long idle gap
static uint8_t g_hostBuf[512];
static size_t g_hostLen = 0;
static unsigned long g_lastHostByteMs = 0;
// A session ends after 5 minutes (300000 ms) without USB input
static const unsigned long SESSION_GAP_MS = 300000;

// Keepalive/guard: periodically re-assert FULL and send a benign stop-drive
static const unsigned long KEEPALIVE_MS = 3000;
static unsigned long g_lastKeepaliveMs = 0;

// Hardware power control toggle (connected to digital pin 9). Pulled low by default.
#ifndef POWER_TOGGLE_PIN
#define POWER_TOGGLE_PIN 9
#endif

// Pulse the power toggle line once (active-high pulse)
static void pulsePowerToggle(unsigned long highMs) {
  digitalWrite(POWER_TOGGLE_PIN, HIGH);
  delay(highMs);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
}

// Reboot (OI-level) the robot and ensure FULL control, motors stopped
static void powerRebootAndInitOi() {
  // Power-cycle via hardware toggle: off then on (two pulses)
  pulsePowerToggle(200);
  delay(500);
  pulsePowerToggle(200);
  // Give the Create time to boot its Open Interface
  delay(1000);
  // Then perform a minimal OI initialization to a known FULL, stopped state
  CREATE_SERIAL.write(OI_START);
  delay(20);
  CREATE_SERIAL.write(OI_SAFE);
  delay(20);
  CREATE_SERIAL.write(OI_DRIVE_DIRECT);
  CREATE_SERIAL.write((uint8_t)0); CREATE_SERIAL.write((uint8_t)0);
  CREATE_SERIAL.write((uint8_t)0); CREATE_SERIAL.write((uint8_t)0);
  delay(10);
  CREATE_SERIAL.write(OI_FULL);
  delay(10);
}

// Forward buffered bytes to robot with basic filtering of OI mode/baud opcodes
static void flushBufferedToRobot() {
  if (g_hostLen == 0) return;

  // Perform a power-toggle reboot before sending the session to ensure clean state
  powerRebootAndInitOi();

  // Filter: swallow OI_START/SAFE/FULL, and swallow OI_BAUD plus its 1 arg
  uint8_t discardN = 0;
  for (size_t i = 0; i < g_hostLen; ++i) {
    uint8_t b = g_hostBuf[i];
    if (discardN > 0) { discardN--; continue; }
    if (b == OI_START || b == OI_SAFE || b == OI_FULL) {
      continue; // do not forward
    }
    if (b == OI_BAUD) { discardN = 1; continue; }
    CREATE_SERIAL.write(b);
  }
  g_hostLen = 0;
}

void setup() {
  // Host-side USB CDC: baud often ignored, but set for clarity
  Serial.begin(CREATE_BAUD);
  // Robot-side UART
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  // LEDs start off
  initLeds();
  // Prepare power toggle control (default low)
  pinMode(POWER_TOGGLE_PIN, OUTPUT);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
  // Aggressively claim control so robot stays awake until midbrain boots.
  powerRebootAndInitOi();
  g_lastKeepaliveMs = millis();
}

void loop() {
  bool toRobot = false;
  bool fromRobot = false;
  // Host -> Robot: buffer until a short idle gap, then flush as a session
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (g_hostLen < sizeof(g_hostBuf)) {
      g_hostBuf[g_hostLen++] = (uint8_t)c;
      g_lastHostByteMs = millis();
      toRobot = true; // host activity seen
    } else {
      // Buffer full: flush immediately to avoid overflow
      flushBufferedToRobot();
      g_hostBuf[0] = (uint8_t)c;
      g_hostLen = 1;
      g_lastHostByteMs = millis();
      toRobot = true;
    }
  }
  // If we have buffered data and saw an idle gap, treat it as a session and flush
  if (g_hostLen > 0 && (millis() - g_lastHostByteMs) >= SESSION_GAP_MS) {
    flushBufferedToRobot();
  }
  // Robot -> Host
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
    fromRobot = true;
  }
  // Keepalive and FULL guard
  unsigned long now = millis();
  if (now - g_lastKeepaliveMs >= KEEPALIVE_MS) {
    // Re-assert FULL and send a benign stop-drive to keep OI awake
    CREATE_SERIAL.write(OI_FULL);
    CREATE_SERIAL.write(OI_DRIVE_DIRECT);
    CREATE_SERIAL.write((uint8_t)0); CREATE_SERIAL.write((uint8_t)0);
    CREATE_SERIAL.write((uint8_t)0); CREATE_SERIAL.write((uint8_t)0);
    g_lastKeepaliveMs = now;
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
