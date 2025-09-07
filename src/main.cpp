// Brainstem boot: ensure robot is OFF, wait for USB input, buffer and reboot, then passthrough
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// AutonomyLab Create 1 expects 57600 8N1 on the robot link
static const unsigned long CREATE_BAUD = 57600;

// Hardware power control toggle: Create 1 power toggle is on robot pin 3;
// on Pro Micro this is wired to MCU pin 9.
#ifndef POWER_TOGGLE_PIN
#define POWER_TOGGLE_PIN 9
#endif

// Minimal OI opcode used for presence check
static const uint8_t OI_SENSORS = 142; // query single packet

// Host buffer for initial pre-boot commands
static uint8_t g_buf[1024];
static size_t g_len = 0;

enum Mode { WAIT_FOR_USB, PASSTHROUGH };
static Mode g_mode = WAIT_FOR_USB;

static void pulsePowerToggle(unsigned long ms) {
  digitalWrite(POWER_TOGGLE_PIN, HIGH);
  delay(ms);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
}

static bool robotLikelyOn(unsigned long waitMs) {
  // Drain any stale bytes
  while (CREATE_SERIAL.available()) (void)CREATE_SERIAL.read();
  // Try a simple sensor query; if powered and in OI, it will respond
  CREATE_SERIAL.write(OI_SENSORS);
  CREATE_SERIAL.write((uint8_t)7); // bumps/wheel-drops packet (1 byte)
  unsigned long until = millis() + waitMs;
  while (millis() < until) {
    if (CREATE_SERIAL.available() > 0) return true;
    delay(2);
  }
  return false;
}

static void ensureRobotOff() {
  if (robotLikelyOn(60)) {
    pulsePowerToggle(200); // toggle off
    delay(900);            // allow power down
  }
}

static void powerOnAndWait() {
  pulsePowerToggle(200);      // toggle on
  delay(1500);                // boot OI
}

static void flushBufferedToRobot() {
  if (g_len == 0) return;
  CREATE_SERIAL.write(g_buf, g_len);
  g_len = 0;
}

void setup() {
  Serial.begin(CREATE_BAUD);
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  pinMode(POWER_TOGGLE_PIN, OUTPUT);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
  ensureRobotOff();
}

void loop() {
  if (g_mode == WAIT_FOR_USB) {
    // Buffer all USB input until first activity, then power on and flush
    while (Serial.available() > 0) {
      int c = Serial.read();
      if (c < 0) break;
      if (g_len < sizeof(g_buf)) g_buf[g_len++] = (uint8_t)c;
    }
    if (g_len > 0) {
      powerOnAndWait();
      flushBufferedToRobot();
      g_mode = PASSTHROUGH;
    }
    // Nothing to proxy from robot while OFF; just idle briefly
    delay(2);
    return;
  }

  // PASSTHROUGH
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    CREATE_SERIAL.write((uint8_t)c);
  }
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
}
