// Pro Micro Brainstem — HELLO/READY handshake + reboot, then passthrough
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// Serial settings
static const unsigned long HOST_BAUD = 115200;   // USB CDC (host)
static const unsigned long CREATE_BAUD = 57600;  // Create 1 default (AutonomyLab)

// Power toggle wiring (DB-25 pin 3 ↔ Pro Micro D9)
#ifndef POWER_TOGGLE_PIN
#define POWER_TOGGLE_PIN 9
#endif
// Polarity: default active-low; override with -DPOWER_TOGGLE_ACTIVE_HIGH=1 if needed
#ifndef POWER_TOGGLE_ACTIVE_HIGH
#define POWER_TOGGLE_ACTIVE_HIGH 0
#endif
static const unsigned long POWER_PULSE_HIGH_MS = 100;  // pulse width
static const unsigned long POWER_OFF_SETTLE_MS = 1200; // after OFF
static const unsigned long POWER_POST_DELAY_MS = 2000; // after ON

// Small control-line parser for HELLO
static char g_ctrlBuf[16];
static uint8_t g_ctrlLen = 0;
static bool g_hostMode = false; // becomes true after READY

// Minimal OI opcodes for benign init
static const uint8_t OI_START = 128;
static const uint8_t OI_SAFE  = 131;

static void pulsePowerToggle() {
  // Drive only during pulse; tri-state otherwise to avoid unintended toggles
  if (POWER_TOGGLE_ACTIVE_HIGH) {
    pinMode(POWER_TOGGLE_PIN, OUTPUT);
    digitalWrite(POWER_TOGGLE_PIN, LOW);  // idle
    delay(2);
    digitalWrite(POWER_TOGGLE_PIN, HIGH); // active pulse
    delay(POWER_PULSE_HIGH_MS);
    digitalWrite(POWER_TOGGLE_PIN, LOW);  // release
    pinMode(POWER_TOGGLE_PIN, INPUT);     // tri-state
  } else {
    pinMode(POWER_TOGGLE_PIN, OUTPUT);
    digitalWrite(POWER_TOGGLE_PIN, HIGH); // idle
    delay(2);
    digitalWrite(POWER_TOGGLE_PIN, LOW);  // active pulse (low)
    delay(POWER_PULSE_HIGH_MS);
    digitalWrite(POWER_TOGGLE_PIN, HIGH); // release
    pinMode(POWER_TOGGLE_PIN, INPUT);     // tri-state
  }
}

void setup() {
  Serial.begin(HOST_BAUD);
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  // Leave power toggle floating until explicitly pulsed
  pinMode(POWER_TOGGLE_PIN, INPUT);
  g_ctrlLen = 0;
  g_hostMode = false;
}

void loop() {
  // Host → control or passthrough
  while (Serial.available() > 0) {
    int ci = Serial.read();
    if (ci < 0) break;
    uint8_t b = (uint8_t)ci;
    if (!g_hostMode) {
      // Accumulate an ASCII line and look for HELLO
      if (b == '\n' || b == '\r') {
        g_ctrlBuf[g_ctrlLen < sizeof(g_ctrlBuf)-1 ? g_ctrlLen : sizeof(g_ctrlBuf)-1] = '\0';
        if (strcmp(g_ctrlBuf, "HELLO") == 0) {
          Serial.println("BUSY");
          // Deterministic OFF → ON power cycle
          pulsePowerToggle();                // OFF
          delay(POWER_OFF_SETTLE_MS);
          pulsePowerToggle();                // ON
          delay(POWER_POST_DELAY_MS);
          // Minimal OI init to a benign state
          CREATE_SERIAL.write(OI_START);
          CREATE_SERIAL.write(OI_SAFE);
          // Hand over to host
          Serial.println("READY");
          g_hostMode = true;
        }
        g_ctrlLen = 0;
      } else if (g_ctrlLen + 1 < sizeof(g_ctrlBuf)) {
        g_ctrlBuf[g_ctrlLen++] = (char)b;
      } else {
        // overflow; reset buffer
        g_ctrlLen = 0;
      }
      // Do not forward bytes before READY
      continue;
    }
    // Passthrough mode
    CREATE_SERIAL.write(b);
  }

  // Robot → Host (only when in passthrough)
  if (g_hostMode) {
    while (CREATE_SERIAL.available() > 0) {
      int c = CREATE_SERIAL.read();
      if (c < 0) break;
      Serial.write((uint8_t)c);
    }
  }
}
