#include "leds.h"
#include <Arduino.h>

// Helpers to control TX/RX LEDs across cores.
// On 32U4 boards like Pro Micro, TX/RX LEDs are active-low and cores often
// provide TXLED0/TXLED1 and RXLED0/RXLED1. Fall back to LED_BUILTIN_* if present.

static inline void setTx(bool on) {
#if defined(TXLED0) && defined(TXLED1)
  if (on) { TXLED0; } else { TXLED1; }
#elif defined(LED_BUILTIN_TX)
  pinMode(LED_BUILTIN_TX, OUTPUT);
  digitalWrite(LED_BUILTIN_TX, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

static inline void setRx(bool on) {
#if defined(RXLED0) && defined(RXLED1)
  if (on) { RXLED0; } else { RXLED1; }
#elif defined(LED_BUILTIN_RX)
  pinMode(LED_BUILTIN_RX, OUTPUT);
  digitalWrite(LED_BUILTIN_RX, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

static LedPattern current = PATTERN_CONNECTING;
static unsigned long patternStartMs = 0;
static uint8_t idleBatteryLevel = 100;

void initLeds() {
  // Ensure LEDs start known-off
  setTx(false);
  setRx(false);
  patternStartMs = millis();
}

void setLedPattern(LedPattern p) {
  if (p != current) {
    current = p;
    patternStartMs = millis();
    // Log LED pattern changes to verify they mirror state
    auto name = [](LedPattern lp) -> const char* {
      switch (lp) {
        case PATTERN_CONNECTING: return "CONNECTING";
        case PATTERN_WAITING: return "WAITING";
        case PATTERN_SEEKING: return "SEEKING";
        case PATTERN_ADVANCING: return "ADVANCING";
        case PATTERN_RECOILING: return "RECOILING";
        case PATTERN_TURNING_LEFT: return "TURNING_LEFT";
        case PATTERN_TURNING_RIGHT: return "TURNING_RIGHT";
        case PATTERN_FROZEN: return "FROZEN";
        case PATTERN_ALERT: return "ALERT";
        case PATTERN_IDLE: return "IDLE";
        case PATTERN_SEEKING_RIGHT: return "SEEKING_RIGHT";
        case PATTERN_BOTH_SOLID: return "BOTH_SOLID";
      }
      return "?";
    };
    Serial.print("[LED] pattern=");
    Serial.println(name(current));
  }
}

// Compute LED on/off for the given time offset
static void patternAt(unsigned long tMs, bool &txOn, bool &rxOn) {
  switch (current) {
    case PATTERN_CONNECTING:
      // Alternate TX/RX at 4 Hz (250 ms each)
      txOn = ((tMs / 250) % 2) == 0;
      rxOn = !txOn;
      break;
    case PATTERN_WAITING:
      // Heartbeat: both on for 100 ms every 1 s
      txOn = (tMs % 1000) < 100;
      rxOn = txOn;
      break;
    case PATTERN_SEEKING:
      // TX slow blink 1 Hz, RX off
      txOn = (tMs % 1000) < 500;
      rxOn = false;
      break;
    case PATTERN_ADVANCING:
      // TX solid on, RX off
      txOn = true;
      rxOn = false;
      break;
    case PATTERN_RECOILING:
      // RX solid on, TX off
      txOn = false;
      rxOn = true;
      break;
    case PATTERN_TURNING_LEFT:
      // TX fast blink (5 Hz), RX off
      txOn = (tMs % 200) < 100;
      rxOn = false;
      break;
    case PATTERN_TURNING_RIGHT:
      // RX fast blink (5 Hz), TX off
      txOn = false;
      rxOn = (tMs % 200) < 100;
      break;
    case PATTERN_FROZEN:
      // Both strobe (5 Hz)
      txOn = (tMs % 200) < 100;
      rxOn = txOn;
      break;
    case PATTERN_ALERT:
      // Rapid alternating flash (10 Hz), eye-catching
      txOn = (tMs % 100) < 50;
      rxOn = !txOn;
      break;
    case PATTERN_IDLE:
      // Random asynchronous blips scaled by battery level
      txOn = random(100) < idleBatteryLevel;
      rxOn = random(100) < idleBatteryLevel;
      break;
    case PATTERN_SEEKING_RIGHT:
      // RX slow blink 1 Hz, TX off
      txOn = false;
      rxOn = (tMs % 1000) < 500;
      break;
    case PATTERN_BOTH_SOLID:
      txOn = true; rxOn = true;
      break;
  }
}

void updateLeds() {
  // In native tests, Arduino timing may not exist; guard with ARDUINO macro.
#ifdef ARDUINO
  unsigned long now = millis();
  unsigned long t = now - patternStartMs;
  bool txOn = false, rxOn = false;
  patternAt(t, txOn, rxOn);
  setTx(txOn);
  setRx(rxOn);
#endif
}

LedPattern getLedPattern() { return current; }
void setIdleBatteryLevel(uint8_t pct) { idleBatteryLevel = pct; }
