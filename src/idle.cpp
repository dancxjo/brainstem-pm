#include "idle.h"
#include "leds.h"
#include "motion.h"
#include "sensors.h"
#include "utils.h"
#include <Arduino.h>

// Declare setIdleBatteryLevel if not already declared in included headers
void setIdleBatteryLevel(uint8_t level);

static unsigned long idleTimeoutMs = 300000; // default 5 min
static unsigned long lastUsbMs = 0;
static bool idleActive = false;
static bool sleeping = false;
static unsigned long nextFidgetMs = 0;

void initIdle(unsigned long timeoutMs) {
  idleTimeoutMs = timeoutMs;
  lastUsbMs = millis();
  idleActive = false;
  sleeping = false;
  nextFidgetMs = 0;
  setIdleBatteryLevel(100);
}

bool idleIsActive() { return idleActive; }
bool idleIsSleeping() { return sleeping; }

static void doFidget() {
  // Quiet idle: no motion or sounds. LED pattern handles blinky cue.
  // Keep a lightweight timer to avoid tight-loop churn when idle.
  unsigned long now = millis();
  if (now < nextFidgetMs) return;
  // Nudge the timer forward with a modest jitter so updateIdle() isn't hot.
  nextFidgetMs = now + 500 + (unsigned long)random(1500); // 0.5-2s
}

void updateIdle(bool usbConnected) {
  unsigned long now = millis();
  int pct = batteryPercent();
  setIdleBatteryLevel((uint8_t)pct);

  // Battery check first
  if (pct < 20) {
    if (!sleeping) {
      playLowBatteryTone();
      stopAllMotors();
      setLedPattern(PATTERN_ALERT);
      sleeping = true;
      idleActive = false;
    }
    return;
  } else if (sleeping && pct >= 20) {
    sleeping = false;
  }

  if (usbConnected) {
    lastUsbMs = now;
    if (idleActive) {
      idleActive = false;
      setLedPattern(PATTERN_BOTH_SOLID);
    }
    return;
  }

  if (!idleActive && (now - lastUsbMs >= idleTimeoutMs)) {
    idleActive = true;
    setLedPattern(PATTERN_IDLE);
    nextFidgetMs = now;
  }

  if (idleActive) {
    doFidget();
  }
}
