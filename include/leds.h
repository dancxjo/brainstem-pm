#pragma once
#include <stdint.h>

// Patterns for the two on-board LEDs (e.g. RX/TX on ATmega32U4)
enum LedPattern {
  PATTERN_CONNECTING,
  PATTERN_WAITING,
  PATTERN_SEEKING,
  PATTERN_ADVANCING,
  PATTERN_RECOILING,
  PATTERN_TURNING_LEFT,
  PATTERN_TURNING_RIGHT,
  PATTERN_FROZEN,
  PATTERN_ALERT,
  PATTERN_IDLE,           // random asynchronous pulses
  PATTERN_SEEKING_RIGHT,   // RX slow blink, TX off
  PATTERN_BOTH_SOLID,      // both LEDs solid on
  PATTERN_GREETER_SLIDE    // bounce between RX/TX; accelerates gradually
};

void initLeds();
void setLedPattern(LedPattern p);
void updateLeds();
LedPattern getLedPattern();
void setIdleBatteryLevel(uint8_t pct);
// Directly set LED states: tx=left, rx=right
void setLeds(bool txOn, bool rxOn);
