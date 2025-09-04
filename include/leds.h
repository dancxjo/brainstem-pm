#pragma once

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
  PATTERN_SEEKING_RIGHT,   // RX slow blink, TX off
  PATTERN_BOTH_SOLID       // both LEDs solid on
};

void initLeds();
void setLedPattern(LedPattern p);
void updateLeds();
