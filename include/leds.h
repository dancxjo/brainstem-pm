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
  PATTERN_FROZEN
};

void initLeds();
void setLedPattern(LedPattern p);
void updateLeds();

