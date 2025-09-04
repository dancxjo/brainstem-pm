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
static const uint8_t OI_SONG  = 140; // Define song
static const uint8_t OI_PLAY  = 141; // Play song

// Keepalive cadence (ms). Sending a no-op drive keeps OI from idling.
static const unsigned long KEEPALIVE_INTERVAL_MS = 1000;
static unsigned long lastKeepaliveMs = 0;
// Watchdog: require periodic pet; if missed, force stop
static const unsigned long ROBOT_WATCHDOG_TIMEOUT_MS = 300; // ms
static unsigned long lastRobotWatchdogMs = 0;
static bool watchdogTripped = false;

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

  // Give the Create time to initialize its OI after boot/reset (no power control here)
  delay(1000);

  // Enter OI and take control
  CREATE_SERIAL.write(OI_START);
  delay(20);
  CREATE_SERIAL.write(OI_FULL); // Full control mode (vs SAFE)
  delay(20);

  // Send a stop drive to ensure motors are idle and start keepalive timer
  writeHighLow(OI_DRIVE, 0, 0);
  lastKeepaliveMs = millis();
  lastRobotWatchdogMs = lastKeepaliveMs;
  watchdogTripped = false;
}

void keepAliveTick() {
  unsigned long now = millis();
  if (now - lastKeepaliveMs >= KEEPALIVE_INTERVAL_MS) {
    // No-op drive (velocity 0, radius 0) acts as a benign keepalive
    writeHighLow(OI_DRIVE, 0, 0);
    lastKeepaliveMs = now;
  }
}

void feedRobotWatchdog() {
  lastRobotWatchdogMs = millis();
}

void enforceRobotWatchdog() {
  unsigned long now = millis();
  if ((now - lastRobotWatchdogMs) > ROBOT_WATCHDOG_TIMEOUT_MS) {
    if (!watchdogTripped) {
      // Only log on first trip to avoid spamming
      Serial.println("[WDOG] Motion watchdog expired; forcing STOP");
      watchdogTripped = true;
    }
    // Force a stop drive repeatedly as long as expired
    writeHighLow(OI_DRIVE, 0, 0);
  } else {
    watchdogTripped = false;
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

void playBumperSong() {
  // Define a short triad: C5, E5, G5, each 1/8 note (8 ticks of 1/64s = 125ms)
  // Middle C is 60; C5 is 72
  const uint8_t songNum = 0;
  const uint8_t notes[] = {72, 76, 79};
  const uint8_t dur = 8; // 8/64 sec
  CREATE_SERIAL.write(OI_SONG);
  CREATE_SERIAL.write(songNum);
  CREATE_SERIAL.write((uint8_t)3);
  for (uint8_t i = 0; i < 3; ++i) {
    CREATE_SERIAL.write(notes[i]);
    CREATE_SERIAL.write(dur);
  }
  // Play the song
  CREATE_SERIAL.write(OI_PLAY);
  CREATE_SERIAL.write(songNum);
}

static inline void defineAndPlay(uint8_t songNum, const uint8_t* notes, const uint8_t* durs, uint8_t count) {
  CREATE_SERIAL.write(OI_SONG);
  CREATE_SERIAL.write(songNum);
  CREATE_SERIAL.write(count);
  for (uint8_t i = 0; i < count; ++i) {
    CREATE_SERIAL.write(notes[i]);
    CREATE_SERIAL.write(durs[i]);
  }
  CREATE_SERIAL.write(OI_PLAY);
  CREATE_SERIAL.write(songNum);
}

void playStateSong(uint8_t id) {
  // Map a handful of distinct audio cues to state ids
  // Durations are in 1/64s; pitch uses MIDI note numbers
  switch (id) {
    case 0: { // CONNECTING – two rising beeps
      const uint8_t n[] = {64, 67};
      const uint8_t d[] = {8, 8};
      defineAndPlay(1, n, d, 2);
      break;
    }
    case 1: { // WAITING – short single beep
      const uint8_t n[] = {72};
      const uint8_t d[] = {6};
      defineAndPlay(2, n, d, 1);
      break;
    }
    case 2: { // SEEKING – two short alternating beeps
      const uint8_t n[] = {76, 72};
      const uint8_t d[] = {4, 4};
      defineAndPlay(3, n, d, 2);
      break;
    }
    case 3: { // ADVANCING – three ascending beeps
      const uint8_t n[] = {67, 71, 74};
      const uint8_t d[] = {4, 4, 4};
      defineAndPlay(4, n, d, 3);
      break;
    }
    case 4: { // RECOILING – descending beeps
      const uint8_t n[] = {74, 71, 67};
      const uint8_t d[] = {4, 4, 4};
      defineAndPlay(5, n, d, 3);
      break;
    }
    case 5: { // TURNING_LEFT – low double-beep
      const uint8_t n[] = {60, 60};
      const uint8_t d[] = {4, 4};
      defineAndPlay(6, n, d, 2);
      break;
    }
    case 6: { // TURNING_RIGHT – high double-beep
      const uint8_t n[] = {79, 79};
      const uint8_t d[] = {4, 4};
      defineAndPlay(7, n, d, 2);
      break;
    }
    case 7: { // FROZEN – attention pattern
      const uint8_t n[] = {84, 76, 84};
      const uint8_t d[] = {6, 6, 12};
      defineAndPlay(8, n, d, 3);
      break;
    }
    default: {
      const uint8_t n[] = {72};
      const uint8_t d[] = {4};
      defineAndPlay(0x0F, n, d, 1);
      break;
    }
  }
}

void pokeOI() {
  // Minimal, repeatable handshake to wake OI and enter FULL mode
  CREATE_SERIAL.write(OI_START);
  delay(20);
  CREATE_SERIAL.write(OI_FULL);
  delay(20);
  // benign drive to keep things alive
  writeHighLow(OI_DRIVE, 0, 0);
}

void playStartupJingle() {
  // Cheerful rising triad
  const uint8_t n[] = {72, 76, 79, 84};
  const uint8_t d[] = {6, 6, 6, 8};
  defineAndPlay(10, n, d, 4);
}

void playShutdownSigh() {
  // Descending slow tones
  const uint8_t n[] = {72, 67, 62};
  const uint8_t d[] = {12, 12, 16};
  defineAndPlay(11, n, d, 3);
}

void playForebrainTrill() {
  // Quick celebratory trill
  const uint8_t n[] = {76, 79, 83, 88};
  const uint8_t d[] = {4, 4, 4, 6};
  defineAndPlay(12, n, d, 4);
}

void playLonelyTune() {
  // Minor 3rd down-up, searching feel
  const uint8_t n[] = {69, 65, 69};
  const uint8_t d[] = {8, 8, 12};
  defineAndPlay(13, n, d, 3);
}

void playOopsChirp() {
  // Playful short chirp
  const uint8_t n[] = {84, 88};
  const uint8_t d[] = {3, 3};
  defineAndPlay(14, n, d, 2);
}

void playLowBatteryTone() {
  // Tired yawning: low down then up slightly
  const uint8_t n[] = {55, 52, 55};
  const uint8_t d[] = {10, 10, 14};
  defineAndPlay(15, n, d, 3);
}

void playCliffWhoa() {
  // Dramatic swoop down
  const uint8_t n[] = {84, 76, 67};
  const uint8_t d[] = {4, 6, 12};
  defineAndPlay(9, n, d, 3);
}

void playEstopAlarmSad() {
  // Alarm blips then a low long tone
  const uint8_t n[] = {96, 96, 48};
  const uint8_t d[] = {4, 4, 16};
  defineAndPlay(16, n, d, 3);
}

void playIdleChirp() {
  // Small bird-like chirp
  const uint8_t n[] = {88, 92};
  const uint8_t d[] = {2, 3};
  defineAndPlay(17, n, d, 2);
}

void playPurrMelody() {
  // Gentle repetitive notes
  const uint8_t n[] = {64, 66, 64, 66};
  const uint8_t d[] = {6, 6, 6, 6};
  defineAndPlay(18, n, d, 4);
}
