#pragma once
#include <stdint.h>

void initConnection();
void delayBriefly();
void randomWiggle();
void turnRandomly();
void keepAliveTick();
void playBumperSong();
// Play a short, distinct song for a given state id (0-15 supported by OI)
void playStateSong(uint8_t songId);
// Lightweight OI handshake poke: send START/FULL and benign drive
void pokeOI();
// Expressive audio cues
void playStartupJingle();
void playShutdownSigh();
void playForebrainTrill();
void playLonelyTune();
void playOopsChirp();
void playLowBatteryTone();
void playCliffWhoa();
void playEstopAlarmSad();
void playIdleChirp();
void playPurrMelody();

// Robot motion watchdog: if not fed within timeout, force a stop drive.
void feedRobotWatchdog();
void enforceRobotWatchdog();
