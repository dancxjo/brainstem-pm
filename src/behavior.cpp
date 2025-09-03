#include "behavior.h"
#include "motion.h"
#include "sensors.h"
#include "utils.h"
#include <Arduino.h>

enum State {
  CONNECTING,
  WAITING,
  SEEKING,
  ADVANCING,
  RECOILING,
  TURNING_LEFT,
  TURNING_RIGHT,
  FROZEN
};

static State currentState = CONNECTING;
static unsigned long lastTick = 0;
const unsigned long tickInterval = 100; // ms

void initializeBehavior() {
  initMotors();
  initSensors();
  currentState = CONNECTING;
  lastTick = millis();
}

void updateBehavior() {
  if (millis() - lastTick < tickInterval) return;
  lastTick = millis();

  switch (currentState) {
    case CONNECTING:
      initConnection();
      currentState = WAITING;
      break;

    case WAITING:
      delayBriefly();
      currentState = SEEKING;
      break;

    case SEEKING: {
      int stimulus = scanEnvironment();
      if (stimulus == 1) currentState = ADVANCING;
      else if (stimulus == -1) currentState = TURNING_LEFT;
      else if (stimulus == 2) currentState = TURNING_RIGHT;
      else {
        randomWiggle();
        currentState = SEEKING;
      }
      break;
    }

    case ADVANCING:
      forwardOneTick();
      if (bumperTriggered()) currentState = RECOILING;
      else if (cliffDetected()) currentState = FROZEN;
      else currentState = SEEKING;
      break;

    case RECOILING:
      backwardOneTick();
      turnRandomly();
      currentState = SEEKING;
      break;

    case TURNING_LEFT:
      turnLeftOneTick();
      currentState = ADVANCING;
      break;

    case TURNING_RIGHT:
      turnRightOneTick();
      currentState = ADVANCING;
      break;

    case FROZEN:
      stopAllMotors();
      alertFreeze();
      break;
  }
}
