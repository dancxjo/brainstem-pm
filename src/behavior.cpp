#include "behavior.h"
#include "motion.h"
#include "sensors.h"
#include "utils.h"
#include "leds.h"
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

// Micro-behavior internal state inspired by simple animal foraging:
// - run-and-cast: forward "runs" with gentle bias, punctuated by casting (small arcs)
// - habituation: repeated bumps increase turn magnitude and adjust bias
// - persistence: maintain heading bias to avoid pure random walk
static unsigned long stateEnterMs = 0;
static int8_t turnBias = 1;           // +1 = favor right, -1 = favor left
static unsigned runTicksTarget = 5;   // desired forward ticks in a run
static unsigned runTicksSoFar = 0;    // progress through the current run
static unsigned castingPhase = 0;     // toggles small left/right arcs while seeking
static unsigned bumpsRecently = 0;    // habituation counter
static unsigned long lastBumpMs = 0;  // timestamp of last bumper event

static inline void enterState(State s) {
  currentState = s;
  stateEnterMs = millis();
  // reset per-state counters where appropriate
  if (s == ADVANCING) {
    runTicksSoFar = 0;
  } else if (s == SEEKING) {
    castingPhase = 0;
  }
}

void initializeBehavior() {
  initMotors();
  initSensors();
  currentState = CONNECTING;
  lastTick = millis();
  stateEnterMs = lastTick;
  // seed a turning bias to avoid symmetric dithering
  turnBias = (random(2) == 0) ? -1 : 1;
}

void updateBehavior() {
  if (millis() - lastTick < tickInterval) return;
  lastTick = millis();
  // Keep the Create's OI alive with periodic no-op commands
  keepAliveTick();

  // Reflect current state on LEDs each tick
  switch (currentState) {
    case CONNECTING:     setLedPattern(PATTERN_CONNECTING); break;
    case WAITING:        setLedPattern(PATTERN_WAITING); break;
    case SEEKING:        setLedPattern(PATTERN_SEEKING); break;
    case ADVANCING:      setLedPattern(PATTERN_ADVANCING); break;
    case RECOILING:      setLedPattern(PATTERN_RECOILING); break;
    case TURNING_LEFT:   setLedPattern(PATTERN_TURNING_LEFT); break;
    case TURNING_RIGHT:  setLedPattern(PATTERN_TURNING_RIGHT); break;
    case FROZEN:         setLedPattern(PATTERN_FROZEN); break;
  }

  switch (currentState) {
    case CONNECTING:
      initConnection();
      enterState(WAITING);
      break;

    case WAITING:
      delayBriefly();
      enterState(SEEKING);
      break;

    case SEEKING: {
      int stimulus = scanEnvironment();
      if (stimulus == 1) {
        // Forward attractant: begin a run with current bias
        // Longer runs if we haven't bumped recently
        unsigned long sinceBump = millis() - lastBumpMs;
        runTicksTarget = (sinceBump > 5000) ? 10 : 6;
        enterState(ADVANCING);
      } else if (stimulus == -1) {
        enterState(TURNING_LEFT);
      } else if (stimulus == 2) {
        enterState(TURNING_RIGHT);
      } else {
        // No stimulus: casting — alternating gentle arcs, slightly biased
        // toward turnBias to create persistence without random walk
        bool favorRight = (turnBias > 0);
        // 0-2: favored direction, 3: opposite direction
        if ((castingPhase % 4) < 3) {
          if (favorRight) veerRightOneTick(); else veerLeftOneTick();
        } else {
          if (favorRight) veerLeftOneTick(); else veerRightOneTick();
        }
        castingPhase++;
        // Periodically attempt a short forward tick to probe ahead
        if ((castingPhase % 5) == 0) {
          forwardOneTick();
        }
        // Remain in SEEKING
        enterState(SEEKING);
      }
      break;
    }

    case ADVANCING:
      // Gentle veer in the direction of bias to create a run
      if (turnBias > 0) veerRightOneTick(); else veerLeftOneTick();
      runTicksSoFar++;

      if (bumperTriggered()) {
        // Habituation: record bump timing and increase turn magnitude
        lastBumpMs = millis();
        if (bumpsRecently < 10) bumpsRecently++;
        enterState(RECOILING);
      } else if (cliffDetected()) {
        enterState(FROZEN);
      } else if (runTicksSoFar < runTicksTarget) {
        // continue the run
        enterState(ADVANCING);
      } else {
        // finished a run; brief seek to reassess
        enterState(SEEKING);
      }
      break;

    case RECOILING:
      // Back up longer if we've bumped repeatedly (simple habituation)
      backwardOneTick();
      if (bumpsRecently >= 3) {
        backwardOneTick();
      }
      // Turn away using bias; if we've been bumping a lot, flip the bias to escape
      if (bumpsRecently >= 5) {
        turnBias = -turnBias; // escape trap
        bumpsRecently = 0;    // reset after decisive change
      }
      if (turnBias > 0) {
        turnRightOneTick();
        if (bumpsRecently >= 2) turnRightOneTick();
      } else {
        turnLeftOneTick();
        if (bumpsRecently >= 2) turnLeftOneTick();
      }
      // Start a shorter forward run after recoil to test the new heading
      runTicksTarget = 4;
      enterState(SEEKING);
      break;

    case TURNING_LEFT:
      turnLeftOneTick();
      turnBias = -1;            // reinforce left bias after explicit turn
      runTicksTarget = 6;       // set a medium run to capitalize on turn
      enterState(ADVANCING);
      break;

    case TURNING_RIGHT:
      turnRightOneTick();
      turnBias = 1;             // reinforce right bias after explicit turn
      runTicksTarget = 6;
      enterState(ADVANCING);
      break;

    case FROZEN:
      stopAllMotors();
      alertFreeze();
      break;
  }
}
