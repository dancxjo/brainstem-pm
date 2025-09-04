#include "behavior.h"
#include "motion.h"
#include "sensors.h"
#include "utils.h"
#include "leds.h"
#include <Arduino.h>

enum State {
  CONNECTING,
  WAITING,
  WALL_FOLLOWING,
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
static unsigned long bumperFlashUntil = 0; // LED alert window
// Wall-follow settings
static bool followRight = true; // default side
// Reconnect backoff state
static unsigned long nextConnectAttemptMs = 0;
static unsigned connectRetry = 0; // exponential backoff counter
static const unsigned long CONNECT_BASE_MS = 500;   // initial interval
static const unsigned long CONNECT_MAX_MS  = 8000;  // cap interval

static inline void enterState(State s) {
  State prev = currentState;
  auto name = [](State st) -> const char* {
    switch (st) {
      case CONNECTING: return "CONNECTING";
      case WAITING: return "WAITING";
      case SEEKING: return "SEEKING";
      case ADVANCING: return "ADVANCING";
      case RECOILING: return "RECOILING";
      case TURNING_LEFT: return "TURNING_LEFT";
      case TURNING_RIGHT: return "TURNING_RIGHT";
      case FROZEN: return "FROZEN";
    }
    return "?";
  };
  Serial.print("[FSM] ");
  Serial.print(name(currentState));
  Serial.print(" -> ");
  Serial.println(name(s));
  Serial.print("  bias="); Serial.print((int)turnBias);
  Serial.print(" runTarget="); Serial.print(runTicksTarget);
  Serial.print(" bumps="); Serial.println(bumpsRecently);
  // Play a distinct audio cue only when state actually changes
  if (s != currentState) {
    playStateSong((uint8_t)s);
  }
  currentState = s;
  stateEnterMs = millis();
  // Event-based expressions
  if (prev == RECOILING && s == SEEKING) {
    // After escaping a bump, playful chirp
    playOopsChirp();
  }
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

void setWallFollowSide(bool right) { followRight = right; }
void toggleWallFollowSide() { followRight = !followRight; }

void updateBehavior() {
  if (millis() - lastTick < tickInterval) return;
  lastTick = millis();
  // Feed motion watchdog at the cadence of control ticks
  feedRobotWatchdog();
  // Keep the Create's OI alive only when connected or not in CONNECTING
  if (!(currentState == CONNECTING && !oiConnected())) {
    keepAliveTick();
  }
  // Sensor stream is polled in main; cached values are current

  // Handle asynchronous bumper interrupt: play song, flash LEDs, and recoil
  if (bumperEventTriggeredAndClear()) {
    playBumperSong();
    bumperFlashUntil = millis() + 600; // flash for 0.6s
    enterState(RECOILING);
  }

  // (Optional pet-me mode could be added here by counting rapid ISR taps.)

  // Safety preemption: if a hazard is present, force immediate transition
  if (cliffDetected()) {
    enterState(FROZEN);
  } else if (bumperTriggered()) {
    enterState(RECOILING);
  }

  // Reflect current state on LEDs each tick, with alert override window
  switch (currentState) {
    case CONNECTING:     setLedPattern(PATTERN_CONNECTING); break;
    case WAITING:        setLedPattern(PATTERN_WAITING); break;
    case WALL_FOLLOWING: setLedPattern(PATTERN_ADVANCING); break;
    case SEEKING:        setLedPattern(PATTERN_SEEKING); break;
    case ADVANCING:      setLedPattern(PATTERN_ADVANCING); break;
    case RECOILING:      setLedPattern(PATTERN_RECOILING); break;
    case TURNING_LEFT:   setLedPattern(PATTERN_TURNING_LEFT); break;
    case TURNING_RIGHT:  setLedPattern(PATTERN_TURNING_RIGHT); break;
    case FROZEN:         setLedPattern(PATTERN_FROZEN); break;
  }

  if (bumperFlashUntil && millis() < bumperFlashUntil) {
    setLedPattern(PATTERN_ALERT);
  }

  switch (currentState) {
    case CONNECTING:
      // Ensure motors are idle while attempting connection
      stopAllMotors();
      // If we are already seeing the stream, proceed and reset backoff.
      if (oiConnected()) {
        connectRetry = 0;
        nextConnectAttemptMs = 0;
        enterState(WAITING);
        break;
      }
      // Periodically try to wake/configure the OI with exponential backoff
      if (millis() >= nextConnectAttemptMs) {
        unsigned long now = millis();
        // Compute interval = min(MAX, BASE << retry)
        unsigned long interval = CONNECT_BASE_MS;
        if (connectRetry < 14) { // guard shifts
          interval <<= connectRetry;
        }
        if (interval > CONNECT_MAX_MS) interval = CONNECT_MAX_MS;
        // Add +/-20% jitter to avoid phase-locking
        long jitter = (long)(interval / 5);
        long delta = (long)random((long)(2 * jitter + 1)) - jitter;
        nextConnectAttemptMs = now + interval + (unsigned long)((delta < 0) ? 0 - delta : delta);
        Serial.print("[FSM] CONNECTING attempt "); Serial.print(connectRetry);
        Serial.print(" interval="); Serial.print(interval);
        Serial.print(" next in ~"); Serial.println((long)(interval + delta));
        pokeOI();
        beginSensorStream();
        if (connectRetry < 20) connectRetry++;
      }
      // stay in CONNECTING until stream becomes active
      break;

    case WAITING:
      // Idle: keep motors stopped during brief wait
      stopAllMotors();
      if (!oiConnected()) { enterState(CONNECTING); break; }
      delayBriefly();
      // enterState(WALL_FOLLOWING); // remain in WAITING
      break;

    case WALL_FOLLOWING: {
      if (!oiConnected()) { enterState(CONNECTING); break; }
      // Simple heuristic wall follow using OI 'wall' boolean
      if (cliffDetected()) { enterState(FROZEN); break; }
      if (bumperTriggered()) { enterState(RECOILING); break; }
      if (wallDetected()) {
        // When on a wall, bias toward it slightly and move forward
        if (followRight) veerRightOneTick(); else veerLeftOneTick();
        forwardOneTick();
      } else {
        // Search for wall: rotate toward the side we follow
        if (followRight) turnRightOneTick(); else turnLeftOneTick();
      }
      enterState(WALL_FOLLOWING);
      break;
    }

    case SEEKING: {
      if (!oiConnected()) { enterState(CONNECTING); break; }
      int stimulus = scanEnvironment();
      if (stimulus == 1) {
        // Forward attractant: begin a run with current bias
        // Longer runs if we haven't bumped recently
        unsigned long sinceBump = millis() - lastBumpMs;
        runTicksTarget = (sinceBump > 5000) ? 10 : 6;
        Serial.print("[FSM] SEEKING stimulus forward, runTicksTarget=");
        Serial.println(runTicksTarget);
        enterState(ADVANCING);
      } else if (stimulus == -1) {
        Serial.println("[FSM] SEEKING stimulus left");
        enterState(TURNING_LEFT);
      } else if (stimulus == 2) {
        Serial.println("[FSM] SEEKING stimulus right");
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
        Serial.print("[FSM] SEEKING cast phase="); Serial.println(castingPhase);
        // Periodically attempt a short forward tick to probe ahead
        if ((castingPhase % 5) == 0) {
          Serial.println("[FSM] SEEKING forward probe");
          forwardOneTick();
        }
        // Remain in SEEKING
        enterState(SEEKING);
      }
      break;
    }

    case ADVANCING:
      if (!oiConnected()) { enterState(CONNECTING); break; }
      // Check sensors first to avoid driving further into obstacles
      if (bumperTriggered()) {
        // Habituation: record bump timing and increase turn magnitude
        lastBumpMs = millis();
        if (bumpsRecently < 10) bumpsRecently++;
        Serial.print("[FSM] ADV bump! bumpsRecently="); Serial.println(bumpsRecently);
        enterState(RECOILING);
      } else if (cliffDetected()) {
        Serial.println("[FSM] ADV cliff!");
        enterState(FROZEN);
      } else {
        // Gentle veer in the direction of bias to create a run
        if (turnBias > 0) veerRightOneTick(); else veerLeftOneTick();
        runTicksSoFar++;
        Serial.print("[FSM] ADV tick run="); Serial.print(runTicksSoFar);
        Serial.print("/"); Serial.println(runTicksTarget);
        if (runTicksSoFar < runTicksTarget) {
          // continue the run
          enterState(ADVANCING);
        } else {
          // finished a run; brief seek to reassess
          enterState(SEEKING);
        }
      }
      break;

    case RECOILING:
      if (!oiConnected()) { enterState(CONNECTING); break; }
      // Back up longer if we've bumped repeatedly (simple habituation)
      backwardOneTick();
      if (bumpsRecently >= 3) {
        backwardOneTick();
      }
      // Turn away using bias; if we've been bumping a lot, flip the bias to escape
      if (bumpsRecently >= 5) {
        turnBias = -turnBias; // escape trap
        bumpsRecently = 0;    // reset after decisive change
        Serial.println("[FSM] RECOIL flipping bias to escape");
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
      enterState(WALL_FOLLOWING);
      break;

    case TURNING_LEFT:
      if (!oiConnected()) { enterState(CONNECTING); break; }
      turnLeftOneTick();
      turnBias = -1;            // reinforce left bias after explicit turn
      runTicksTarget = 6;       // set a medium run to capitalize on turn
      enterState(ADVANCING);
      break;

    case TURNING_RIGHT:
      if (!oiConnected()) { enterState(CONNECTING); break; }
      turnRightOneTick();
      turnBias = 1;             // reinforce right bias after explicit turn
      runTicksTarget = 6;
      enterState(ADVANCING);
      break;

    case FROZEN:
      if (!oiConnected()) { enterState(CONNECTING); break; }
      stopAllMotors();
      alertFreeze();
      break;
  }
}
