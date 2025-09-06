#include "motion.h"
#include <Arduino.h>
#include "sensors.h"  // for pause/resume of OI sensor stream during blocking motions

// iRobot Create 1 Open Interface opcodes
static constexpr uint8_t OI_START = 128;
static constexpr uint8_t OI_SAFE = 131;
static constexpr uint8_t OI_FULL = 132;
static constexpr uint8_t OI_DRIVE_DIRECT = 145;
static constexpr uint16_t VELOCITY = 200; // mm/s base before scaling
static constexpr unsigned long TICK_MS = 100; // duration of one tick
static float SPEED_SCALE = 0.25f; // 25% speed for gentle autonomous/presence

/**
 * Helper to send direct wheel speeds to the Create.
 * @param right Right wheel velocity in mm/s
 * @param left  Left wheel velocity in mm/s
 */
static void driveWheels(int16_t right, int16_t left) {
  // Apply global scale to behavior/presence motions only
  right = (int16_t)(right * SPEED_SCALE);
  left  = (int16_t)(left * SPEED_SCALE);
  uint8_t cmd[] = {
      OI_DRIVE_DIRECT,
      static_cast<uint8_t>((right >> 8) & 0xFF),
      static_cast<uint8_t>(right & 0xFF),
      static_cast<uint8_t>((left >> 8) & 0xFF),
      static_cast<uint8_t>(left & 0xFF)};
  Serial1.write(cmd, sizeof(cmd));
}

/**
 * Initialize the Create's drive system and enter SAFE mode.
 *
 * Example:
 * ```
 * initMotors();
 * ```
 */
void initMotors() {
  Serial1.begin(57600);
  delay(100);
  Serial1.write(OI_START);
  // Create 1: prefer FULL mode to avoid unexpected passive/safe drops during autonomous ticks
  Serial1.write(OI_FULL);
}

/**
 * Advance the robot forward for one control tick.
 */
void forwardOneTick() {
  pauseSensorStream();
  driveWheels(VELOCITY, VELOCITY);
  delay(TICK_MS);
  driveWheels(0, 0);
  resumeSensorStream();
}

/**
 * Move the robot backward for one control tick.
 */
void backwardOneTick() {
  pauseSensorStream();
  driveWheels(-VELOCITY, -VELOCITY);
  delay(TICK_MS);
  driveWheels(0, 0);
  resumeSensorStream();
}

/**
 * Turn the robot left in place for one control tick.
 */
void turnLeftOneTick() {
  pauseSensorStream();
  driveWheels(VELOCITY, -VELOCITY);
  delay(TICK_MS);
  driveWheels(0, 0);
  resumeSensorStream();
}

/**
 * Turn the robot right in place for one control tick.
 */
void turnRightOneTick() {
  pauseSensorStream();
  driveWheels(-VELOCITY, VELOCITY);
  delay(TICK_MS);
  driveWheels(0, 0);
  resumeSensorStream();
}

// Internal helper to linearly ramp wheel speeds over a number of steps.
static void rampWheels(int16_t rStart, int16_t lStart,
                       int16_t rTarget, int16_t lTarget,
                       uint8_t steps, unsigned long stepMs) {
  for (uint8_t i = 1; i <= steps; ++i) {
    // integer lerp
    int16_t r = (int16_t)(rStart + (int32_t)(rTarget - rStart) * i / steps);
    int16_t l = (int16_t)(lStart + (int32_t)(lTarget - lStart) * i / steps);
    driveWheels(r, l);
    delay(stepMs);
  }
}

// Gentle, eased turn in place for idle fidgets
void gentleTurnLeft() {
  pauseSensorStream();
  const int16_t rGoal = (int16_t)(VELOCITY / 2);  // half-speed
  const int16_t lGoal = (int16_t)(-VELOCITY / 2);
  const uint8_t steps = 3;
  const unsigned long stepMs = 30; // ~90ms ramp
  // Ramp up
  rampWheels(0, 0, rGoal, lGoal, steps, stepMs);
  // Brief hold
  delay(30);
  // Ramp down
  rampWheels(rGoal, lGoal, 0, 0, steps, stepMs);
  driveWheels(0, 0);
  resumeSensorStream();
}

void gentleTurnRight() {
  pauseSensorStream();
  const int16_t rGoal = (int16_t)(-VELOCITY / 2);
  const int16_t lGoal = (int16_t)(VELOCITY / 2);
  const uint8_t steps = 3;
  const unsigned long stepMs = 30;
  rampWheels(0, 0, rGoal, lGoal, steps, stepMs);
  delay(30);
  rampWheels(rGoal, lGoal, 0, 0, steps, stepMs);
  driveWheels(0, 0);
  resumeSensorStream();
}

// Gentle eased forward arcs
void gentleVeerLeft() {
  pauseSensorStream();
  const int16_t rGoal = (int16_t)(VELOCITY * 0.55f);
  const int16_t lGoal = (int16_t)(VELOCITY * 0.35f);
  const uint8_t steps = 3;
  const unsigned long stepMs = 30;
  rampWheels(0, 0, rGoal, lGoal, steps, stepMs);
  delay(80);
  rampWheels(rGoal, lGoal, 0, 0, steps, stepMs);
  driveWheels(0, 0);
  resumeSensorStream();
}

void gentleVeerRight() {
  pauseSensorStream();
  const int16_t rGoal = (int16_t)(VELOCITY * 0.35f);
  const int16_t lGoal = (int16_t)(VELOCITY * 0.55f);
  const uint8_t steps = 3;
  const unsigned long stepMs = 30;
  rampWheels(0, 0, rGoal, lGoal, steps, stepMs);
  delay(80);
  rampWheels(rGoal, lGoal, 0, 0, steps, stepMs);
  driveWheels(0, 0);
  resumeSensorStream();
}

/**
 * Veer left while moving forward for one control tick (gentle arc).
 */
void veerLeftOneTick() {
  int16_t fast = VELOCITY;
  int16_t slow = (VELOCITY * 3) / 5; // ~60% speed on left wheel
  pauseSensorStream();
  driveWheels(fast, slow);
  delay(TICK_MS);
  driveWheels(0, 0);
  resumeSensorStream();
}

/**
 * Veer right while moving forward for one control tick (gentle arc).
 */
void veerRightOneTick() {
  int16_t slow = (VELOCITY * 3) / 5; // ~60% speed on right wheel
  int16_t fast = VELOCITY;
  pauseSensorStream();
  driveWheels(slow, fast);
  delay(TICK_MS);
  driveWheels(0, 0);
  resumeSensorStream();
}

/**
 * Immediately stop all wheel motion.
 */
void stopAllMotors() {
  driveWheels(0, 0);
}

/**
 * Emit an audible alert when the robot freezes.
 */
void alertFreeze() {
  tone(9, 440, 100);
}

void setMotionSpeedScale(float scale) {
  if (scale < 0.05f) scale = 0.05f;
  if (scale > 1.0f) scale = 1.0f;
  SPEED_SCALE = scale;
}
