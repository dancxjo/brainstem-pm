#include "motion.h"
#include <Arduino.h>
#include "sensors.h"  // for pause/resume of OI sensor stream during blocking motions

// iRobot Create 1 Open Interface opcodes
static constexpr uint8_t OI_START = 128;
static constexpr uint8_t OI_SAFE = 131;
static constexpr uint8_t OI_DRIVE_DIRECT = 145;
static constexpr uint16_t VELOCITY = 200; // mm/s
static constexpr unsigned long TICK_MS = 100; // duration of one tick

/**
 * Helper to send direct wheel speeds to the Create.
 * @param right Right wheel velocity in mm/s
 * @param left  Left wheel velocity in mm/s
 */
static void driveWheels(int16_t right, int16_t left) {
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
  Serial1.write(OI_SAFE);
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
