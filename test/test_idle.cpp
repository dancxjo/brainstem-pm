#include <unity.h>
#include "idle.h"
#include "leds.h"
#include "sensors.h"
#include "Arduino.h"

HardwareSerial Serial1; // mock

void setUp() {
  Serial1.clear();
  setBatteryPercentOverride(-1);
  initIdle(100); // short timeout for tests
}

void test_idle_activation_after_timeout() {
  for (int i = 0; i < 15; ++i) updateIdle(false); // advance >100ms
  TEST_ASSERT_TRUE(idleIsActive());
  TEST_ASSERT_EQUAL(PATTERN_IDLE, getLedPattern());
}

void test_idle_deactivation_on_usb_connect() {
  for (int i = 0; i < 15; ++i) updateIdle(false);
  TEST_ASSERT_TRUE(idleIsActive());
  updateIdle(true);
  TEST_ASSERT_FALSE(idleIsActive());
}

void test_low_battery_triggers_sleep() {
  setBatteryPercentOverride(10);
  updateIdle(false);
  TEST_ASSERT_TRUE(idleIsSleeping());
  TEST_ASSERT_EQUAL(PATTERN_ALERT, getLedPattern());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_idle_activation_after_timeout);
  RUN_TEST(test_idle_deactivation_on_usb_connect);
  RUN_TEST(test_low_battery_triggers_sleep);
  return UNITY_END();
}
