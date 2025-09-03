#include <unity.h>
#include "motion.h"
#include "Arduino.h"

HardwareSerial Serial1; // define the mock serial

void setUp() {
  Serial1.clear();
}

void test_initMotors() {
  initMotors();
  TEST_ASSERT_EQUAL_UINT8(128, Serial1.buffer[0]);
  TEST_ASSERT_EQUAL_UINT8(131, Serial1.buffer[1]);
}

void test_forwardOneTick() {
  forwardOneTick();
  const uint8_t expected[] = {145, 0x00, 0xC8, 0x00, 0xC8, 145, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_INT(sizeof(expected), Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, Serial1.buffer.data(), sizeof(expected));
}

void test_backwardOneTick() {
  backwardOneTick();
  const uint8_t expected[] = {145, 0xFF, 0x38, 0xFF, 0x38, 145, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_INT(sizeof(expected), Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, Serial1.buffer.data(), sizeof(expected));
}

void test_turnLeftOneTick() {
  turnLeftOneTick();
  const uint8_t expected[] = {145, 0x00, 0xC8, 0xFF, 0x38, 145, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_INT(sizeof(expected), Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, Serial1.buffer.data(), sizeof(expected));
}

void test_turnRightOneTick() {
  turnRightOneTick();
  const uint8_t expected[] = {145, 0xFF, 0x38, 0x00, 0xC8, 145, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_INT(sizeof(expected), Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, Serial1.buffer.data(), sizeof(expected));
}

void test_stopAllMotors() {
  stopAllMotors();
  const uint8_t expected[] = {145, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_INT(sizeof(expected), Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, Serial1.buffer.data(), sizeof(expected));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_initMotors);
  RUN_TEST(test_forwardOneTick);
  RUN_TEST(test_backwardOneTick);
  RUN_TEST(test_turnLeftOneTick);
  RUN_TEST(test_turnRightOneTick);
  RUN_TEST(test_stopAllMotors);
  return UNITY_END();
}
