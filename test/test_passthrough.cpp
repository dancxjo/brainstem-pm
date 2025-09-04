#include <unity.h>
#include "passthrough.h"
#include "Arduino.h"

HardwareSerial Serial1; // mock robot serial

void setUp() {
  Serial.clear();
  Serial1.clear();
}

void test_usb_to_robot() {
  passthroughEnable();
  Serial.clear();
  Serial1.clear();
  Serial.rx = {0x55, 0xAA};
  passthroughPump();
  TEST_ASSERT_EQUAL_INT(2, Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8(0x55, Serial1.buffer[0]);
  TEST_ASSERT_EQUAL_UINT8(0xAA, Serial1.buffer[1]);
}

void test_robot_to_usb() {
  passthroughEnable();
  Serial.clear();
  Serial1.clear();
  Serial1.rx = {0x10, 0x20};
  passthroughPump();
  TEST_ASSERT_EQUAL_INT(2, Serial.buffer.size());
  TEST_ASSERT_EQUAL_UINT8(0x10, Serial.buffer[0]);
  TEST_ASSERT_EQUAL_UINT8(0x20, Serial.buffer[1]);
}

void test_exit_on_nul() {
  passthroughEnable();
  Serial.clear();
  Serial1.clear();
  Serial.rx = {0x00, 0x42};
  passthroughPump();
  TEST_ASSERT_FALSE(passthroughActive());
  TEST_ASSERT_EQUAL_INT(0, Serial1.buffer.size());
  TEST_ASSERT_EQUAL_INT(1, Serial.rx.size());
  TEST_ASSERT_EQUAL_UINT8(0x42, Serial.rx[0]);
}

void test_reenable_passthrough() {
  passthroughEnable();
  Serial.rx = {0x00};
  passthroughPump();
  TEST_ASSERT_FALSE(passthroughActive());
  Serial.clear();
  Serial1.clear();
  passthroughEnable();
  Serial.rx = {0x33};
  passthroughPump();
  TEST_ASSERT_TRUE(passthroughActive());
  TEST_ASSERT_EQUAL_INT(1, Serial1.buffer.size());
  TEST_ASSERT_EQUAL_UINT8(0x33, Serial1.buffer[0]);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_usb_to_robot);
  RUN_TEST(test_robot_to_usb);
  RUN_TEST(test_exit_on_nul);
  RUN_TEST(test_reenable_passthrough);
  return UNITY_END();
}
