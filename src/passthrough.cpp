#include "passthrough.h"
#include "sensors.h"
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

extern bool tx_paused;

static bool g_passthrough = false;

void passthroughEnable() {
  if (!g_passthrough) {
    g_passthrough = true;
    tx_paused = true;
    pauseSensorStream();
  }
}

void passthroughDisable() {
  if (g_passthrough) {
    g_passthrough = false;
    tx_paused = false;
    resumeSensorStream();
  }
}

bool passthroughActive() { return g_passthrough; }

void passthroughPump() {
  while (g_passthrough && Serial.available()) {
    int c = Serial.read();
    if (c == 0) {
      passthroughDisable();
      break;
    }
    if (c >= 0) CREATE_SERIAL.write((uint8_t)c);
  }
  if (!g_passthrough) return;
  while (CREATE_SERIAL.available()) {
    int c = CREATE_SERIAL.read();
    if (c >= 0) Serial.write((uint8_t)c);
  }
}
