// Minimal USB CDC ↔ iRobot Create 1 UART passthrough
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

static const unsigned long HOST_BAUD = 115200;   // USB CDC (host)
static const unsigned long CREATE_BAUD = 57600;  // Create 1 default (AutonomyLab)

void setup() {
  Serial.begin(HOST_BAUD);
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
}

void loop() {
  // Host → Robot
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    CREATE_SERIAL.write((uint8_t)c);
  }
  // Robot → Host
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
}
