// Minimal USB <-> iRobot Create 1 serial proxy
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// AutonomyLab create_1 drivers expect 57600 8N1 on the robot link
static const unsigned long CREATE_BAUD = 57600;

void setup() {
  // Host-side USB CDC: baud often ignored, but set for clarity
  Serial.begin(CREATE_BAUD);
  // Robot-side UART
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
}

void loop() {
  // Host -> Robot
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    CREATE_SERIAL.write((uint8_t)c);
  }
  // Robot -> Host
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
}

