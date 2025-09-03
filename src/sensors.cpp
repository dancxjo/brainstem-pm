// Read Create 1 sensors via the Open Interface (OI)
// Reference: OI opcode 142 (Sensors), with packet IDs such as 7 (Bumps/Wheel Drops)
// and 9-12 (Cliff Left, Front Left, Front Right, Right). Each of these packets
// returns one byte where non-zero means the event is active.

#include "sensors.h"
#include <Arduino.h>

// Select the hardware serial used to talk to the Create OI.
#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

static const uint8_t OI_SENSORS = 142; // Query single sensor packet

// Read exactly len bytes from CREATE_SERIAL within timeoutMs. Returns true if full buffer read.
static bool readBytes(uint8_t* dst, size_t len, unsigned long timeoutMs) {
  unsigned long start = millis();
  size_t got = 0;
  while (got < len) {
    while (CREATE_SERIAL.available() && got < len) {
      int c = CREATE_SERIAL.read();
      if (c < 0) break;
      dst[got++] = static_cast<uint8_t>(c);
    }
    if (got >= len) break;
    if (millis() - start > timeoutMs) return false;
    // brief pause to yield
    delay(1);
  }
  return got == len;
}

// Query a single-byte sensor packet by ID. Returns true and fills out if a byte was read.
static bool queryBytePacket(uint8_t packetId, uint8_t &out) {
  CREATE_SERIAL.write(OI_SENSORS);
  CREATE_SERIAL.write(packetId);
  uint8_t b = 0;
  if (!readBytes(&b, 1, 20)) return false;
  out = b;
  return true;
}

void initSensors() {
  // Nothing to initialize beyond the OI connection handled elsewhere.
}

int scanEnvironment() {
  // Placeholder for directional stimulus sensing (e.g., IR beacons).
  // For now, we return neutral (0). Extend by querying IR/opcode as needed.
  return 0;
}

bool bumperTriggered() {
  // Packet 7: Bumps and Wheel Drops (1 byte)
  // bit0 = right bump, bit1 = left bump (Create/Roomba OI)
  uint8_t v = 0;
  if (!queryBytePacket(7, v)) return false;
  bool rightBump = (v & 0x01) != 0;
  bool leftBump  = (v & 0x02) != 0;
  return rightBump || leftBump;
}

bool cliffDetected() {
  // Packets 9-12: cliff sensors (Left, Front Left, Front Right, Right); 1 if cliff.
  uint8_t b = 0;
  if (queryBytePacket(9, b) && b) return true;
  if (queryBytePacket(10, b) && b) return true;
  if (queryBytePacket(11, b) && b) return true;
  if (queryBytePacket(12, b) && b) return true;
  return false;
}
