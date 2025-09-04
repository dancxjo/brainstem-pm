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
static const uint8_t OI_STREAM  = 148; // Start data stream
static const uint8_t OI_PAUSE   = 150; // Pause/Resume data stream

// Optional external bumper interrupt (active-low microswitch on a GPIO)
#ifndef BUMPER_PIN
#define BUMPER_PIN 3
#endif
#ifndef BUMPER_ACTIVE_LOW
#define BUMPER_ACTIVE_LOW 1
#endif
static volatile bool bumperEventFlag = false;
static void bumperIsr() {
#if BUMPER_ACTIVE_LOW
  if (digitalRead(BUMPER_PIN) == LOW)
    bumperEventFlag = true;
#else
  if (digitalRead(BUMPER_PIN) == HIGH)
    bumperEventFlag = true;
#endif
}

// Cached sensor values from OI stream
static bool cachedBumpLeft = false;
static bool cachedBumpRight = false;
static bool cachedCliffL = false, cachedCliffFL = false, cachedCliffFR = false, cachedCliffR = false;

// Stream parsing state
enum StreamParseState { WAIT_HEADER, WAIT_LEN, READ_PAYLOAD, WAIT_CHECKSUM };
static StreamParseState spState = WAIT_HEADER;
static uint8_t spLen = 0;   // OI stream length is one byte
static uint8_t spRead = 0;
// Stream single-byte packets only to keep parsing simple:
//  - 7  = Bumps/Wheel Drops (1 byte)
//  - 9  = Cliff Left (1 byte)
//  - 10 = Cliff Front Left (1 byte)
//  - 11 = Cliff Front Right (1 byte)
//  - 12 = Cliff Right (1 byte)
//  - 18 = Buttons (1 byte)
//  - 8  = Wall (boolean, 1 byte)
static const uint8_t requestedPackets[] = { 7, 9, 10, 11, 12, 18, 8 };
static uint8_t payloadBuf[32];
static unsigned long lastStreamMs = 0;
// Cached wall and button edges
static bool cachedWall = false;
static uint8_t lastButtons = 0;
static volatile bool btnPlayEdge = false;
static volatile bool btnAdvEdge = false;

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
  if (!readBytes(&b, 1, 20)) {
    Serial.print("[SENS] pkt "); Serial.print((int)packetId);
    Serial.println(" timeout");
    return false;
  }
  Serial.print("[SENS] pkt "); Serial.print((int)packetId);
  Serial.print(" = "); Serial.println((int)b);
  out = b;
  return true;
}

void beginSensorStream() {
  // Pause any existing stream, configure, then resume
  CREATE_SERIAL.write(OI_PAUSE);
  CREATE_SERIAL.write((uint8_t)0); // pause
  CREATE_SERIAL.write(OI_STREAM);
  CREATE_SERIAL.write((uint8_t)(sizeof(requestedPackets)));
  for (uint8_t i = 0; i < sizeof(requestedPackets); ++i) {
    CREATE_SERIAL.write(requestedPackets[i]);
  }
  CREATE_SERIAL.write(OI_PAUSE);
  CREATE_SERIAL.write((uint8_t)1); // resume
  // Reset parser state and drain any stale bytes
  spState = WAIT_HEADER;
  spLen = 0;
  spRead = 0;
  while (CREATE_SERIAL.available()) { (void)CREATE_SERIAL.read(); }
  Serial.println("[SENS] OI stream started (7,9,10,11,12,18,8)");
}

void pauseSensorStream() {
  // Pause any ongoing OI stream
  CREATE_SERIAL.write(OI_PAUSE);
  CREATE_SERIAL.write((uint8_t)0);
}

void resumeSensorStream() {
  // Resume an already configured OI stream
  CREATE_SERIAL.write(OI_PAUSE);
  CREATE_SERIAL.write((uint8_t)1);
}

void updateSensorStream() {
  while (CREATE_SERIAL.available()) {
    int bi = CREATE_SERIAL.read();
    if (bi < 0) break;
    uint8_t b = (uint8_t)bi;
    switch (spState) {
      case WAIT_HEADER:
        if (b == 19) spState = WAIT_LEN;
        break;
      case WAIT_LEN:
        spLen = b; // one byte length
        if (spLen == 0 || spLen > sizeof(payloadBuf)) {
          Serial.print("[SENS] stream len invalid: "); Serial.println((int)spLen);
          spState = WAIT_HEADER; // resync to next header
        } else {
          spRead = 0;
          spState = READ_PAYLOAD;
        }
        break;
      case READ_PAYLOAD:
        payloadBuf[spRead++] = b;
        if (spRead >= spLen) spState = WAIT_CHECKSUM;
        break;
      case WAIT_CHECKSUM: {
        // Verify checksum: sum(header, len, payload, checksum) & 0xFF == 0
        uint16_t sum = 19;
        sum += spLen;
        for (uint8_t i = 0; i < spLen; ++i) sum += payloadBuf[i];
        sum += b;
        if ((sum & 0xFF) == 0) {
          // Assume ID/Value pairs per OI manual: [id][value] ... in any order
          if ((spLen % 2) != 0) {
            Serial.print("[SENS] stream odd len="); Serial.println((int)spLen);
            Serial.print("[SENS] stream raw: ");
            for (uint8_t i = 0; i < spLen; ++i) {
              Serial.print((int)payloadBuf[i]);
              if (i < spLen - 1) Serial.print(",");
            }
            Serial.println();
          } else {
            for (uint8_t i = 0; i + 1 < spLen; i += 2) {
              uint8_t id = payloadBuf[i];
              uint8_t val = payloadBuf[i + 1];
              switch (id) {
                case 7:  cachedBumpRight = (val & 0x01) != 0; cachedBumpLeft = (val & 0x02) != 0; break;
                case 9:  cachedCliffL  = (val != 0); break;
                case 10: cachedCliffFL = (val != 0); break;
                case 11: cachedCliffFR = (val != 0); break;
                case 12: cachedCliffR  = (val != 0); break;
                case 18: {
                  // Buttons: assume bit0=Play, bit2=Advance (Create 1 OI)
                  uint8_t prev = lastButtons;
                  lastButtons = val;
                  if ((!(prev & 0x01)) && (val & 0x01)) btnPlayEdge = true;
                  if ((!(prev & 0x04)) && (val & 0x04)) btnAdvEdge = true;
                  break;
                }
                case 8: cachedWall = (val != 0); break; // Wall sensor boolean
                default: break; // ignore others if present
              }
            }
            lastStreamMs = millis();
            Serial.print("[SENS] stream bumps L="); Serial.print((int)cachedBumpLeft);
            Serial.print(" R="); Serial.print((int)cachedBumpRight);
            Serial.print(" cliffs=");
            Serial.print((int)cachedCliffL); Serial.print(",");
            Serial.print((int)cachedCliffFL); Serial.print(",");
            Serial.print((int)cachedCliffFR); Serial.print(",");
            Serial.print((int)cachedCliffR);
            Serial.print(" wall="); Serial.print((int)cachedWall);
            Serial.print(" btn="); Serial.println((int)lastButtons);
          }
        } else {
          Serial.println("[SENS] stream checksum error; resyncing");
        }
        spState = WAIT_HEADER;
        break;
      }
    }
  }
}

bool oiConnected() {
  // Consider connected if we saw a valid stream frame recently
  unsigned long now = millis();
  return (lastStreamMs != 0) && (now - lastStreamMs < 1000);
}

void initSensors() {
  // Nothing to initialize beyond the OI connection handled elsewhere.
  // Try to attach an external interrupt for a GPIO-wired bumper.
#ifdef ARDUINO
  pinMode(BUMPER_PIN, BUMPER_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
#  ifdef digitalPinToInterrupt
  int irq = digitalPinToInterrupt(BUMPER_PIN);
  if (irq != NOT_AN_INTERRUPT) {
    attachInterrupt(irq, bumperIsr, CHANGE);
    Serial.print("[SENS] Bumper ISR attached on pin ");
    Serial.println(BUMPER_PIN);
  } else {
    Serial.println("[SENS] Bumper ISR not available on this pin");
  }
#  endif
#endif
}

int scanEnvironment() {
  // Placeholder for directional stimulus sensing (e.g., IR beacons).
  // For now, we return neutral (0). Extend by querying IR/opcode as needed.
  return 0;
}

bool bumperTriggered() {
  bool any = (cachedBumpLeft || cachedBumpRight);
  if (any) {
    Serial.print("[SENS] bumperTriggered via stream L=");
    Serial.print((int)cachedBumpLeft);
    Serial.print(" R=");
    Serial.println((int)cachedBumpRight);
  }
  return any;
}

bool cliffDetected() {
  bool any = (cachedCliffL || cachedCliffFL || cachedCliffFR || cachedCliffR);
  if (any) Serial.println("[SENS] cliff detected via stream");
  return any;
}

bool bumperEventTriggeredAndClear() {
  bool was = bumperEventFlag;
  bumperEventFlag = false;
  if (was) Serial.println("[SENS] bumper ISR event");
  return was;
}

bool wallDetected() { return cachedWall; }

bool playButtonPressedAndClear() {
  bool was = btnPlayEdge;
  btnPlayEdge = false;
  return was;
}

bool advanceButtonPressedAndClear() {
  bool was = btnAdvEdge;
  btnAdvEdge = false;
  return was;
}
