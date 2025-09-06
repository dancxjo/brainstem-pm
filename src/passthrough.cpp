#include "passthrough.h"
#include "sensors.h"
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

extern bool tx_paused;

static bool g_passthrough = false;
// Which OI PLAY song triggers exit from passthrough to interpreter/managed mode.
// Can be overridden via build flags: -DHANDSHAKE_SONG=<n>
#ifndef HANDSHAKE_SONG
#define HANDSHAKE_SONG 12
#endif
static const uint8_t OI_PLAY = 141;
static int handshakeState = 0; // 0=normal, 1=seen OI_PLAY awaiting song byte

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

// Notify core when USB activity occurs (defined in main.cpp for UART build; stubbed in tests)
extern void usbLinkActivity();

// Extern hook to enter managed mode when handshake is detected (implemented in main.cpp)
extern void enterForebrainModeFromPassthrough(uint8_t songId);

void passthroughPump() {
  // Host → Robot
  while (g_passthrough && Serial.available()) {
    int c = Serial.read();
    if (c >= 0) {
      usbLinkActivity(); // mark USB as active without emitting any messages
      uint8_t b = (uint8_t)c;
      // Detect OI_PLAY,<HANDSHAKE_SONG> pattern; swallow it and exit passthrough
      if (handshakeState == 0) {
        if (b == OI_PLAY) {
          handshakeState = 1;
          // Do not forward yet; wait to inspect next byte
          continue;
        }
        // Normal byte passthrough
        CREATE_SERIAL.write(b);
      } else if (handshakeState == 1) {
        // This byte is the song id
        if (b == (uint8_t)HANDSHAKE_SONG) {
          // Swallow the handshake and switch to managed mode
          handshakeState = 0;
          enterForebrainModeFromPassthrough(b);
          passthroughDisable();
          break;
        } else {
          // Not our handshake. Forward both bytes (OI_PLAY and this id)
          CREATE_SERIAL.write(OI_PLAY);
          CREATE_SERIAL.write(b);
          handshakeState = 0;
        }
      }
    }
  }
  if (!g_passthrough) return;

  // Robot → Host
  while (CREATE_SERIAL.available()) {
    int c = CREATE_SERIAL.read();
    if (c >= 0) {
      // Writing to USB also implies link is up; mark activity
      usbLinkActivity();
      Serial.write((uint8_t)c);
    }
  }
}
