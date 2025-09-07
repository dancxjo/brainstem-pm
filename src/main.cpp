// Pro Micro Brainstem — HELLO/READY handshake + reboot, pre-handshake Safe mode with periodic note, then passthrough
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// Serial settings
static const unsigned long HOST_BAUD = 115200;   // USB CDC (host)
static const unsigned long CREATE_BAUD = 57600;  // Create 1 default (AutonomyLab)

// Power toggle wiring (DB-25 pin 3 ↔ Pro Micro D9)
#ifndef POWER_TOGGLE_PIN
#define POWER_TOGGLE_PIN 9
#endif
// Polarity: default active-low; override with -DPOWER_TOGGLE_ACTIVE_HIGH=1 if needed
#ifndef POWER_TOGGLE_ACTIVE_HIGH
#define POWER_TOGGLE_ACTIVE_HIGH 0
#endif
static const unsigned long POWER_PULSE_HIGH_MS = 100;  // pulse width
static const unsigned long POWER_OFF_SETTLE_MS = 1200; // after OFF
static const unsigned long POWER_POST_DELAY_MS = 2000; // after ON

// Small control-line parser for HELLO
static char g_ctrlBuf[16];
static uint8_t g_ctrlLen = 0;
static bool g_hostMode = false; // becomes true after READY

// Minimal OI opcodes for benign init
static const uint8_t OI_START = 128;
static const uint8_t OI_SAFE  = 131;
static const uint8_t OI_SONG  = 140;
static const uint8_t OI_PLAY  = 141;

// Pre-handshake ambient beeps (R2D2-style): spaced, gentle phrases
static unsigned long g_nextAmbientMs = 0;   // when to play next ambient phrase
static bool g_ambientDefined = false;       // whether ambient songs were defined on OI
static unsigned long g_lastOiReadyMs = 0;   // last time probe returned a byte
// Periodic OI mode assertion (START/SAFE) to catch late power-on
static const unsigned long OI_ASSERT_MS = 1000; // 1s
static unsigned long g_lastOiAssertMs = 0;
// Lightweight probe timing
static const unsigned long PROBE_WAIT_MS = 60; // wait up to 60ms for a sensor byte
static const unsigned long PROBE_INTERVAL_MS = 1000;
static unsigned long g_lastProbeMs = 0;
// Recommended inter-opcode gap (per OI spec guidance)
static const unsigned long OI_GAP_MS = 20;

// Define a small palette of pleasant phrases (IDs 0..5)
// 0: yawn (down then up), 1: stretch (upwards arpeggio), 2: soft warble,
// 3: chirp-up, 4: chirp-down, 5: trill
static void defineAmbientSongs() {
  auto defineSong = [](uint8_t id, const uint8_t* notes, const uint8_t* durs, uint8_t count) {
    CREATE_SERIAL.write(OI_SONG);
    CREATE_SERIAL.write(id);
    CREATE_SERIAL.write(count);
    for (uint8_t i = 0; i < count; ++i) {
      CREATE_SERIAL.write(notes[i]);
      CREATE_SERIAL.write(durs[i]);
    }
    delay(OI_GAP_MS);
  };

  // Song 0: Yawn (A4..C4..G4), gentle and slow
  {
    const uint8_t notes[] = {69, 67, 65, 64, 62, 60, 62, 64, 65, 67};
    const uint8_t durs[]  = {12, 12, 12, 10, 10, 10, 10, 10, 12, 14};
    defineSong(0, notes, durs, (uint8_t)(sizeof(notes)));
  }
  // Song 1: Stretch (C4 E4 G4 C5 G4)
  {
    const uint8_t notes[] = {60, 64, 67, 72, 67};
    const uint8_t durs[]  = {10, 10, 10, 12, 10};
    defineSong(1, notes, durs, (uint8_t)(sizeof(notes)));
  }
  // Song 2: Soft warble around E5
  {
    const uint8_t notes[] = {76, 75, 77, 75, 76, 74};
    const uint8_t durs[]  = {6,  6,  6,  6,  8,  8};
    defineSong(2, notes, durs, (uint8_t)(sizeof(notes)));
  }
  // Song 3: Chirp up (quick, light)
  {
    const uint8_t notes[] = {76, 79, 83};
    const uint8_t durs[]  = {4,  4,  6};
    defineSong(3, notes, durs, (uint8_t)(sizeof(notes)));
  }
  // Song 4: Chirp down (quick, light)
  {
    const uint8_t notes[] = {83, 79, 76};
    const uint8_t durs[]  = {4,  4,  6};
    defineSong(4, notes, durs, (uint8_t)(sizeof(notes)));
  }
  // Song 5: Trill (gentle alternating pair)
  {
    const uint8_t notes[] = {79, 81, 79, 81, 79, 81};
    const uint8_t durs[]  = {3,  3,  3,  3,  3,  6};
    defineSong(5, notes, durs, (uint8_t)(sizeof(notes)));
  }
  g_ambientDefined = true;
}

static inline void oiWriteDelay(uint8_t b) {
  CREATE_SERIAL.write(b);
  delay(OI_GAP_MS);
}

static void pulsePowerToggle() {
  // Drive only during pulse; tri-state otherwise to avoid unintended toggles
  if (POWER_TOGGLE_ACTIVE_HIGH) {
    pinMode(POWER_TOGGLE_PIN, OUTPUT);
    digitalWrite(POWER_TOGGLE_PIN, LOW);  // idle
    delay(2);
    digitalWrite(POWER_TOGGLE_PIN, HIGH); // active pulse
    delay(POWER_PULSE_HIGH_MS);
    digitalWrite(POWER_TOGGLE_PIN, LOW);  // release
    pinMode(POWER_TOGGLE_PIN, INPUT);     // tri-state
  } else {
    pinMode(POWER_TOGGLE_PIN, OUTPUT);
    digitalWrite(POWER_TOGGLE_PIN, HIGH); // idle
    delay(2);
    digitalWrite(POWER_TOGGLE_PIN, LOW);  // active pulse (low)
    delay(POWER_PULSE_HIGH_MS);
    digitalWrite(POWER_TOGGLE_PIN, HIGH); // release
    pinMode(POWER_TOGGLE_PIN, INPUT);     // tri-state
  }
}

void setup() {
  Serial.begin(HOST_BAUD);
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  // Leave power toggle floating until explicitly pulsed
  pinMode(POWER_TOGGLE_PIN, INPUT);
  g_ctrlLen = 0;
  g_hostMode = false;
  // Seed RNG for random note selection
  randomSeed((unsigned long)micros());
  // Attempt to place robot into Safe mode (no power toggle here)
  oiWriteDelay(OI_START);
  oiWriteDelay(OI_SAFE);
  unsigned long now = millis();
  // Schedule first ambient phrase in a few seconds
  g_nextAmbientMs = now + 3000;
  g_lastOiAssertMs = now;
  g_lastProbeMs = now;
}

// (DTMF implementation removed in favor of friendlier tones)

void loop() {
  // Host → control or passthrough
  while (Serial.available() > 0) {
    int ci = Serial.read();
    if (ci < 0) break;
    uint8_t b = (uint8_t)ci;
    if (!g_hostMode) {
      // Host began typing: ensure pleasant chirps are defined; play a light chirp per char
      if ((millis() - g_lastOiReadyMs) < 5000 && !g_ambientDefined) {
        defineAmbientSongs();
      }
      if ((millis() - g_lastOiReadyMs) < 5000 && g_ambientDefined) {
        // Pick from chirp songs 3..5 for variation
        uint8_t pick = (uint8_t)(3 + (random(3))); // 3,4,5
        uint8_t play[] = { OI_PLAY, pick };
        CREATE_SERIAL.write(play, sizeof(play));
      }
      // Accumulate an ASCII line and look for HELLO
      if (b == '\n' || b == '\r') {
        g_ctrlBuf[g_ctrlLen < sizeof(g_ctrlBuf)-1 ? g_ctrlLen : sizeof(g_ctrlBuf)-1] = '\0';
        if (strcmp(g_ctrlBuf, "HELLO") == 0) {
          Serial.println("BUSY");
          // Deterministic OFF → ON power cycle
          pulsePowerToggle();                // OFF
          delay(POWER_OFF_SETTLE_MS);
          pulsePowerToggle();                // ON
          delay(POWER_POST_DELAY_MS);
          // Minimal OI init to a benign state with proper inter-opcode gap
          oiWriteDelay(OI_START);
          oiWriteDelay(OI_SAFE);
          // Hand over to host
          Serial.println("READY");
          g_hostMode = true;
        }
        g_ctrlLen = 0;
      } else if (g_ctrlLen + 1 < sizeof(g_ctrlBuf)) {
        g_ctrlBuf[g_ctrlLen++] = (char)b;
      } else {
        // overflow; reset buffer
        g_ctrlLen = 0;
      }
      // Do not forward bytes before READY
      continue;
    }
    // Passthrough mode
    CREATE_SERIAL.write(b);
  }

  // Robot → Host (only when in passthrough)
  if (g_hostMode) {
    while (CREATE_SERIAL.available() > 0) {
      int c = CREATE_SERIAL.read();
      if (c < 0) break;
      Serial.write((uint8_t)c);
    }
  } else {
    // Pre-handshake: keep robot in SAFE and occasionally play gentle phrases
    unsigned long now = millis();
    // Re-assert OI mode periodically so if the robot powers up later, we catch it
    if (now - g_lastOiAssertMs >= OI_ASSERT_MS) {
      // Do NOT resend START repeatedly (that would drop back to PASSIVE).
      // Only re-assert SAFE to remain in Safe mode.
      oiWriteDelay(OI_SAFE);
      g_lastOiAssertMs = now;
    }
    // Lightweight probe occasionally to confirm OI is listening
    bool oi_ready = false;
    if (now - g_lastProbeMs >= PROBE_INTERVAL_MS) {
      // Query packet 7 (one byte). If any byte comes back quickly, consider ready.
      while (CREATE_SERIAL.available() > 0) (void)CREATE_SERIAL.read();
      CREATE_SERIAL.write((uint8_t)142); // OI_SENSORS
      CREATE_SERIAL.write((uint8_t)7);
      unsigned long until = now + PROBE_WAIT_MS;
      while (millis() < until) {
        if (CREATE_SERIAL.available() > 0) { (void)CREATE_SERIAL.read(); oi_ready = true; break; }
      }
      g_lastProbeMs = now;
      if (oi_ready) g_lastOiReadyMs = now;
    }
    // Define ambient songs once after OI responds
    if (!g_ambientDefined && (now - g_lastOiReadyMs) < 5000) {
      defineAmbientSongs();
    }
    // Occasionally play a gentle phrase if OI seems ready
    if (g_ambientDefined && (now - g_lastOiReadyMs) < 5000 && now >= g_nextAmbientMs) {
      // Choose one of the ambient songs (0..2) mostly; occasionally a chirp (3..5)
      uint8_t base = (random(4) == 0) ? 3 : 0; // 25% pick chirp bank
      uint8_t songId = base + (uint8_t)random(3); // 0..2 or 3..5
      uint8_t play[] = { OI_PLAY, songId };
      CREATE_SERIAL.write(play, sizeof(play));
      // Next phrase in 4–9 seconds
      g_nextAmbientMs = now + 4000 + (unsigned long)random(5000);
    }
  }
}
