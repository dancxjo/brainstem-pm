// Pro Micro Brainstem — Wake, Handshake, and Passthrough for iRobot Create 1
#include <Arduino.h>
#ifdef ARDUINO_AVR_LEONARDO
#include <avr/wdt.h>
#endif

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// Constants
static const unsigned long HOST_BAUD = 115200;  // USB CDC to midbrain
static const unsigned long CREATE_BAUD = 57600; // Create OI default (pin 15 floating)

// Power toggle wiring: DB-25 pin 3 ↔ Pro Micro D9
#ifndef POWER_TOGGLE_PIN
#define POWER_TOGGLE_PIN 9
#endif

// Timing
static const unsigned long POWER_PULSE_HIGH_MS = 100;
static const unsigned long POWER_POST_DELAY_MS = 2000;
static const unsigned long OI_WRITE_GAP_MS     = 20;

// Escape prefix for control during bridge
#define BRIDGE_ESCAPE 0xFF
static const uint8_t BRIDGE_ESCAPE_FOLLOW = 0x00;

// OI opcodes
static const uint8_t OI_START = 128;
static const uint8_t OI_SAFE  = 131;
static const uint8_t OI_SONG  = 140;
static const uint8_t OI_PLAY  = 141;
static const uint8_t OI_LED   = 139;

// State machine
enum State { IDLE, POWER_SEQUENCE, OI_INIT, BRIDGE_READY };
static State g_state = IDLE;

// Counters/metrics
static unsigned long g_toRobot = 0; // USB->Robot bridged bytes
static unsigned long g_toHost  = 0; // Robot->USB bridged bytes
static const char* g_lastErr = "";

// Control line buffer (for HELLO/!commands)
static char g_ctrlBuf[128];
static size_t g_ctrlLen = 0;

// Escape-state for bridge input
static uint8_t g_escState = 0; // 0=normal,1=seen FF awaiting 00,2=capturing control line

// Helpers
static void oiWrite(uint8_t b) { CREATE_SERIAL.write(b); delay(OI_WRITE_GAP_MS); }

static void pulsePower() {
  // Low initially; a low->high->low pulse toggles power
  digitalWrite(POWER_TOGGLE_PIN, HIGH);
  delay(POWER_PULSE_HIGH_MS);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
}

static void oiDefineAndPlayCute() {
  // Define a short song: C4,E4,G4,C5
  const uint8_t song[] = { OI_SONG, 0, 4, 60, 16, 64, 16, 67, 16, 72, 24 };
  CREATE_SERIAL.write(song, sizeof(song));
  delay(30);
  const uint8_t play[] = { OI_PLAY, 0 };
  CREATE_SERIAL.write(play, sizeof(play));
  // LED sweep on the power LED: color 0..255, intensity 30..255
  for (int i = 0; i <= 255; i += 32) {
    CREATE_SERIAL.write(OI_LED);
    CREATE_SERIAL.write((uint8_t)0);            // LED bits off; power LED color used
    CREATE_SERIAL.write((uint8_t)i);            // color (green→red on some models)
    uint8_t intensity = (uint8_t)min(255, 30 + i);
    CREATE_SERIAL.write(intensity);
    delay(30);
  }
}

static void enterBridgeReady() {
  g_state = BRIDGE_READY;
  Serial.println("READY");
}

static void startPowerSequence() {
  Serial.println("BUSY");
  g_state = POWER_SEQUENCE;
}

static void doOiInit() {
  // Initialize robot OI and play wake sequence
  oiWrite(OI_START);
  oiWrite(OI_SAFE);
  oiDefineAndPlayCute();
}

static void handleStatus() {
  const char* sname = (g_state == BRIDGE_READY) ? "BRIDGE_READY" :
                      (g_state == OI_INIT) ? "OI_INIT" :
                      (g_state == POWER_SEQUENCE) ? "POWER_SEQUENCE" : "IDLE";
  Serial.print("STATUS:{\"state\":\"");
  Serial.print(sname);
  Serial.print("\",\"baud\":");
  Serial.print((int)CREATE_BAUD);
  Serial.print(",\"rx_to_robot\":");
  Serial.print(g_toRobot);
  Serial.print(",\"tx_to_host\":");
  Serial.print(g_toHost);
  Serial.print(",\"last_error\":\"");
  Serial.print(g_lastErr);
  Serial.println("\"}");
}

static void softReboot() {
#ifdef ARDUINO_AVR_LEONARDO
  wdt_enable(WDTO_15MS);
  while (1) {}
#else
  // Fallback: indicate error
  Serial.println("ERR:reboot_unsupported");
#endif
}

static void handleControlLine(const char* line) {
  // Trim trailing CR/LF
  size_t n = strlen(line);
  while (n && (line[n-1] == '\r' || line[n-1] == '\n')) n--;
  // Compare
  if (n == 5 && strncmp(line, "HELLO", 5) == 0) {
    if (g_state == BRIDGE_READY) { Serial.println("READY"); return; }
    startPowerSequence();
    return;
  }
  if (n == 12 && strncmp(line, "!power_cycle", 12) == 0) { startPowerSequence(); return; }
  if (n == 5 && strncmp(line, "!cute", 5) == 0) { oiDefineAndPlayCute(); return; }
  if (n == 7 && strncmp(line, "!status", 7) == 0) { handleStatus(); return; }
  if (n == 7 && strncmp(line, "!reboot", 7) == 0) { softReboot(); return; }
  Serial.println("ERR:unknown_cmd");
}

void setup() {
  Serial.begin(HOST_BAUD);
  CREATE_SERIAL.begin(CREATE_BAUD, SERIAL_8N1);
  pinMode(POWER_TOGGLE_PIN, OUTPUT);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
  g_state = IDLE;
  g_toRobot = g_toHost = 0;
  g_lastErr = "";
  g_ctrlLen = 0; g_escState = 0;
}

void loop() {
  // USB → (control or robot)
  while (Serial.available() > 0) {
    int ci = Serial.read();
    if (ci < 0) break;
    uint8_t b = (uint8_t)ci;
    if (g_state == BRIDGE_READY) {
      // Bridge with escape handling
      if (g_escState == 0) {
        if (b == BRIDGE_ESCAPE) { g_escState = 1; continue; }
        CREATE_SERIAL.write(b); g_toRobot++;
      } else if (g_escState == 1) {
        if (b == BRIDGE_ESCAPE_FOLLOW) { g_escState = 2; g_ctrlLen = 0; continue; }
        // Not our escape; forward both bytes
        CREATE_SERIAL.write(BRIDGE_ESCAPE); g_toRobot++;
        CREATE_SERIAL.write(b); g_toRobot++;
        g_escState = 0;
      } else { // capturing control line
        if (b == '\n') {
          g_ctrlBuf[g_ctrlLen < sizeof(g_ctrlBuf)-1 ? g_ctrlLen : sizeof(g_ctrlBuf)-1] = '\0';
          handleControlLine(g_ctrlBuf);
          g_ctrlLen = 0; g_escState = 0;
        } else if (g_ctrlLen + 1 < sizeof(g_ctrlBuf)) {
          g_ctrlBuf[g_ctrlLen++] = (char)b;
        }
      }
    } else {
      // Not in bridge: parse plain ASCII lines for HELLO/!cmd
      if (b == '\n') {
        g_ctrlBuf[g_ctrlLen < sizeof(g_ctrlBuf)-1 ? g_ctrlLen : sizeof(g_ctrlBuf)-1] = '\0';
        handleControlLine(g_ctrlBuf);
        g_ctrlLen = 0;
      } else if (g_ctrlLen + 1 < sizeof(g_ctrlBuf)) {
        g_ctrlBuf[g_ctrlLen++] = (char)b;
      }
    }
  }

  // Robot → Host (only in bridge)
  if (g_state == BRIDGE_READY) {
    while (CREATE_SERIAL.available() > 0) {
      int ri = CREATE_SERIAL.read();
      if (ri < 0) break;
      Serial.write((uint8_t)ri);
      g_toHost++;
    }
  }

  // State machine actions (non-blocking where possible)
  static unsigned long t0 = 0;
  switch (g_state) {
    case POWER_SEQUENCE:
      pulsePower();
      t0 = millis();
      g_state = OI_INIT;
      break;
    case OI_INIT:
      if (millis() - t0 >= POWER_POST_DELAY_MS) {
        doOiInit();
        enterBridgeReady();
      }
      break;
    default: break;
  }
}
