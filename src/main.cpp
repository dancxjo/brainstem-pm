// Pro Micro Brainstem — Always-on Create 1 Link + Raw Passthrough
#include <Arduino.h>

#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif

// Host and robot serial settings
static const unsigned long HOST_BAUD = 115200;   // USB CDC (for host)
static const unsigned long BAUDS[] = {57600, 19200}; // Try 57600 first, then 19200 fallback
static uint8_t g_baudIdx = 0;

// Open Interface (subset)
static const uint8_t OI_START = 128;
static const uint8_t OI_SAFE  = 131;
static const uint8_t OI_SONG  = 140;
static const uint8_t OI_PLAY  = 141;
static const uint8_t OI_LED   = 139;
static const uint8_t OI_SENSORS = 142; // Sensor query

// Connection management timings
static const unsigned long CONNECT_TRY_MS    = 500;   // cadence for attempts when disconnected
static const unsigned long PROBE_INTERVAL_MS = 1000;  // how often to probe when connected
static const unsigned long SILENCE_DROP_MS   = 4000;  // if no robot RX for this long, mark disconnected
static const unsigned long PROBE_WAIT_MS     = 60;    // wait for sensor byte after probe
// Heartbeat/Jingle cadence (randomized around these)
static const unsigned long HEARTBEAT_MIN_MS = 6000;  // 6s
static const unsigned long HEARTBEAT_MAX_MS = 12000; // 12s
// Power toggle control (DB-25 pin 3 ↔ Pro Micro D9)
#ifndef POWER_TOGGLE_PIN
#define POWER_TOGGLE_PIN 9
#endif
static const unsigned long POWER_PULSE_HIGH_MS = 100; // ms high for a toggle
static const unsigned long POWER_OFF_SETTLE_MS = 1200; // ms to allow power-down
static const unsigned long POWER_POST_DELAY_MS = 2000; // ms to allow boot after power-on

// Metrics and connection state
static bool g_connected = false;
static unsigned long g_lastTryMs = 0;
static unsigned long g_lastProbeMs = 0;
static unsigned long g_lastRobotRxMs = 0;
static unsigned long g_nextBeatMs = 0;
static bool g_hostMode = false;        // true after host handshake (HELLO)

// Simple control-line buffer for early HELLO detection
static char g_ctrlBuf[16];
static uint8_t g_ctrlLen = 0;

static void pulsePowerToggle() {
  digitalWrite(POWER_TOGGLE_PIN, HIGH);
  delay(POWER_PULSE_HIGH_MS);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
}

static void beginRobotUart() {
  CREATE_SERIAL.end();
  CREATE_SERIAL.begin(BAUDS[g_baudIdx], SERIAL_8N1);
}

static bool probeOiAndMaybeRead() {
  // Drain any stale input first
  while (CREATE_SERIAL.available() > 0) (void)CREATE_SERIAL.read();
  // Send a single sensor query for packet 7 (1 byte)
  CREATE_SERIAL.write(OI_SENSORS);
  CREATE_SERIAL.write((uint8_t)7);
  unsigned long until = millis() + PROBE_WAIT_MS;
  while (millis() < until) {
    if (CREATE_SERIAL.available() > 0) {
      (void)CREATE_SERIAL.read();
      return true;
    }
  }
  return false;
}

static inline void oiWrite(uint8_t b) { CREATE_SERIAL.write(b); }

static void playHeartbeatLively() {
  // Ensure we're in a mode that accepts LED/song
  oiWrite(OI_SAFE);
  // Pick 1–3 brief notes from a pleasant set
  const uint8_t scale[] = {60, 64, 67, 72, 76}; // C, E, G, C', C', E'
  uint8_t n = (uint8_t)(1 + (random(3))); // 1..3 notes
  uint8_t def[2 + 1 + 2*3]; // opcode, song#, count, up to 3 pairs
  def[0] = OI_SONG; def[1] = 1; def[2] = n;
  for (uint8_t i = 0; i < n; ++i) {
    uint8_t note = scale[random((long)sizeof(scale))];
    uint8_t dur = (uint8_t)(4 + random(5)); // 4..8 (1/64s)
    def[3 + 2*i] = note;
    def[4 + 2*i] = dur;
  }
  CREATE_SERIAL.write(def, (size_t)(3 + 2*n));
  const uint8_t play[] = { OI_PLAY, 1 };
  CREATE_SERIAL.write(play, sizeof(play));
  // LED: brighter, random color pulse so it's noticeable
  uint8_t color = (uint8_t)random(256);
  uint8_t intensity = (uint8_t)(120 + (random(101))); // 120..220 (cap by 255)
  CREATE_SERIAL.write(OI_LED);
  CREATE_SERIAL.write((uint8_t)0);
  CREATE_SERIAL.write(color);
  CREATE_SERIAL.write(intensity);
}

static void tryConnectTick() {
  unsigned long now = millis();
  if (!g_connected) {
    if (now - g_lastTryMs >= CONNECT_TRY_MS) {
      // Cycle baud candidates every attempt window
      beginRobotUart();
      // Minimal handshake to claim OI; Safe mode keeps protections
      oiWrite(OI_START);
      oiWrite(OI_SAFE);
      // Actively probe for a response byte at this baud
      if (probeOiAndMaybeRead()) {
        g_connected = true;
        g_lastRobotRxMs = now;
        // Schedule a subtle heartbeat soon after initial connect (no immediate loud cue)
        g_nextBeatMs = now + 15000; // first beat after 15s
      } else {
        // Try next baud on next attempt
        g_baudIdx = (g_baudIdx + 1) % (sizeof(BAUDS)/sizeof(BAUDS[0]));
      }
      g_lastTryMs = now;
    }
  } else {
    // Periodic probe to keep link warm (and to tickle any dormant OI)
    if (now - g_lastProbeMs >= PROBE_INTERVAL_MS) {
      // Probe sensors; if we fail to see a byte, we'll drop after SILENCE_DROP_MS below
      (void)probeOiAndMaybeRead();
      g_lastProbeMs = now;
    }
    // Subtle, well-spaced heartbeat while we own the link (pre-host)
    if (g_nextBeatMs != 0 && now >= g_nextBeatMs) {
      playHeartbeatLively();
      unsigned long jitter = HEARTBEAT_MAX_MS - HEARTBEAT_MIN_MS;
      g_nextBeatMs = now + HEARTBEAT_MIN_MS + (unsigned long)random((long)(jitter + 1));
    }
    // Drop connection if robot has been silent for too long
    if ((now - g_lastRobotRxMs) > SILENCE_DROP_MS) {
      g_connected = false;
      g_nextBeatMs = 0;
    }
  }
}

void setup() {
  Serial.begin(HOST_BAUD);
  pinMode(POWER_TOGGLE_PIN, OUTPUT);
  digitalWrite(POWER_TOGGLE_PIN, LOW);
  beginRobotUart();
  g_connected = false;
  g_lastTryMs = g_lastProbeMs = g_lastRobotRxMs = millis();
  g_nextBeatMs = 0;
  g_hostMode = false;
  g_ctrlLen = 0;
  // Seed weak PRNG with timer noise
  randomSeed((unsigned long)micros());
}

void loop() {
  // Host → Robot (with early HELLO interception before hostMode)
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    uint8_t b = (uint8_t)c;
    if (!g_hostMode) {
      // Accumulate a short ASCII control line
      if (b == '\n' || b == '\r') {
        g_ctrlBuf[g_ctrlLen < sizeof(g_ctrlBuf)-1 ? g_ctrlLen : sizeof(g_ctrlBuf)-1] = '\0';
        if (strcmp(g_ctrlBuf, "HELLO") == 0) {
          // Host is taking control. Perform a deterministic OFF→ON power cycle,
          // wait for boot, then minimally init OI before replying READY.
          Serial.println("BUSY");
          // Always force OFF first, then ON to ensure a clean reboot
          pulsePowerToggle();                 // OFF
          delay(POWER_OFF_SETTLE_MS);
          pulsePowerToggle();                 // ON
          delay(POWER_POST_DELAY_MS);
          // Ensure UART is configured at our current candidate baud
          beginRobotUart();
          // Minimal OI init: Start + Safe
          oiWrite(OI_START);
          oiWrite(OI_SAFE);
          // Optional: quick probe to confirm OI up; if not, keep going anyway
          (void)probeOiAndMaybeRead();
          // Hand over to host
          g_connected = true;
          g_lastRobotRxMs = millis();
          g_nextBeatMs = 0;
          g_hostMode = true;
          Serial.println("READY");
        }
        g_ctrlLen = 0;
      } else if (g_ctrlLen + 1 < sizeof(g_ctrlBuf)) {
        g_ctrlBuf[g_ctrlLen++] = (char)b;
      } else {
        // overflow; reset
        g_ctrlLen = 0;
      }
      // Do not forward bytes before host handshake to avoid confusing the robot
      continue;
    }
    // In hostMode: pure passthrough
    CREATE_SERIAL.write(b);
  }

  // Robot → Host
  while (CREATE_SERIAL.available() > 0) {
    int c = CREATE_SERIAL.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
    g_connected = true;
    g_lastRobotRxMs = millis();
  }

  // Connection management only until host takes over
  if (!g_hostMode) {
    tryConnectTick();
  }
}
