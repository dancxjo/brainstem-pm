#include "presence.h"
#include "leds.h"
#include "utils.h"
#include "motion.h"
#include <Arduino.h>

// Simple startup presence: brief period of lifelike flashes/sounds/micro-motions
static bool g_active = false;
static unsigned long g_untilMs = 0;
static unsigned long g_nextActMs = 0;
static unsigned long g_ledOverlayUntil = 0;
static bool g_playedBootTone = false;
static int g_overlayPattern = PATTERN_IDLE;

// Configuration
static const unsigned long PRESENCE_WINDOW_MS = 25000;     // active for first ~25s
static const unsigned long ACT_MIN_INTERVAL_MS = 700;      // min gap between actions
static const unsigned long ACT_MAX_INTERVAL_MS = 1800;     // max gap between actions
static const unsigned long LED_OVERLAY_MS = 600;           // duration of random LED overlay

void initPresence() {
  unsigned long now = millis();
  g_active = true;
  g_untilMs = now + PRESENCE_WINDOW_MS;
  g_nextActMs = now; // act immediately
  g_ledOverlayUntil = 0;
  g_playedBootTone = false;
}

static void scheduleNext(unsigned long now) {
  unsigned long jitter = (unsigned long)random(ACT_MAX_INTERVAL_MS - ACT_MIN_INTERVAL_MS + 1);
  g_nextActMs = now + ACT_MIN_INTERVAL_MS + jitter;
}

bool presenceLedOverlayActive() {
  return g_active && millis() < g_ledOverlayUntil;
}

int presenceOverlayPattern() { return g_overlayPattern; }

void updatePresence(bool inPassthrough, bool sleeping) {
  unsigned long now = millis();
  if (!g_active) return;
  if (now >= g_untilMs) { g_active = false; return; }
  if (sleeping) return; // no activity on low battery sleep
  // During passthrough, avoid any OI traffic but allow LED overlay to show life
  if (inPassthrough) {
    if (now >= g_nextActMs) {
      // Pick a gentle overlay pattern
      g_overlayPattern = (random(3) == 0) ? PATTERN_WAITING : PATTERN_IDLE;
      setLedPattern((LedPattern)g_overlayPattern);
      g_ledOverlayUntil = now + LED_OVERLAY_MS;
      scheduleNext(now);
    }
    return;
  }

  if (now < g_nextActMs) return;

  // Ensure we play one boot tone once we leave passthrough
#ifdef ENABLE_TUNES
  if (!g_playedBootTone) { playStartupJingle(); g_playedBootTone = true; }
#endif

  // Pick a small, eased action randomly (no OI during passthrough per guard above)
  int pick = (int)(random(100));
  if (pick < 45) {
    // Soft micro turn
    if (random(2) == 0) gentleTurnLeft(); else gentleTurnRight();
  } else if (pick < 80) {
#ifdef ENABLE_TUNES
    playIdleChirp();
#endif
  } else {
    // LED overlay: briefly show lifelike flicker or heartbeat
    g_overlayPattern = (random(4) == 0) ? PATTERN_WAITING : PATTERN_IDLE;
    setLedPattern((LedPattern)g_overlayPattern);
    g_ledOverlayUntil = now + LED_OVERLAY_MS;
  }

  scheduleNext(now);
}
