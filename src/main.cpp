#include <Arduino.h>
#include <stdlib.h>
#include "leds.h"

#ifdef BRAINSTEM_UART
#include "proto.h"
#include "behavior.h"
#include "sensors.h"
#include "motion.h"
#include "utils.h"

// ---- UART-centric brainstem v1.0 ----

// Control loop timing (Hz)
static const uint16_t CONTROL_HZ = 50;
static const uint16_t CONTROL_DT_MS = 1000 / CONTROL_HZ;

// Runtime parameters (defaults)
static float param_slew_v = 0.50f;   // m/s^2
static float param_slew_w = 4.0f;    // rad/s^2
static uint16_t param_watchdog_ms = 400;
static uint16_t param_odom_hz = 20;
static float param_soft_stop_m = 0.22f;
static float param_hard_stop_m = 0.10f;
static uint32_t param_tx_bytes_per_s = 12000;
static uint16_t param_max_line_len = 96;
static uint8_t param_log_level = 0;  // 0=off..3=debug

// UART line reader (sanitized)
static char lineBuf[80];
static uint16_t lineLen = 0;
static unsigned long last_uart_ms = 0;
static bool linkUp = false;
// Forebrain vs Autonomous mode switching
static bool modeForebrain = false;      // true=FOREBRAIN, false=AUTONOMOUS
static bool autonomousInit = false;     // one-time init guard
static const char* curModeState = nullptr; // edge-dedup for STATE mode telemetry

// TX scheduler (token bucket)
static float tx_tokens = 0.0f;
static unsigned long tx_last_ms = 0;
static bool tx_paused = false;
static uint32_t stat_tx_drop = 0;
static uint32_t stat_rx_overflow = 0;
static uint32_t stat_crc_err = 0;

// Replay ring (compact). Keep small on AVR.
#if defined(ARDUINO_ARCH_AVR)
#  define REPLAY_N 2
#  define REPLAY_MAXLEN 48
#else
#  define REPLAY_N 64
#  define REPLAY_MAXLEN 128
#endif
struct ReplayEntry { uint32_t eid; uint16_t len; char data[REPLAY_MAXLEN]; };
static ReplayEntry replay[REPLAY_N];
static uint16_t replay_head = 0; // next write index
static uint32_t eid_latest = 0;

// Core motion state
static unsigned long lastTickMs = 0;
static unsigned long lastTwistMs = 0;
static uint32_t lastTwistSeq = 0;
static bool safetyEnabled = true;
static bool estopActive = false;
static bool stale = true;
static bool staleAnnounced = false;

// Startle/reflex windows
static unsigned long reflexUntilMs = 0;
static unsigned long hesitateUntilMs = 0;

// LED mask (from LED,<mask>)
static uint32_t ledMask = 0;

// Commanded (target) and actual velocities
static float vx_target = 0.0f, wz_target = 0.0f;
static float vx_actual = 0.0f, wz_actual = 0.0f;

// Differential drive kinematics (approx track width in meters)
static float param_track_m = 0.26f; // Create 1 ~0.26 m wheel separation

static inline void oi_drive_direct(int16_t right_mm_s, int16_t left_mm_s) {
  const uint8_t OI_DRIVE_DIRECT = 145;
  CREATE_SERIAL.write(OI_DRIVE_DIRECT);
  CREATE_SERIAL.write((uint8_t)((right_mm_s >> 8) & 0xFF));
  CREATE_SERIAL.write((uint8_t)(right_mm_s & 0xFF));
  CREATE_SERIAL.write((uint8_t)((left_mm_s >> 8) & 0xFF));
  CREATE_SERIAL.write((uint8_t)(left_mm_s & 0xFF));
}

static inline int16_t clamp_mm_s(float v_mm_s) {
  // OI limits velocity to about +/-500 mm/s
  if (v_mm_s > 500.0f) v_mm_s = 500.0f;
  if (v_mm_s < -500.0f) v_mm_s = -500.0f;
  // Round to nearest integer
  return (int16_t)(v_mm_s >= 0 ? (v_mm_s + 0.5f) : (v_mm_s - 0.5f));
}

static void applyDriveFromTwist(float vx_mps, float wz_rad_s) {
  // Map twist to wheel linear speeds: v_r = vx + wz * (track/2), v_l = vx - wz * (track/2)
  float half = 0.5f * param_track_m;
  float v_r_mps = vx_mps + wz_rad_s * half;
  float v_l_mps = vx_mps - wz_rad_s * half;
  int16_t r = clamp_mm_s(v_r_mps * 1000.0f);
  int16_t l = clamp_mm_s(v_l_mps * 1000.0f);
  oi_drive_direct(r, l);
  feedRobotWatchdog();
}

// Odometry
static float odom_x = 0.0f, odom_y = 0.0f, odom_th = 0.0f;
static unsigned long lastOdomMs = 0;
static unsigned long lastTimeMs = 0;

// Range guard (host-fed)
static const uint8_t MAX_RANGE_IDS = 2;
static float rangeVals[MAX_RANGE_IDS];
static int32_t rangeIds[MAX_RANGE_IDS];
static bool rangeValid[MAX_RANGE_IDS];
static float lastMinRange = NAN;
static int32_t lastMinRangeId = -1;

// STATE tracking
static const char* curState = PROTO_STATE_LINKDOWN;

// Create OI (Serial1) helpers for sensors/odometry
#ifndef CREATE_SERIAL
#define CREATE_SERIAL Serial1
#endif
static constexpr uint8_t OI_START = 128;
static constexpr uint8_t OI_SAFE = 131;
static constexpr uint8_t OI_FULL = 132;
static constexpr uint8_t OI_SENSORS = 142;

static bool oi_read_u8(uint8_t packetId, uint8_t &out) {
  CREATE_SERIAL.write(OI_SENSORS);
  CREATE_SERIAL.write(packetId);
  unsigned long t0 = millis();
  while (millis() - t0 < 5) {
    if (CREATE_SERIAL.available() > 0) { out = (uint8_t)CREATE_SERIAL.read(); return true; }
  }
  return false;
}
static bool oi_read_i16(uint8_t packetId, int16_t &out) {
  CREATE_SERIAL.write(OI_SENSORS);
  CREATE_SERIAL.write(packetId);
  unsigned long t0 = millis();
  int got = 0; int16_t v = 0;
  while (millis() - t0 < 6) {
    while (CREATE_SERIAL.available() && got < 2) {
      uint8_t b = (uint8_t)CREATE_SERIAL.read();
      v = (int16_t)((v << 8) | b);
      got++;
    }
    if (got >= 2) { out = v; return true; }
  }
  return false;
}

// OI sensor cache and publishing helpers
static uint8_t prevBumpMask = 0;
static uint8_t prevCliffMask = 0;
static bool wheelDropHazard = false;
static unsigned long lastBattMs = 0;
static uint16_t batt_mV = 0; static uint8_t batt_pct = 0; static uint8_t batt_chg = 0;

// Helpers
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float stepToward(float cur, float goal, float maxStep) {
  if (cur < goal - maxStep) return cur + maxStep;
  if (cur > goal + maxStep) return cur - maxStep;
  return goal;
}

static void replay_add(const char* line, uint16_t len, uint32_t eid) {
  ReplayEntry &e = replay[replay_head];
  e.eid = eid;
  if (len >= REPLAY_MAXLEN) len = REPLAY_MAXLEN - 1;
  memcpy(e.data, line, len);
  e.data[len] = '\0';
  e.len = len;
  replay_head = (replay_head + 1) % REPLAY_N;
}

// Send a line with priority (0=P0,1=P1,2=P2), append ,eid= and \n, log to replay.
static void tx_send(uint8_t pri, const char* base) {
  // Pause gate for P1/P2
  if (tx_paused && pri > 0) return;
  char out[REPLAY_MAXLEN + 24];
  uint16_t baseLen = (uint16_t)strnlen(base, REPLAY_MAXLEN);
  // Token bucket for P1/P2
  unsigned long now = millis();
  if (tx_last_ms == 0) tx_last_ms = now;
  float add = (float)(now - tx_last_ms) * ((float)param_tx_bytes_per_s / 1000.0f);
  tx_tokens = clampf(tx_tokens + add, 0.0f, (float)param_tx_bytes_per_s);
  tx_last_ms = now;
  uint16_t willBytes = baseLen + 16; // approx including eid
  if (pri > 0 && tx_tokens < willBytes) {
    stat_tx_drop++;
    return;
  }
  if (pri > 0) tx_tokens -= willBytes;

  // Compose with eid
  uint32_t eid = ++eid_latest;
  int n = snprintf(out, sizeof(out), "%s,eid=%lu\n", base, (unsigned long)eid);
  if (n < 0) return;
  uint16_t outLen = (uint16_t)n;
  Serial.write((const uint8_t*)out, outLen);
  replay_add(out, outLen, eid);
}

static void log_msg(uint8_t lvl, const char* msg) {
  if (param_log_level >= lvl) {
    char buf[96];
    snprintf(buf, sizeof(buf), "LOG,%c,%s", lvl==3?'D':lvl==2?'I':lvl==1?'W':'E', msg);
    tx_send(2, buf);
  }
}

static void publish_state(const char* s) {
  if (curState != s) {
    curState = s;
    char buf[48];
    snprintf(buf, sizeof(buf), "STATE,%s", s);
    tx_send(0, buf);
  }
}

// Dedicated mode STATE to avoid interference with substate publishing
static void publish_mode_state(const char* s) {
  if (curModeState != s) {
    curModeState = s;
    char buf[32];
    snprintf(buf, sizeof(buf), "STATE,%s", s);
    tx_send(0, buf);
  }
}

static void publish_link(uint8_t up) {
  char buf[32];
  snprintf(buf, sizeof(buf), "LINK,%u,%lu", (unsigned)up, (unsigned long)lastTwistSeq);
  tx_send(0, buf);
}

static void publish_hello() {
  char buf[64];
  snprintf(buf, sizeof(buf), "HELLO,proto=1.0,build=%s %s", __DATE__, __TIME__);
  tx_send(1, buf);
}

static void publish_health_boot() {
  char buf[32];
  snprintf(buf, sizeof(buf), "HEALTH,1,0,%lu", (unsigned long)lastTwistSeq);
  tx_send(1, buf);
}

static void publish_estop(bool active) {
  char buf[24];
  snprintf(buf, sizeof(buf), "ESTOP,%u,%lu", active?1:0, (unsigned long)lastTwistSeq);
  tx_send(0, buf);
}

static void publish_bump(uint8_t mask) { char b[32]; snprintf(b, sizeof(b), "BUMP,1,%u,%lu", (unsigned)mask, (unsigned long)lastTwistSeq); tx_send(0,b);} 
static void publish_cliff(uint8_t mask) { char b[32]; snprintf(b, sizeof(b), "CLIFF,1,%u,%lu", (unsigned)mask, (unsigned long)lastTwistSeq); tx_send(0,b);} 
static void publish_startle(const char* reason, uint8_t mask) { char b[48]; snprintf(b, sizeof(b), "STARTLE,%s,%u,%lu", reason, (unsigned)mask, (unsigned long)lastTwistSeq); tx_send(0,b);} 
static void publish_stale(unsigned long ms_since) { char b[48]; snprintf(b, sizeof(b), "STALE,twist,%lu", ms_since); tx_send(0,b);} 
static void publish_rgmin(float m, int32_t id) { char b[48]; snprintf(b, sizeof(b), "RGMIN,%.3f,%ld,%lu", (double)m, (long)id, (unsigned long)lastTwistSeq); tx_send(1,b);} 
static void publish_pong(uint32_t seq) { char b[24]; snprintf(b, sizeof(b), "PONG,%lu", (unsigned long)seq); tx_send(0,b);} 
static void publish_time() { char b[24]; snprintf(b, sizeof(b), "TIME,%lu", (unsigned long)millis()); tx_send(1,b);} 
static void publish_odom() { char b[96]; snprintf(b, sizeof(b), "ODOM,%.3f,%.3f,%.3f,%.3f,%.3f,%lu", (double)odom_x,(double)odom_y,(double)odom_th,(double)vx_actual,(double)wz_actual,(unsigned long)lastTwistSeq); tx_send(1,b);} 
static void publish_bat() { char b[48]; snprintf(b,sizeof(b),"BAT,%u,%u,%u", (unsigned)batt_mV, (unsigned)batt_pct, (unsigned)batt_chg); tx_send(1,b); }

static void setLedModeFromState() {
  if (estopActive) setLedPattern(PATTERN_ALERT);
  else if (millis() < reflexUntilMs) setLedPattern(PATTERN_RECOILING);
  else if (!linkUp) setLedPattern(PATTERN_SEEKING);
  else if (stale) setLedPattern(PATTERN_SEEKING);
  else if (fabs(vx_actual) > 1e-3f || fabs(wz_actual) > 1e-3f) setLedPattern(PATTERN_ADVANCING);
  else setLedPattern(PATTERN_WAITING);
}

static void recoil() {
  reflexUntilMs = millis() + 250;
  hesitateUntilMs = reflexUntilMs + 250;
  vx_target = wz_target = 0;
}

// Strict number parse helper
static bool parse_float(const char* s, float &out) { char* e=nullptr; double v = strtod(s,&e); out = (float)v; return e && *e=='\0'; }
static bool parse_uint(const char* s, uint32_t &out) { char* e=nullptr; unsigned long v=strtoul(s,&e,10); if(!(e&&*e=='\0')) return false; out=(uint32_t)v; return true; }
static bool parse_int(const char* s, int32_t &out) { char* e=nullptr; long v=strtol(s,&e,10); if(!(e&&*e=='\0')) return false; out=(int32_t)v; return true; }

// CRC (optional): XOR of bytes before '*', two hex digits after
static bool check_crc_if_present(char* line) {
  size_t L = strlen(line);
  if (L>=3) {
    char* star = strrchr(line, '*');
    if (star && (line + L - star) == 3) {
      uint8_t want=0;
      for (int i=1;i<=2;i++){ char c=star[i]; want<<=4; if (c>='0'&&c<='9') want|=(c-'0'); else if(c>='A'&&c<='F') want|=(c-'A'+10); else if(c>='a'&&c<='f') want|=(c-'a'+10); else return false; }
      uint8_t crc=0; for(char* p=line; p<star; ++p) crc ^= (uint8_t)(*p);
      if (crc != want) { stat_crc_err++; return false; }
      *star='\0'; // strip CRC
    }
  }
  return true;
}

// Handlers
static void handleTwist(char* args) {
  char* a1=strtok(args, ","); char* a2=strtok(nullptr, ","); char* a3=strtok(nullptr, ",");
  if(!(a1&&a2&&a3)){ tx_send(0,"ERR,parse,arity"); return; }
  float vx,wz; uint32_t seq;
  if(!parse_float(a1,vx)||!parse_float(a2,wz)||!parse_uint(a3,seq)){ tx_send(0,"ERR,parse,num"); return; }
  lastTwistSeq = seq; lastTwistMs = millis(); stale=false; staleAnnounced=false;
  vx_target = vx; wz_target = wz; if (millis() < hesitateUntilMs) hesitateUntilMs=0;
  publish_state(PROTO_STATE_TELEOP);
  // Debug ACK to trace inbound commands
  {
    char b[64];
    snprintf(b, sizeof(b), "ACK,TWIST,%.3f,%.3f,%lu", (double)vx, (double)wz, (unsigned long)seq);
    tx_send(0, b);
  }
}
static void handleSafe(char* args) {
  if(!args||!*args){ tx_send(0,"ERR,parse,arity"); return; }
  uint32_t v; if(!parse_uint(args,v)) { tx_send(0,"ERR,parse,num"); return; }
  safetyEnabled = (v!=0); estopActive = !safetyEnabled; publish_estop(estopActive);
  publish_state(estopActive?PROTO_STATE_ESTOP:PROTO_STATE_IDLE);
  if (estopActive) {
    setLedPattern(PATTERN_ALERT);
    playEstopAlarmSad();
  }
  { char b[24]; snprintf(b, sizeof(b), "ACK,SAFE,%u", (unsigned)(v!=0)); tx_send(0, b); }
}
static void handleLed(char* args) { uint32_t m=0; if(args) m=strtoul(args,nullptr,10); ledMask=m; }
static void handlePing(char* args){ if(!args){ tx_send(0,"ERR,parse,arity"); return;} uint32_t s; if(!parse_uint(args,s)){ tx_send(0,"ERR,parse,num"); return;} publish_pong(s); }
static void handlePause(){ tx_paused=true; tx_send(0,"ACK,paused,1"); log_msg(2,"paused"); }
static void handleResume(){ tx_paused=false; tx_send(0,"ACK,paused,0"); log_msg(2,"resumed"); }
static void handleRange(char* args){
  char* a1=strtok(args, ","); char* a2=strtok(nullptr, ",");
  if(!(a1&&a2)){ tx_send(0,"ERR,parse,arity"); return; }
  float m; int32_t id; if(!parse_float(a1,m)||!parse_int(a2,id)){ tx_send(0,"ERR,parse,num"); return; }
  int slot=-1; for(uint8_t i=0;i<MAX_RANGE_IDS;i++){ if(rangeValid[i] && rangeIds[i]==id){ slot=i; break; } if(slot==-1 && !rangeValid[i]) slot=i; }
  if(slot>=0){ rangeIds[slot]=id; rangeVals[slot]=m; rangeValid[slot]=true; }
  float minv=NAN; int32_t minId=-1; for(uint8_t i=0;i<MAX_RANGE_IDS;i++){ if(rangeValid[i]){ if(isnan(minv)||rangeVals[i]<minv){ minv=rangeVals[i]; minId=rangeIds[i]; } } }
  if( (isnan(lastMinRange) && !isnan(minv)) || (!isnan(minv) && fabs(minv - lastMinRange) > 1e-3f) || (minId!=lastMinRangeId) ){
    publish_rgmin(minv, minId);
    lastMinRange=minv; lastMinRangeId=minId;
  }
}
static void handleSet(char* args){
  char* k=strtok(args, ","); char* v=strtok(nullptr, ","); if(!(k&&v)){ tx_send(0,"ERR,parse,arity"); return; }
  float f; uint32_t u; int ok=1; char ack[48];
  if(strcmp(k,PROTO_K_SOFT_STOP)==0){ if(!parse_float(v,f)) ok=0; else param_soft_stop_m=f; }
  else if(strcmp(k,PROTO_K_HARD_STOP)==0){ if(!parse_float(v,f)) ok=0; else param_hard_stop_m=f; }
  else if(strcmp(k,PROTO_K_WATCHDOG)==0){ if(!parse_uint(v,u)) ok=0; else param_watchdog_ms=(uint16_t)u; }
  else if(strcmp(k,PROTO_K_ODOM_HZ)==0){ if(!parse_uint(v,u)) ok=0; else param_odom_hz=(uint16_t)u; }
  else if(strcmp(k,PROTO_K_SLEW_V)==0){ if(!parse_float(v,f)) ok=0; else param_slew_v=f; }
  else if(strcmp(k,PROTO_K_SLEW_W)==0){ if(!parse_float(v,f)) ok=0; else param_slew_w=f; }
  else if(strcmp(k,PROTO_K_TX_BUDGET)==0){ if(!parse_uint(v,u)) ok=0; else param_tx_bytes_per_s=u; }
  else if(strcmp(k,PROTO_K_MAX_LINE)==0){ if(!parse_uint(v,u)) ok=0; else param_max_line_len=(uint16_t)u; }
  else if(strcmp(k,PROTO_K_LOG_LEVEL)==0){ if(!parse_uint(v,u)) ok=0; else param_log_level=(uint8_t)u; }
  else { char eb[64]; snprintf(eb,sizeof(eb),"ERR,param,%s",k); tx_send(0, eb); return; }
  if(!ok){ tx_send(0,"ERR,parse,num"); return; }
  snprintf(ack, sizeof(ack), "ACK,%s,%s", k, v); tx_send(0, ack);
}
static void handleGet(char* args){
  char* k=strtok(args, ","); if(!k){ tx_send(0,"ERR,parse,arity"); return; }
  if(strcmp(k,"evt")==0){ char* v=strtok(nullptr, ","); if(!v){ tx_send(0,"ERR,parse,arity"); return; } uint32_t want; if(!parse_uint(v,want)){ tx_send(0,"ERR,parse,num"); return; }
    for(uint16_t i=0;i<REPLAY_N;i++){ ReplayEntry &e=replay[i]; if(e.len>0 && e.eid==want){ Serial.write((const uint8_t*)e.data, e.len); return; } }
    tx_send(0,"ERR,evt,missing"); return; }
  char buf[48];
  if(strcmp(k,PROTO_K_SOFT_STOP)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%.3f",k,(double)param_soft_stop_m); }
  else if(strcmp(k,PROTO_K_HARD_STOP)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%.3f",k,(double)param_hard_stop_m); }
  else if(strcmp(k,PROTO_K_WATCHDOG)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%u",k,(unsigned)param_watchdog_ms); }
  else if(strcmp(k,PROTO_K_ODOM_HZ)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%u",k,(unsigned)param_odom_hz); }
  else if(strcmp(k,PROTO_K_SLEW_V)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%.3f",k,(double)param_slew_v); }
  else if(strcmp(k,PROTO_K_SLEW_W)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%.3f",k,(double)param_slew_w); }
  else if(strcmp(k,PROTO_K_TX_BUDGET)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%lu",k,(unsigned long)param_tx_bytes_per_s); }
  else if(strcmp(k,PROTO_K_MAX_LINE)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%u",k,(unsigned)param_max_line_len); }
  else if(strcmp(k,PROTO_K_LOG_LEVEL)==0){ snprintf(buf,sizeof(buf),"ACK,%s,%u",k,(unsigned)param_log_level); }
  else { char eb[64]; snprintf(eb,sizeof(eb),"ERR,param,%s",k); tx_send(0, eb); return; }
  tx_send(0, buf);
}
static void handleReplay(char* args){ uint32_t since; if(!args||!parse_uint(args,since)){ tx_send(0,"ERR,parse,num"); return;} uint16_t idx = replay_head; for(uint16_t n=0;n<REPLAY_N;n++){ ReplayEntry &e = replay[idx]; if(e.len>0 && e.eid>since){ Serial.write((const uint8_t*)e.data, e.len); } idx=(idx+1)%REPLAY_N; } }
static void handleStats(){ char b[96]; snprintf(b,sizeof(b),"ACK,stats,tx_drop=%lu,rx_overflow=%lu,crc_err=%lu,eid_latest=%lu", (unsigned long)stat_tx_drop,(unsigned long)stat_rx_overflow,(unsigned long)stat_crc_err,(unsigned long)eid_latest); tx_send(0,b);} 

static void handleLine(char* line) {
  if(!check_crc_if_present(line)) { tx_send(0,"ERR,crc"); return; }
  char* p = strchr(line, ',');
  const char* cmd = line; char* args = nullptr; if (p){ *p='\0'; args=p+1; }
  if (strcmp(cmd, "TWIST") == 0) handleTwist(args);
  else if (strcmp(cmd, "SAFE") == 0) handleSafe(args);
  else if (strcmp(cmd, "LED") == 0) handleLed(args);
  else if (strcmp(cmd, "PING") == 0) handlePing(args);
  else if (strcmp(cmd, "PAUSE") == 0) handlePause();
  else if (strcmp(cmd, "RESUME") == 0) handleResume();
  else if (strcmp(cmd, "RANGE") == 0) handleRange(args);
  else if (strcmp(cmd, "SET") == 0) handleSet(args);
  else if (strcmp(cmd, "GET") == 0) handleGet(args);
  else if (strcmp(cmd, "REPLAY") == 0) handleReplay(args);
  else if (strcmp(cmd, "STATS") == 0) handleStats();
  else { char b[48]; snprintf(b,sizeof(b),"ERR,cmd,%s", cmd); tx_send(0,b); }
}

static void pollUart(bool allowHandle) {
  while (Serial.available()) {
    int c = Serial.read(); if (c < 0) break; char ch = (char)c; unsigned long now=millis(); last_uart_ms = now;
    if (!linkUp) { linkUp=true; publish_link(1); }
    if (ch=='\0') continue; // strip NULs
    if (!(ch=='\r'||ch=='\n' || (ch>=32 && ch<=126))) { tx_send(0,"ERR,parse,char"); continue; }
    if (ch=='\r' || ch=='\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        if (allowHandle) {
          handleLine(lineBuf);
        } else {
          // Limited command handling even in AUTONOMOUS mode: respond to benign queries
          if (strncmp(lineBuf, "PING", 4) == 0 || strncmp(lineBuf, "GET", 3) == 0 || strncmp(lineBuf, "STATS", 5) == 0) {
            handleLine(lineBuf);
          }
        }
        lineLen=0;
      }
    } else {
      uint16_t cap = param_max_line_len; if (cap >= sizeof(lineBuf)) cap = sizeof(lineBuf)-1;
      if (lineLen + 1 < cap) { lineBuf[lineLen++] = ch; }
      else { lineLen=0; stat_rx_overflow++; tx_send(0,"ERR,parse,overflow"); }
    }
  }
}

static void pollCreateSensors() {
  // Bump + wheel drop (packet 7)
  uint8_t b7=0; if (oi_read_u8(7, b7)) {
    uint8_t bumpMask = 0;
    if (b7 & 0x02) bumpMask |= PROTO_MASK_LEFT;
    if (b7 & 0x01) bumpMask |= PROTO_MASK_RIGHT;
    bool wheel = (b7 & 0x0C) != 0; // either wheel drop
    if (bumpMask && (bumpMask & ~prevBumpMask)) { publish_bump(bumpMask & ~prevBumpMask); publish_startle("bump", bumpMask & ~prevBumpMask); recoil(); }
    prevBumpMask = bumpMask;
    if (wheel && !wheelDropHazard) { wheelDropHazard = true; estopActive = true; publish_estop(true); publish_state(PROTO_STATE_ESTOP); }
    if (!wheel && wheelDropHazard) { wheelDropHazard = false; }
  }
  // Cliff sensors (9..12), treat left/right mask
  uint8_t c9=0,c10=0,c11=0,c12=0; bool ok9=oi_read_u8(9,c9), ok10=oi_read_u8(10,c10), ok11=oi_read_u8(11,c11), ok12=oi_read_u8(12,c12);
  if (ok9||ok10||ok11||ok12) {
    uint8_t cliffMask=0; if (c9||c10) cliffMask |= PROTO_MASK_LEFT; if (c11||c12) cliffMask |= PROTO_MASK_RIGHT;
    uint8_t rise = cliffMask & ~prevCliffMask; if (rise) { publish_cliff(rise); publish_startle("cliff", rise); recoil(); playCliffWhoa(); }
    prevCliffMask = cliffMask;
  }
  // Odom from distance (19 mm) and angle (20 deg)
  int16_t dmm=0, deg=0; bool okd = oi_read_i16(19,dmm); bool oka = oi_read_i16(20,deg);
  if (okd || oka) {
    float d = (float)dmm / 1000.0f; float dth = (float)deg * (PI/180.0f);
    float th_mid = odom_th + 0.5f * dth;
    odom_x += d * cosf(th_mid);
    odom_y += d * sinf(th_mid);
    odom_th += dth;
  }
  // Battery/charging (1 Hz)
  if (millis() - lastBattMs >= 1000) {
    lastBattMs = millis();
    uint8_t chg=0; int16_t mv=0; int16_t charge=0; int16_t cap=0;
    if (oi_read_u8(21, chg)) batt_chg = chg;
    if (oi_read_i16(22, mv)) batt_mV = (uint16_t)mv;
    if (oi_read_i16(25, charge) && oi_read_i16(26, cap) && cap>0) {
      int pct = (int)((long)charge * 100L / (long)cap);
      if (pct < 0) pct = 0; if (pct > 100) pct = 100; batt_pct = (uint8_t)pct;
    }
    publish_bat();
    static bool lowBatNotified = false;
    if (batt_pct <= 15 && !lowBatNotified) { playLowBatteryTone(); lowBatNotified = true; }
    if (batt_pct >= 20 && lowBatNotified) { lowBatNotified = false; }
  }

  // Buttons (packet 18): Play -> ESTOP, Advance -> toggle wall-follow side
  uint8_t btn=0; static uint8_t prevBtn=0; if (oi_read_u8(18, btn)) {
    uint8_t rise = btn & ~prevBtn;
    prevBtn = btn;
    if (rise & 0x01) {
      // Play button: engage ESTOP immediately
      estopActive = true; safetyEnabled = false; publish_estop(true); publish_state(PROTO_STATE_ESTOP);
      setLedPattern(PATTERN_ALERT); stopAllMotors();
    }
    if (rise & 0x04) {
      // Advance (fast forward): toggle wall-follow side sensibly
      toggleWallFollowSide();
      log_msg(2, "button:advance toggle wall-follow side");
    }
  }
}

static void computeAndPublishRates(unsigned long now) {
  // ODOM rate
  uint16_t odom_dt = (param_odom_hz>0)? (1000/param_odom_hz) : 50;
  if (!tx_paused && now - lastOdomMs >= odom_dt) { lastOdomMs = now; publish_odom(); }
  // TIME 1 Hz
  if (!tx_paused && now - lastTimeMs >= 1000) { lastTimeMs = now; publish_time(); }
}

static void controlTick() {
  unsigned long now = millis(); float dt = (CONTROL_DT_MS) / 1000.0f;
  // Watchdog stale edge
  if (now - lastTwistMs >= param_watchdog_ms) {
    if (!stale) { stale = true; staleAnnounced=false; }
    if (!staleAnnounced) { publish_stale(now - lastTwistMs); staleAnnounced = true; }
  }
  // Linkdown edge handled by mode manager

  // Arbitration: ESTOP > REFLEX > RANGE guards > hesitate > slew
  float vx_goal = vx_target; float wz_goal = wz_target;
  if (estopActive || !safetyEnabled) { vx_goal=0; wz_goal=0; }
  else if (now < reflexUntilMs) { vx_goal=0; wz_goal=0; }
  else if (now < hesitateUntilMs) { vx_goal=0; wz_goal=0; }
  else {
    float minv = lastMinRange; if (!isnan(minv)) {
      if (minv < param_hard_stop_m && vx_goal > 0) { publish_startle("range_min", 0); recoil(); vx_goal=0; wz_goal=0; }
      else if (minv < param_soft_stop_m && vx_goal > 0) {
        float scale = clampf((minv - param_hard_stop_m) / (param_soft_stop_m - param_hard_stop_m), 0.0f, 1.0f);
        vx_goal *= scale;
      }
    }
  }

  // Slew toward goals (for vx_actual/wz_actual telemetry only)
  float maxDv = param_slew_v * dt; float maxDw = param_slew_w * dt;
  vx_actual = stepToward(vx_actual, vx_goal, maxDv);
  wz_actual = stepToward(wz_actual, wz_goal, maxDw);

  // Send drive command to Create OI
  oiFullGuardTick();
  applyDriveFromTwist(vx_actual, wz_actual);

  // STATE select
  if (!linkUp) publish_state(PROTO_STATE_LINKDOWN);
  else if (estopActive) publish_state(PROTO_STATE_ESTOP);
  else if (now < reflexUntilMs) publish_state(PROTO_STATE_REFLEX);
  else if (stale) publish_state(PROTO_STATE_STALE);
  else if (fabs(vx_actual) > 1e-3f || fabs(wz_actual) > 1e-3f) publish_state(PROTO_STATE_TELEOP);
  else publish_state(PROTO_STATE_IDLE);

  setLedModeFromState();
  computeAndPublishRates(now);
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 200) { /* wait up to 200ms */ }
  // Init Create OI for sensor reads only
  CREATE_SERIAL.begin(57600);
  delay(50);
  CREATE_SERIAL.write(OI_START);
  // Create 1: operate in FULL for consistent teleop control and stream access
  CREATE_SERIAL.write(OI_FULL);

  for (uint8_t i=0;i<MAX_RANGE_IDS;i++){ rangeValid[i]=false; rangeIds[i]=0; rangeVals[i]=NAN; }
  last_uart_ms = millis(); linkUp=false; modeForebrain = false; autonomousInit = false; curModeState = nullptr;
  tx_last_ms = millis(); tx_tokens = (float)param_tx_bytes_per_s; // allow initial burst
  initLeds();
  publish_hello();
  publish_health_boot();
  playStartupJingle();
  // Default to AUTONOMOUS mode until a forebrain appears
  initializeBehavior();
  autonomousInit = true;
  publish_mode_state(PROTO_STATE_AUTONOMOUS);
}

void loop() {
  // 1) Always poll UART first (count activity even if not handling commands)
  pollUart(modeForebrain /*allowHandle*/);

  // 2) Mode manager: switch between FOREBRAIN and AUTONOMOUS based on UART activity
  unsigned long now = millis();
  bool connectedNow = (now - last_uart_ms < 2000);
  if (connectedNow && !modeForebrain) {
    // Transition AUTONOMOUS -> FOREBRAIN
    modeForebrain = true;
    // Stop any autonomous motion and pause sensor stream
    stopAllMotors();
    pauseSensorStream();
    // Clear any stale TWIST targets
    vx_target = 0.0f; wz_target = 0.0f; stale = true; staleAnnounced = false;
    // Announce link up if needed and mode switch
    if (!linkUp) { linkUp = true; publish_link(1); }
    publish_mode_state(PROTO_STATE_FOREBRAIN);
    playForebrainTrill();
  } else if (!connectedNow && modeForebrain) {
    // Transition FOREBRAIN -> AUTONOMOUS (seamless fallback)
    modeForebrain = false;
    // Safe stop on handover
    vx_target = 0.0f; wz_target = 0.0f; stopAllMotors();
    if (linkUp) { linkUp = false; publish_link(0); }
    publish_mode_state(PROTO_STATE_AUTONOMOUS);
    playLonelyTune();
    // Ensure autonomous stack initialized
    if (!autonomousInit) { initializeBehavior(); autonomousInit = true; }
  }

  if (modeForebrain) {
    // 3) Forebrain-connected: poll Create sensors and run UART control loop
    pollCreateSensors();
    if (now - lastTickMs >= CONTROL_DT_MS) {
      lastTickMs = now;
      controlTick();
    }
    // Idle chirp in forebrain mode if robot is idle for a long while
    static unsigned long lastIdleChirpMs = 0;
    if (curState == PROTO_STATE_IDLE && linkUp) {
      if (now - lastIdleChirpMs > 30000UL) { playIdleChirp(); lastIdleChirpMs = now; }
    } else {
      if (now - lastIdleChirpMs > 60000UL) lastIdleChirpMs = now; // avoid wrap
    }
  } else {
    // 3) Autonomous: stream sensors + behavior FSM + safety watchdog
    updateSensorStream();
    // Local estop via Play/Advance buttons
    static bool autoEstop = false;
    if (playButtonPressedAndClear()) { autoEstop = true; }
    if (advanceButtonPressedAndClear()) { autoEstop = false; }
    if (autoEstop || bumperEventTriggeredAndClear() || bumperTriggered() || cliffDetected()) {
      stopAllMotors();
      setLedPattern(PATTERN_ALERT);
    }
    if (!autoEstop) {
      updateBehavior();
    }
    enforceRobotWatchdog();
  }

  // 4) Drive LED patterns non-blocking
  updateLeds();
  // Always enforce motion watchdog in both modes
  enforceRobotWatchdog();
}

#else // BRAINSTEM_UART

#include "behavior.h"
#include "sensors.h"
#include "motion.h"
#include "utils.h"

void setup() {
  // Bring up USB CDC for debugging
  Serial.begin(115200);
  delay(50);
  Serial.println("[BOOT] brainstem-pm starting");
  initializeBehavior();
  initLeds();
}

void loop() {
  // Always poll OI sensor stream to keep caches fresh
  updateSensorStream();
  // Safety: stop immediately on hazards (bumper/cliff) before behavior acts
  if (bumperEventTriggeredAndClear() || bumperTriggered() || cliffDetected()) {
    stopAllMotors();
    setLedPattern(PATTERN_ALERT);
  }
  updateBehavior();    // one tick per loop
  enforceRobotWatchdog(); // ensure stop if control loop stalls
  updateLeds();      // drive LED patterns non-blocking
}

#endif // BRAINSTEM_UART
