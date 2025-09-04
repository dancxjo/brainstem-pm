// Serial Protocol v1.0 for brainstem UART skeleton
// Inbound (host → MCU):
//   TWIST,<vx_mps>,<wz_radps>,<seq>\n
//   SAFE,<0|1>\n
//   PING,<seq>\n
//   RANGE,<meters>,<id>\n
//   SET,<key>,<value> | GET,<key> | GET,evt,<eid>\n
//   LED,<bitmask>\n
//   <NUL> (0x00) to enter interpreter from default passthrough\n
//   PAUSE | RESUME | PASS (return to passthrough)\n
//   REPLAY,<since_eid> | STATS\n
// Outbound (MCU → host):
//   HELLO,proto=1.0,build=<date> <time>\n
//   LINK,<0|1>,<seq>\n
//   PONG,<seq>\n
//   ODOM,<x>,<y>,<theta>,<vx>,<wz>,<seq>\n         
//   TIME,<millis>\n
//   STATE,<name>\n
//   BUMP,1,<mask>,<seq>\n                          
//   CLIFF,1,<mask>,<seq>\n                         
//   STARTLE,<reason>,<mask>,<seq>\n                
//   ESTOP,<0|1>,<seq>\n
//   STALE,twist,<ms_since>\n
//   RGMIN,<meters>,<id>,<seq>\n
//   ACK,<key>,<value>\n
//   ERR,parse,<reason> | ERR,cmd,<name> | ERR,param,<key> | ERR,crc | ERR,evt,missing\n
//   ... All outbound lines append final suffix: ,eid=<n>\n
// Optional additional telemetry:
//   BAT,<mV>,<percent>,<charging>

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Reason strings for STARTLE
// Use literal strings "bump" or "cliff" in the stream.

// Mask bits for BUMP/CLIFF and STARTLE mask fields
enum {
  PROTO_MASK_LEFT  = 0x01,
  PROTO_MASK_RIGHT = 0x02,
};

// Health codes
enum {
  PROTO_HEALTH_BOOT = 1,
  PROTO_HEALTH_LIMIT_UPDATE = 42, // reserved for future
};

// State names for STATE messages
#define PROTO_STATE_IDLE     "IDLE"
#define PROTO_STATE_TELEOP   "TELEOP"
#define PROTO_STATE_REFLEX   "REFLEX"
#define PROTO_STATE_ESTOP    "ESTOP"
#define PROTO_STATE_STALE    "STALE"
#define PROTO_STATE_LINKDOWN "LINKDOWN"
// Mode names for high-level mode switching
#define PROTO_STATE_FOREBRAIN  "FOREBRAIN"
#define PROTO_STATE_AUTONOMOUS "AUTONOMOUS"

// Parameter keys
#define PROTO_K_SOFT_STOP   "soft_stop_m"
#define PROTO_K_HARD_STOP   "hard_stop_m"
#define PROTO_K_WATCHDOG    "watchdog_ms"
#define PROTO_K_ODOM_HZ     "odom_hz"
#define PROTO_K_SLEW_V      "slew_v"
#define PROTO_K_SLEW_W      "slew_w"
#define PROTO_K_TX_BUDGET   "tx_bytes_per_s"
#define PROTO_K_MAX_LINE    "max_line_len"
#define PROTO_K_LOG_LEVEL   "log_level"

#ifdef __cplusplus
}
#endif
