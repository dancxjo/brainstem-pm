Pro Micro Brainstem — Wake, Handshake, and Passthrough for iRobot Create 1

Overview
- Board: Arduino Pro Micro (ATmega32U4)
- Robot: iRobot Create 1 (Open Interface, 57600 8N1)
- Role: Wake + jingle + READY handshake, then low‑latency Serial bridge

Wiring
- DB‑25 Pin 1 (RXD) ← Pro Micro TX (D1/TxO)
- DB‑25 Pin 2 (TXD) → Pro Micro RX (D0/RxI)
- DB‑25 Pin 3 (Power Toggle) ← Pro Micro D9
- DB‑25 GND ↔ Pro Micro GND
- Leave DB‑25 Pin 15 floating (don’t force 19200)

Serial
- Host USB CDC: 115200 (ignored by CDC but used here for clarity)
- Create OI UART: 57600 8N1

State Machine
- IDLE → (HELLO) → POWER_SEQUENCE → OI_INIT → BRIDGE_READY
- POWER_SEQUENCE: pulse Pin 9 (100 ms high), wait 2 s
- OI_INIT: send START(128), SAFE(131), LED pulse(139), define song(140) + play(141)
- BRIDGE_READY: full‑duplex USB↔Serial1 bridging

Host ↔ Brainstem Protocol
- Control messages (host → brainstem, ASCII lines):
  - HELLO\n — request wake/init + bridge
  - !power_cycle\n — pulse Pin 3, reinit OI, re‑enter bridge (BUSY printed immediately)
  - !cute\n — play jingle + LED pulse (non‑destructive; stays in bridge)
  - !status\n — dump JSON one‑liner with state/counters/baud
  - !reboot\n — soft reset the brainstem (watchdog)

- Responses (brainstem → host):
  - READY\n — wake/init complete; bridge is active
  - BUSY\n — init in progress (e.g., after !power_cycle or HELLO in progress)
  - ERR:<msg>\n — error string for unknown commands or unsupported ops
  - STATUS:{...}\n — JSON one‑liner metrics

Escape in Bridge
- Prefix any control while bridging with two bytes: 0xFF 0x00
  - Example: FF 00 !status\n
- Any bytes without the escape are bridged raw to the robot.

Timing Constants
- POWER_PULSE_HIGH_MS = 100 ms
- POWER_POST_DELAY_MS = 2000 ms
- OI_WRITE_GAP_MS = 20 ms (gap between OI opcodes)

Cute/Wake Sequence (OI)
- START(128), SAFE(131)
- LED pulse: 139, 0, <color 0..255>, <intensity 30..255>
- Song: 140,<song#=0>,<N=4>, 60,16, 64,16, 67,16, 72,24 then 141,0

Recovery
- !power_cycle triggers POWER_SEQUENCE → OI_INIT → READY within ~2–4 s.
- If USB disconnects, firmware stays in current state; reconnect host and send HELLO/escape commands as needed.

