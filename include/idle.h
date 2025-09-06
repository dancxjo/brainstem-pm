#pragma once
#include <stdbool.h>

// Initialize idle manager with optional timeout (ms). Defaults to 5 minutes.
void initIdle(unsigned long timeoutMs = 300000);
// Update idle behavior based on USB connection status. Call each loop.
void updateIdle(bool usbConnected);
// Query whether idle fidget behavior is active.
bool idleIsActive();
// Query whether system is in low-battery sleep mode.
bool idleIsSleeping();
