#pragma once

#include <stdbool.h>

// Initialize startup presence (lively behavior at boot)
void initPresence();
// Tick presence each loop; pass whether passthrough is active and if sleeping
void updatePresence(bool inPassthrough, bool sleeping);
// Returns true while presence is temporarily driving LED flashes
bool presenceLedOverlayActive();
// Pattern to use while overlay is active (varies randomly)
int presenceOverlayPattern();
