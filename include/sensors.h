#pragma once

void initSensors();
void beginSensorStream();
void updateSensorStream();
// Pause/resume helpers for the OI stream when switching modes
void pauseSensorStream();
void resumeSensorStream();
// Simple queries from cached OI stream
bool wallDetected();
bool playButtonPressedAndClear();
bool advanceButtonPressedAndClear();
void beginSensorStream();
void updateSensorStream();
bool oiConnected();
int scanEnvironment();       // -1 = left, 1 = forward, 2 = right, 0 = none
bool bumperTriggered();
bool cliffDetected();

// Optional: external bumper interrupt support
// If your bumper switch is wired to a GPIO, call initSensors() and this will
// auto-attach on supported boards. The event flag can be polled in the loop.
bool bumperEventTriggeredAndClear();
int batteryPercent();
void setBatteryPercentOverride(int pct);
