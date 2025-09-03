#pragma once

void initSensors();
int scanEnvironment();       // -1 = left, 1 = forward, 2 = right, 0 = none
bool bumperTriggered();
bool cliffDetected();
