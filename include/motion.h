#pragma once

void initMotors();
void forwardOneTick();
void backwardOneTick();
void turnLeftOneTick();
void turnRightOneTick();
// Gentle forward arcs to bias heading without in-place spins
void veerLeftOneTick();
void veerRightOneTick();
void stopAllMotors();
void alertFreeze();
