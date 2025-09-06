#pragma once

void initMotors();
void forwardOneTick();
void backwardOneTick();
void turnLeftOneTick();
void turnRightOneTick();
// Smoother, gentler in-place turns with ramped speeds suitable for idle fidgets
void gentleTurnLeft();
void gentleTurnRight();
// Gentle forward arcs to bias heading without in-place spins
void veerLeftOneTick();
void veerRightOneTick();
// Gentle eased forward arcs for lifelike idle motion
void gentleVeerLeft();
void gentleVeerRight();
void stopAllMotors();
void alertFreeze();

// Scale all behavior/presence motion speeds (0.0..1.0). Forebrain TWIST unaffected.
void setMotionSpeedScale(float scale);
