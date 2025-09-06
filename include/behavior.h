#pragma once

void initializeBehavior();
void updateBehavior();
// Wall-follow control
void setWallFollowSide(bool followRight);
void toggleWallFollowSide();
// Enable/disable wandering translation in AUTONOMOUS. When disabled, behavior fidgets in place.
void setBehaviorWanderEnabled(bool enabled);
