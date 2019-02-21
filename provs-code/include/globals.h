#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "flywheel.h"
#include "odometry.h"

volatile extern FlyController flywheel;

volatile extern PositionTracker robot;

volatile extern bool anglePIDActive;

#endif
