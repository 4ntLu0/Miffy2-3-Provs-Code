#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "flywheel.h"
#include "odometry.h"

volatile extern FlyController flywheel;

volatile extern PositionTracker robot;

volatile extern bool anglePIDActive;

volatile extern bool intakeTaskActive;

volatile extern bool scraperTaskActive;
volatile extern bool scraperDown;

#endif
