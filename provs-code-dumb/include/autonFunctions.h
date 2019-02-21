#ifndef AUTON_FUNCTIONS_H_
#define AUTON_FUNCTIONS_H_

#include "odometry.h"

void taskAnglePID( void* parameter );

void setAngle( volatile PositionTracker* robot, double radians );
void setAngleWithPoint( volatile PositionTracker* robot, double targetX, double targetY );

void waitAngle( volatile PositionTracker* robot );

void driveStraight( double inches, int power, bool forward, bool brake );
void turnDumb( int ticks, int power, bool forward, bool brake );

void taskIntake( void* parameter );

void enableTaskIntake();
void disableTaskIntake();

void taskScraper( void* parameter );

void primeBall();
void shootBall();

void waitBallTop();
void waitBallBottom();
void waitFlyAtSpeed();

#endif
