#include "autonFunctions.h"

#include <math.h>
#include "main.h"
#include "util.h"
#include "odometry.h"
#include "globals.h"

void taskAnglePID( void* parameter ) {

  updatePosition( &robot );

  double prevTarget = robot.targetTheta;

  // pid constents
  double kp = 110.0;
  double kd = 20.0;
  double ki = 13.0;

  long prevTime = micros();
  long deltaTime;

  double error;
  double prevError;
  double integral = 0;
  double derivative = 0;

  double drivePower;

  if ( clipAnglePi( robot.targetTheta - robot.theta ) >= 0 ) {

    if ( robot.targetTheta != prevTarget ) {
      error = clipAnglePi( robot.targetTheta - robot.theta );
      prevError = error;
    }

  } else {

    if ( robot.targetTheta != prevTarget ) {
      error = -clipAnglePi( robot.targetTheta - robot.theta );
      prevError = error;
    }

  }

  prevTarget = robot.targetTheta;

  while ( true ) {

    updatePosition( &robot );

    if ( clipAnglePi( robot.targetTheta - robot.theta ) >= 0 ) {

      if ( robot.targetTheta != prevTarget ) {
        error = clipAnglePi( robot.targetTheta - robot.theta );
        prevError = error;
      }

    } else {

      error = -clipAnglePi( robot.targetTheta - robot.theta );
      prevError = error;

    }

    if ( clipAnglePi( robot.targetTheta - robot.theta ) >= 0 ) {

      deltaTime = micros() - prevTime;
      deltaTime = deltaTime > 0 ? deltaTime : 1;
      prevTime = micros();

      error = clipAnglePi( robot.targetTheta - robot.theta );

      integral += error * ( deltaTime / 1000000.0 );
      derivative = ( error - prevError ) / ( deltaTime / 1000000.0 );

      prevError = error;

      // if speed is close to target, reset integral
      if ( absDouble( error ) < 1.0 * PI / 180.0 ) {
        integral = 0;
      }

      drivePower = error * kp + derivative * kd;

      // if motor is max torque, do not integrate error
      if ( absDouble( drivePower ) < 75 ) {
        drivePower += integral * ki;
      }

      drivePower = clipDouble( drivePower, 75 );

      if ( drivePower > 0 && drivePower < 25 ) {
        drivePower = 25;
      } else if ( drivePower < 0 && drivePower > -25 ) {
        drivePower = -25;
      }

      if ( !isAutonomous() && anglePIDActive ) {
        setDriveLeft( -drivePower );
        setDriveRight( drivePower );
      }

    } else {

      deltaTime = micros() - prevTime;
      deltaTime = deltaTime > 0 ? deltaTime : 1;
      prevTime = micros();

      error = -clipAnglePi( robot.targetTheta - robot.theta );

      integral += error * ( deltaTime / 1000000.0 );
      derivative = ( error - prevError ) / ( deltaTime / 1000000.0 );

      prevError = error;

      // if speed is close to target, reset integral
      if ( absDouble( error ) < 1.0 * PI / 180.0 ) {
        integral = 0;
      }

      drivePower = error * kp + derivative * kd;

      // if motor is max torque, do not integrate error
      if ( absDouble( drivePower ) < 75 ) {
        drivePower += integral * ki;
      }

      drivePower = clipDouble( drivePower, 75 );

      if ( drivePower > 0 && drivePower < 25 ) {
        drivePower = 25;
      } else if ( drivePower < 0 && drivePower > -25 ) {
        drivePower = -25;
      }

      if ( !isAutonomous() && anglePIDActive ) {
        setDriveLeft( drivePower );
        setDriveRight( -drivePower );
      }

    }

    delay( 20 );

  }

}

void setAngle( volatile PositionTracker* robot, double radians ) {

  robot->targetTheta = radians;

}

void setAngleWithPoint( volatile PositionTracker* robot, double targetX, double targetY ) {

  double deltaX = targetX - robot->x;
  double deltaY = targetY - robot->y;

  robot->targetTheta = atan2( deltaY, deltaX );

}

void waitAngle( volatile PositionTracker* robot ) {

  int counter = 0;

  while ( counter < 25 ) {

    if ( absDouble( robot->targetTheta - robot->theta ) < degreesToRadians( 5.0 ) ) {
      counter++;
    } else {
      counter = 0;
    }

    delay( 20 );

  }

}

void driveStraight( double inches, int power, bool forward, bool brake ) {

  int ticks = inchesToTicks( inches );
  int error;
  double kp = 0.0;

  encoderReset( driveLeftOSE );
  encoderReset( driveRightOSE );

  if ( forward ) {

    while ( getOSEDriveLeft() < ticks && getOSEDriveRight() < ticks ) {

      error = getOSEDriveLeft() - getOSEDriveRight();

      setDriveLeft( power * ( 1 - error * kp ) );
      setDriveRight( power * ( 1 + error * kp ) );

      delay( 20 );

    }

    if ( brake ) {

      setDriveLeft( -15 );
      setDriveRight( -15 );

    }

    setDriveLeft( 0 );
    setDriveRight( 0 );

  } else {

    while ( getOSEDriveLeft() > -ticks && getOSEDriveRight() > -ticks ) {

      error = getOSEDriveLeft() - getOSEDriveRight();

      setDriveLeft( -power * ( 1 + error * kp ) );
      setDriveRight( -power * ( 1 - error * kp ) );

      delay( 20 );

    }

    if ( brake ) {

      setDriveLeft( 15 );
      setDriveRight( 15 );

    }

    setDriveLeft( 0 );
    setDriveRight( 0 );

  }

}
