#include "autonFunctions.h"

#include <math.h>
#include "main.h"
#include "util.h"
#include "odometry.h"
#include "globals.h"

volatile bool intakeTaskActive2;

void taskAnglePID( void* parameter ) {

  updatePosition( &robot );

  double prevTarget = robot.targetTheta;

  // pid constents
  double kp = 75.0;
  double kd = 25.0;
  double ki = 40.0;

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
      if ( absDouble( drivePower ) < 60 ) {
        drivePower += integral * ki;
      }

      drivePower = clipDouble( drivePower, 60 );

      if ( drivePower > 0 && drivePower < 25 ) {
        drivePower = 25;
      } else if ( drivePower < 0 && drivePower > -25 ) {
        drivePower = -25;
      }

      if ( isAutonomous() && robot.turningEnabled ) {
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
      if ( absDouble( drivePower ) < 60 ) {
        drivePower += integral * ki;
      }

      drivePower = clipDouble( drivePower, 60 );

      if ( drivePower > 0 && drivePower < 25 ) {
        drivePower = 25;
      } else if ( drivePower < 0 && drivePower > -25 ) {
        drivePower = -25;
      }

      if ( isAutonomous() && robot.turningEnabled ) {
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

  int ticks;
  if ( inches >= 6.0 ) {
    ticks = inchesToTicks( inches - 5.0 );
  } else {
    ticks = inchesToTicks( inches - 3.0 );
  }

  int error;
  double kp = 0.0;

  encoderReset( driveLeftOSE );
  encoderReset( driveRightOSE );

  if ( !isAutonomous() ) {
    return;
  }

  if ( forward ) {

    while ( getOSEDriveLeft() < ticks && getOSEDriveRight() < ticks ) {

      error = getOSEDriveLeft() - getOSEDriveRight();

      setDriveLeft( power * ( 1 - error * kp ) );
      setDriveRight( power * ( 1 + error * kp ) );

      delay( 20 );

    }

    if ( brake ) {

      setDriveLeft( -30 );
      setDriveRight( -30 );

      delay( 100 );

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

      setDriveLeft( 30 );
      setDriveRight( 30 );

      delay( 100 );

    }

    setDriveLeft( 0 );
    setDriveRight( 0 );

  }

}

void turnDumb( int ticks, int power, bool forward, bool brake ) {

  encoderReset( driveLeftOSE );
  encoderReset( driveRightOSE );

  if ( !isAutonomous() ) {
    return;
  }

  if ( forward ) {

    while ( getOSEDriveLeft() > -ticks && getOSEDriveRight() < ticks ) {

      setDriveLeft( -power );
      setDriveRight( power );

      delay( 20 );

    }

    if ( brake ) {

      setDriveLeft( 30 );
      setDriveRight( -30 );

      delay( 100 );

    }

    setDriveLeft( 0 );
    setDriveRight( 0 );

  } else {

    while ( getOSEDriveLeft() < ticks && getOSEDriveRight() > -ticks ) {

      setDriveLeft( power );
      setDriveRight( -power );

      delay( 20 );

    }

    if ( brake ) {

      setDriveLeft( -30 );
      setDriveRight( 30 );

      delay( 100 );

    }

    setDriveLeft( 0 );
    setDriveRight( 0 );

  }

}

void taskIntake( void* parameter ) {

  while ( true ) {

    if ( isAutonomous() && intakeTaskActive ) {

      if ( isBallPrimed() ) {
        if ( isBallBottom() ) {
          setIntake( 0 );
        } else {
          setIntake( 60 );
        }
      }

      if ( isBallTop() ) {
        if ( isBallBottom() ) {
          setIntake( 0 );
        } else {
          setIntake( 45 );
        }
      } else {
        setIntake( 100 );
      }

    }

    delay( 20 );

  }

}

void taskScraper( void* parameter ) {

  while ( true ) {

    if ( isAutonomous() && scraperTaskActive ) {

      if ( scraperDown ) {
        if ( analogRead( POT_ARM ) < 1600 ) {
          setIndex( -127 );
        } else {
          setIndex( -15 );
        }
      } else {
        setIndex( -15 );
      }

    }

    delay( 20 );

  }

}

void primeBall() {

  intakeTaskActive = false;
  scraperTaskActive = false;

  if ( isBallTop() ) {
    while ( !isBallPrimed() ) {
      setIndex( 40 );
      setIntake( 40 );

      delay( 20 );
    }
    setIndex( 0 );
    setIntake( 0 );
  }

  intakeTaskActive = true;
  scraperTaskActive = true;

}

void shootBall() {

  intakeTaskActive = false;
  scraperTaskActive = false;

  while ( !isBallPrimed() ) {
    setIndex( 40 );
    setIntake( 40 );

    delay( 20 );
  }
  setIntake( 0 );

  setIndex( 127 );

  delay( 100 );

  setIndex( 0 );

  intakeTaskActive = true;
  scraperTaskActive = true;

}

void waitBallTop() {

  while ( !isBallTop() ) {
    delay( 20 );
  }

}

void waitBallBottom() {

  while ( !isBallBottom() ) {
    delay( 20 );
  }

}

void waitFlyAtSpeed() {

  while( !flyAtTarget( &flywheel ) ) {
    delay( 20 );
  }

}
