#include "main.h"

#include <math.h>

#include "util.h"
#include "globals.h"

#include "flywheel.h"

void initFlyController( volatile FlyController* fly ) {

  fly->targetRPM = 0;

}

// using the controller (setting a target rpm)
void flySetRpm( volatile FlyController* fly, int rpm, double drivePredicted ) {

  fly->prevRPM = fly->currentRPM;

  // reset error
  fly->errorRPM = rpm - fly->currentRPM;
  fly->prevErrorRPM = fly->errorRPM;

  fly->prevTime = micros();

  fly->driveApprox = drivePredicted;

  // set first zero crossing flag
  // clear tbh variable
  if ( fly->targetRPM > rpm && rpm != 0 ) {

    fly->firstCross = false;
    fly->drive = fly->driveApprox;
    fly->driveAtZero = fly->drive;

  } else {

    fly->firstCross = true;
    fly->driveAtZero = 0;

  }

  /*
  fly->firstCross = true;
  fly->driveAtZero = 0;
  */

  // set target
  fly->targetRPM = rpm;

}

void flyCalculateSpeed( volatile FlyController* fly ) {

  long deltaTime;

  // get time difference and set prev time
  deltaTime = micros() - fly->prevTime;
  fly->prevTime = micros();


  // Calculate velocity in rpm
  fly->current = 180.0 * ( getOSEFly() / 360.0 ) * ( 1000000.0 / deltaTime );

  // reset OSE
  encoderReset( flyOSE );

}

void flyControlUpdateVelocityTBH( volatile FlyController* fly ) {

  // calculate error
  if ( fly->currentRPM < 0 || fly->currentRPM > 5000 ) {
    fly->currentRPM = 0;
  }

  fly->errorRPM = fly->targetRPM - fly->currentRPM;

  fly->drive = fly->drive + ( fly->errorRPM * fly->gain );

  // clip
  if ( fly->drive > 1 ) {
    fly->drive = 1;
  } else if ( fly->drive < 0 ) {
    fly->drive = 0;
  }

  // check for zero cross
  if ( sign( fly->errorRPM ) != sign( fly->prevErrorRPM ) ) {

    // first zero crossing
    if ( fly->firstCross ) {

      fly->drive = fly->driveApprox;

      fly->firstCross = false;

    } else {

      // take back half
      fly->drive = 0.5 * ( fly->drive + fly->driveAtZero );

    }

    // save TBH value
    fly->driveAtZero = fly->drive;

  }

  fly->prevErrorRPM = fly->errorRPM;

}

void taskFlyControl( void* parameter ) {

  volatile FlyController* fly = &flywheel;

  // set gain
  fly->gain = 0.00005;

  while ( true ) {

    flyCalculateSpeed( fly );

    // set rpm for TBH calculations
    fly->currentRPM = fly->current;

    flyControlUpdateVelocityTBH( fly );

    fly->flyPower = fly->drive * 127;

    fly->flyPower = clipInt( fly->flyPower, 127 ) + 0.5;

    // set the motor
    setFly( fly->flyPower );

    delay( FLY_INTERVAL );

  }

}

bool flyAtTarget( volatile FlyController* fly ) {

  return fly->targetRPM - fly->currentRPM > -50 && fly->targetRPM - fly->currentRPM < 50;

}

int sign( double x ) {
  return ( x < 0 ) - ( x > 0 );
}
