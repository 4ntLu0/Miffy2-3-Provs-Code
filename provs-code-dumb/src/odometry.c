#include <math.h>

#include "main.h"

#include "odometry.h"
#include "util.h"

void initPositionTracker( volatile PositionTracker* robot ) {

  robot->prevLeftOSE = getOSEDriveLeft();
  robot->prevRightOSE = getOSEDriveRight();
  robot->prevPerpOSE = getOSEDrivePerp();

  robot->prevTheta = robot->initTheta;

}

void updatePosition( volatile PositionTracker* robot ) {

  double sR = 3.1;
  double sL = 3.1;
  double sP = 6.5;

  // get encoder values
  long leftOSE = getOSEDriveLeft();
  long rightOSE = getOSEDriveRight();
  long perpOSE = getOSEDrivePerp();

  // get change in encoder values
  long deltaR = rightOSE - robot->prevRightOSE;
  long deltaP = perpOSE - robot->prevPerpOSE;

  // update previous values
  robot->prevLeftOSE = leftOSE;
  robot->prevRightOSE = rightOSE;
  robot->prevPerpOSE = perpOSE;

  // get absolute angle
  robot->theta = robot->initTheta + ( ticksToInches( rightOSE ) - ticksToInches( leftOSE ) ) / ( sR + sL );
  robot->theta = clipAnglePi( robot->theta );

  // get change in angle
  double deltaTheta = robot->theta - robot->prevTheta;
  double avgTheta = robot->prevTheta + deltaTheta / 2.0;

  // update previous angle
  robot->prevTheta = robot->theta;

  double localDispX, localDispY;

  if ( deltaTheta == 0 ) {

    localDispX = ticksToInches( deltaR );
    localDispY = ticksToInches( deltaP );

  } else {

    localDispX = 2 * sin( deltaTheta / 2.0 ) * ( ticksToInches( deltaR ) / deltaTheta + sR );
    localDispY = 2 * sin( deltaTheta / 2.0 ) * ( ticksToInches( deltaP ) / deltaTheta + sP );

  }

  double globalDispX, globalDispY;
  globalDispX = localDispX * cos( avgTheta ) - localDispY * sin( avgTheta );
  globalDispY = localDispX * sin( avgTheta ) + localDispY * cos( avgTheta );

  robot->x += globalDispX;
  robot->y += globalDispY;

}
