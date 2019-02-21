#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdbool.h>

typedef struct {

  // xy pos in inches
  double x, y;

  // angle
  double initTheta;
  double prevTheta;
  double theta;
  double targetTheta;

  // encoder values
  long prevLeftOSE;
  long prevRightOSE;
  long prevPerpOSE;

  bool turningEnabled;

} PositionTracker;

void initPositionTracker( volatile PositionTracker* robot );

void updatePosition( volatile PositionTracker* robot );

#endif
