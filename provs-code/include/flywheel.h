#ifndef FLYWHEEL_H_
#define FLYWHEEL_H_

#define FLY_INTERVAL 50

#include <stdbool.h>

typedef struct {

  double current;
  long prevTime;

  int targetRPM;
  double currentRPM;
  double prevRPM;
  double errorRPM;
  double prevErrorRPM;
  double gain;
  double drive;
  double driveAtZero;
  bool firstCross;
  double driveApprox;

  int flyPower;

  bool rpmLowerNotZero;

} FlyController;

void initFlyController( volatile FlyController* fly );

void flySetRpm( volatile FlyController* fly, int rpm, double powerPredicted );
void flyCalculateSpeed( volatile FlyController* fly );
void flyControlUpdateVelocityTBH( volatile FlyController* fly );

bool flyAtTarget( volatile FlyController* fly );

void taskFlyControl( void* parameter );

int sign( double x );

#endif
