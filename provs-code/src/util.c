#include "main.h"

#include <math.h>

#include "util.h"
#include "globals.h"

void setDriveLeft( int power ) {

  motorSet( MOT_LDRIVE_1, -power );
  motorSet( MOT_LDRIVE_2, -power );

}

void setDriveRight( int power ) {

  motorSet( MOT_RDRIVE_1, power );
  motorSet( MOT_RDRIVE_2, power );

}

void setIntake( int power ) {

  motorSet( MOT_INTAKE, -power );

}

void setIndex( int power ) {

  motorSet( MOT_INDEX, -power );

}

void setFly( int power ) {

  motorSet( MOT_FLY, -power );

}

int getOSEFly() {

  return encoderGet( flyOSE );

}

int getOSEDriveLeft() {

  return encoderGet( driveLeftOSE );

}

int getOSEDriveRight() {

  return encoderGet( driveRightOSE );

}

int getOSEDrivePerp() {

  return encoderGet( drivePerpOSE ) / 2;

}

bool isBallTop() {

  return analogRead( LINE_TOP ) < 1700;

}

bool isBallBottom() {
 
  return analogRead( LINE_BOTTOM ) < 2000;

}

bool isBallPrimed() {

  return analogRead( LINE_PRIMED ) < 2000;

}

void gyrosReset() {

  gyroReset( gyroNormal );
  gyroReset( gyroInverted );

}

double getGyroValue() {

  return ( (double) gyroGet( gyroNormal ) - (double) gyroGet( gyroInverted ) ) / 2.0;

}

double ticksToInches( int ticks ) {

  return ( (double) ticks / 360.0 ) * 2.75 * PI;

}

int inchesToTicks( double inches ) {

  return (int) ( ( inches / ( 2.75 * PI ) ) * 360.0 );

}

int clipInt( int power, int limit ) {

  if ( power > limit ) {
    return limit;
  } else if ( power < -limit ) {
    return -limit;
  } else {
    return power;
  }

}

double clipDouble( double power, double limit ) {

  if ( power > limit ) {
    return limit;
  } else if ( power < -limit ) {
    return -limit;
  } else {
    return power;
  }

}

double clipAnglePi( double angle ) {

  double temp = angle;

  while ( temp > PI ) {
    temp -= 2 * PI;
  }
  while ( temp <= -PI ) {
    temp += 2 * PI;
  }

  return temp;

}

double absDouble( double x ) {

  return x < 0 ? -x : x;

}

double radiansToDegrees( double radians ) {

  return 360.0 * radians / ( 2 * PI );

}

double degreesToRadians( double degrees ) {

  return 2 * PI * degrees / 360.0;

}

double distanceBetween( double x1, double y1, double x2, double y2 ) {

  return sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );

}
