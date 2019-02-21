#ifndef UTIL_H_
#define UTIL_H_

#include <stdbool.h>

#define PI 3.14159265358979323846

typedef struct {

  double x, y;

} Point;

void setDriveLeft( int power );
void setDriveRight( int power );
void setIntake( int power );
void setIndex( int power );
void setFly( int power );

int getOSEFly();
int getOSEDriveLeft();
int getOSEDriveRight();
int getOSEDrivePerp();

bool isBallTop();
bool isBallBottom();
bool isBallPrimed();

void gyrosReset();
double getGyroValue();

double ticksToInches( int ticks );
int inchesToTicks( double inches );

int clipInt( int power, int limit );
double clipDouble( double power, double limit );

double clipAnglePi( double angle );

double absDouble( double x );

double radiansToDegrees( double radians );
double degreesToRadians( double degrees );

double distanceBetween( double x1, double y1, double x2, double y2 );

#endif
