/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

#include "util.h"
#include "globals.h"
#include "flywheel.h"
#include "autonFunctions.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */

/*

TURNING CODE

encoderReset( driveLeftOSE );
encoderReset( driveRightOSE );
encoderReset( drivePerpOSE );
robot.initTheta = 0.0;
robot.x = 0.0;
robot.y = 0.0;
initPositionTracker( &robot );

setAngle( &robot, degreesToRadians( INSERT ANGLE IN DEGREES HERE ) );
robot.turningEnabled = true;

waitAngle( &robot );

delay( 1000 );

setDriveLeft( 0 );
setDriveRight( 0 );
robot.turningEnabled = false;

*/

void autonomous() {

  int potDiv1[9] = { 0, 510, 1020, 1500, 1990, 2500, 3020, 3610, 4100 };
  int potDiv2[9] = { 0, 350, 820, 1300, 1800, 2320, 2860, 3450, 4100 };

  encoderReset( driveLeftOSE );
  encoderReset( driveRightOSE );
  encoderReset( drivePerpOSE );
  robot.initTheta = 0.0;
  robot.x = 0.0;
  robot.y = 0.0;
  initPositionTracker( &robot );

  initFlyController( &flywheel );
  TaskHandle taskFlyControlHandle = taskCreate( taskFlyControl, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );
  flySetRpm( &flywheel, 0, 0 );

  scraperTaskActive = true;
  scraperDown = false;
  TaskHandle taskScraperHandle = taskCreate( taskScraper, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );

  setAngle( &robot, degreesToRadians( 0.0 ) );
  TaskHandle taskAnglePIDHandle = taskCreate( taskAnglePID, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );
  robot.turningEnabled = false;

  if ( analogRead( POT_1 ) > potDiv1[0] && analogRead( POT_1 ) < potDiv1[1] ) { // red front

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[1] && analogRead( POT_1 ) < potDiv1[2] ) { // blue front

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[2] && analogRead( POT_1 ) < potDiv1[3] ) { // red back

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[3] && analogRead( POT_1 ) < potDiv1[4] ) { // blue back

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

      // 2 flag center
      // setup: aligned with perimeter centered on tile facing cap

        // park

        flySetRpm( &flywheel, 2600, 0.40 );
        setIntake( 40 );

        driveStraight( 40.0, 60, true, false );

        driveStraight( 9.0, 40, true, true );

        delay( 500 );

        driveStraight( 12.0, 60, false, true );

        waitBallBottom();
        setIntake( 0 );

        delay( 500 );

        turnDumb( 150, 40, false, true );

        primeBall();
        shootBall();

        delay( 500 );

        flySetRpm( &flywheel, 2350, 0.32 );;

        waitFlyAtSpeed();
        delay( 100 );

        setIntake( 40 );
        waitBallTop();
        setIntake( 0 );
        primeBall();
        shootBall();

        turnDumb( 150, 40, true, true );

        delay( 500 );

        driveStraight( 30.0, 60, false, true );

        setDriveLeft( -50 );
        setDriveRight( -50 );

        delay( 500 );

        setDriveLeft( 0 );
        setDriveRight( 0 );

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[4] && analogRead( POT_1 ) < potDiv1[5] ) { // skills

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  }

  delay( 15000 );

  taskDelete( taskFlyControlHandle );
  taskDelete( taskScraperHandle );
  taskDelete( taskAnglePIDHandle );

}
