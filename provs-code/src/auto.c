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

volatile bool intakeTaskActive;

void taskIntake( void* parameter );
void shootBall();
void waitBallTop();
void waitFlyAtSpeed();

void autonomous() {

  int potDiv1[9] = { 0, 510, 1020, 1500, 1990, 2500, 3020, 3610, 4100 };
  int potDiv2[9] = { 0, 350, 820, 1300, 1800, 2320, 2860, 3450, 4100 };

  TaskHandle taskFlyControlHandle = taskCreate( taskFlyControl, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );
  flySetRpm( &flywheel, 0, 0 );

  intakeTaskActive = true;
  TaskHandle taskIntakeHandle = taskCreate( taskIntake, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );

  encoderReset( driveLeftOSE );
  encoderReset( driveRightOSE );
  encoderReset( drivePerpOSE );

  anglePIDActive = true;

  if ( analogRead( POT_1 ) > potDiv1[0] && analogRead( POT_1 ) < potDiv1[1] ) {

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[1] && analogRead( POT_1 ) < potDiv1[2] ) {

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[2] && analogRead( POT_1 ) < potDiv1[3] ) {

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[3] && analogRead( POT_1 ) < potDiv1[4] ) {

    if ( analogRead( POT_2 ) > potDiv2[0] && analogRead( POT_2 ) < potDiv2[1] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[1] && analogRead( POT_2 ) < potDiv2[2] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[2] && analogRead( POT_2 ) < potDiv2[3] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[3] && analogRead( POT_2 ) < potDiv2[4] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[4] && analogRead( POT_2 ) < potDiv2[5] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[5] && analogRead( POT_2 ) < potDiv2[6] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[6] && analogRead( POT_2 ) < potDiv2[7] ) {

    } else if ( analogRead( POT_2 ) > potDiv2[7] && analogRead( POT_2 ) < potDiv2[8] ) {

    }

  } else if ( analogRead( POT_1 ) > potDiv1[4] && analogRead( POT_1 ) < potDiv1[5] ) {

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
  taskDelete( taskIntakeHandle );

}

void taskIntake( void* parameter ) {

  while ( true ) {

    if ( isAutonomous() && intakeTaskActive ) {

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

void shootBall() {

  intakeTaskActive = false;

  if ( !isBallBottom() ) {

    setIndex( 127 );
    setIntake( 60 );

    delay( 300 );

    setIntake( 0 );

    delay( 700 );

    setIndex( 0 );
    setIntake( 0 );

  } else {

    setIndex( 127 );
    setIntake( 50 );

    delay( 100 );

    setIndex( 0 );

    delay( 500 );

    setIntake( 0 );

  }

  intakeTaskActive = true;

}

void waitBallTop() {

  while ( !isBallTop() ) {
    delay( 20 );
  }

}

void waitFlyAtSpeed() {

  while( !flyAtTarget( &flywheel ) ) {
    delay( 20 );
  }

}
