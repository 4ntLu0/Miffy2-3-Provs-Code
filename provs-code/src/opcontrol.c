/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
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
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */

void taskDrive( void* parameter );
void taskFly( void* parameter );

void operatorControl() {

	encoderReset( driveLeftOSE );
	encoderReset( driveRightOSE );
	encoderReset( drivePerpOSE );

	/*
	robot.initTheta = 0.0;
	robot.x = 0.0;
	robot.y = 0.0;
	initPositionTracker( &robot );

	setAngle( &robot, degreesToRadians( 90.0 ) );
	anglePIDActive = true;
	TaskHandle taskAnglePIDHandle = taskCreate( taskAnglePID, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );

	waitAngle( &robot );

	setDriveLeft( 0 );
	setDriveRight( 0 );
	taskDelete( taskAnglePIDHandle );
	*/

	/*
	driveStraight( 12.0, 80, true, true );
	*/

	initFlyController( &flywheel );
	TaskHandle taskFlyControlHandle = taskCreate( taskFlyControl, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );

	flySetRpm( &flywheel, 3500, 0.0 );
	while ( true ) {
		printf("%.4f, %d, %d, %.2f\n", flywheel.drive, flywheel.targetRPM, flywheel.firstCross ? 1 : 0, flywheel.currentRPM );
		delay( 20 );
	}

	//TaskHandle taskDriveHandle = taskCreate( taskDrive, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );
	//TaskHandle taskFlyHandle = taskCreate( taskFly, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );
	//TaskHandle taskFlyControlHandle = taskCreate( taskFlyControl, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT );

	while ( true ) { delay( 20 ); }

	//taskDelete( taskDriveHandle );
	//taskDelete( taskFlyHandle );
	taskDelete( taskFlyControlHandle );

}

void taskDrive( void* parameter ) {

	while ( true ) {

		if ( !isAutonomous() ) {
			/*
			setDriveLeft( joystickGetAnalog( JOY_MASTER, AXIS_LEFT_V ) + joystickGetAnalog( JOY_MASTER, AXIS_RIGHT_H ) );
			setDriveRight( joystickGetAnalog( JOY_MASTER, AXIS_LEFT_V ) - joystickGetAnalog( JOY_MASTER, AXIS_RIGHT_H ) );
			*/
			setDriveLeft( joystickGetAnalog( JOY_MASTER, AXIS_RIGHT_V ) + joystickGetAnalog( JOY_MASTER, AXIS_RIGHT_H ) );
			setDriveRight( joystickGetAnalog( JOY_MASTER, AXIS_RIGHT_V ) - joystickGetAnalog( JOY_MASTER, AXIS_RIGHT_H ) );
		}

		delay( 20 );

	}

}

void taskFly( void* parameter ) {

	bool flag = false;
	bool flag2 = false;
	bool ballInBot;
	int target = 0;

	while ( true ) {

		if ( !isAutonomous() ) {

			if ( joystickGetDigital( JOY_MASTER, 5, JOY_UP ) ) {
				if ( isBallTop() ) {
					if ( isBallBottom() ) {
						setIntake( 0 );
					} else {
						setIntake( 45 );
					}
				} else {
					setIntake( 100 );
				}
			} else if ( joystickGetDigital( JOY_MASTER, 5, JOY_DOWN ) ) {
				setIntake( -127 );
			} else {
				setIntake( 0 );
			}

			if ( joystickGetDigital( JOY_MASTER, 8, JOY_RIGHT ) ) {
				setIntake( 127 );
			}

			if ( isBallTop() ) {
				if ( joystickGetDigital( JOY_MASTER, 6, JOY_UP ) && !flag ) {

					flag = true;

					ballInBot = isBallBottom();

					while ( !isBallPrimed() ) {
						setIndex( 30 );
						setIntake( 40 );

						if ( joystickGetDigital( JOY_MASTER, 8, JOY_LEFT ) ) {
							break;
						}

						delay( 20 );
					}
					setIndex( 0 );
					setIntake( 0 );

				}
			}

			if ( !joystickGetDigital( JOY_MASTER, 6, JOY_UP ) && flag ) {

				flag = false;

				setIndex( 127 );

				delay( 100 );
				setIndex( 0 );

				if ( ballInBot ) {
					while ( !isBallTop() ) {
						setIntake( 70 );

						if ( joystickGetDigital( JOY_MASTER, 8, JOY_LEFT ) ) {
							break;
						}

						delay( 20 );
					}
				}
				setIntake( 0 );

			}

			// index override
			if ( joystickGetDigital( JOY_MASTER, 7, JOY_UP ) ) {

				if ( analogRead( POT_ARM ) < 1600 ) {
					setIndex( -127 );
				} else {
					setIndex( -15 );
				}

			} else if ( joystickGetDigital( JOY_MASTER, 8, JOY_DOWN ) ) {
				setIndex( 127 );
			} else {
				setIndex( 0 );
			}


			if ( joystickGetDigital( JOY_MASTER, 8, JOY_UP ) && !flag2 ) {
				if ( target != 2500 ) {
					flySetRpm( &flywheel, 2500, 0.47 );
					target = 2500;
				} else {
					flySetRpm( &flywheel, 0, 0 );
					target = 0;
				}
				flag2 = true;
			}
			if ( !joystickGetDigital( JOY_MASTER, 8, JOY_UP ) && flag2 ) {
				flag2 = false;
			}

			delay( 20 );

		}

	}

}
