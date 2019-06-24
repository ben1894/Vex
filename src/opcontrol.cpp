#include "main.h"
#include "forwardDeclairations.hpp"
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() 
{
	while (true) 
	{
		for(int i = 0; i < leftDrive.size(); i++)
		{		
			rightDrive[i].move(cVal(ANALOG_LEFT_Y));
			leftDrive[i].move(cVal(ANALOG_RIGHT_Y));
		}
		if(autonTest == true)
		{
			if(cVal(DIGITAL_A))
			{
				autonomous();
			}
		}
	}
}
