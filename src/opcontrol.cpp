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
pros::ADIGyro gyro(2);
pros::ADIEncoder leftEncoder(3, 4, true);
pros::ADIEncoder rightEncoder(5, 6, true);

void opcontrol() //0.0078740157480315 = quadradic value
{
	for(int i = 0; i < leftDrive.size(); i++)
	{
		leftDrive[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}
	for(int i = 0; i < rightDrive.size(); i++)
	{
		rightDrive[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}
	while (true)
	{
		driveMotorsSpeed(cVal(ANALOG_RIGHT_Y),rightDrive);
		driveMotorsSpeed(cVal(ANALOG_LEFT_Y),leftDrive);

		if(cVal(DIGITAL_R1))
		{
			motorGroupMove(127, intakeM);
		}
		else if(cVal(DIGITAL_R2))
		{
			motorGroupMove(-127, intakeM);
		}
		else
		{
			motorGroupMove(0, intakeM);
		}

		if(cVal(DIGITAL_DOWN))
		{
			lift.move(127);
		}
		else if(cVal(DIGITAL_B))
		{
			lift.move(-127);
		}
		else
		{
			lift.move(0);
		}

		if(cVal(DIGITAL_L1))
		{
			tilter.move(80);
		}
		else if(cVal(DIGITAL_L2))
		{
			tilter.move(-120);
		}
		else
		{
			tilter.move(0);
		}
		
		pros::lcd::print(3,"%f", leftEncoder.get_value());
		//pros::lcd::print(4,"%f", gyro.get_value());
		pros::lcd::print(2,"%d", leftEncoder.get_value());

		if(cVal(DIGITAL_X))
		{
			rightDrive[1].tare_position();
		}

		if(autonTest == true)
		{
			if(cVal(DIGITAL_A))
			{
				autonomous();
			}
		}
		pros::delay(3);
	}
}