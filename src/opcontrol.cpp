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

pros::Imu gyroI(18);
pros::ADIEncoder leftEncoder(3, 4, true);
pros::ADIEncoder rightEncoder(5, 6, false);

void opcontrol() //0.0078740157480315 = quadradic value
{
	Timer controllerTimer;
	Timer matchTimer;
	Timer rumbleTimer;
	rumbleTimer.clear();
	matchTimer.clear();
	mainController.clear();
	controllerTimer.clear();
	bool clear = false;
	int oldButtonY = 0;
	int intakeHoldingPower = 0;
	leftEncoder.reset();
	rightEncoder.reset();
	rightDrive[0].tare_position();
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

		if(cVal(DIGITAL_Y) != oldButtonY)
		{
			if(cVal(DIGITAL_Y) == 1)
			{
				if(intakeHoldingPower == 0)
				{
					intakeHoldingPower = 50;
				}
				else 
				{
					intakeHoldingPower = 0;
				}
			}
		}
		if(cVal(DIGITAL_R1))
		{
			motorGroupMove(127, intakeM);
		}
		else if(cVal(DIGITAL_RIGHT))
		{
			motorGroupMove(-69, intakeM);
		}
		else if(cVal(DIGITAL_R2))
		{
			motorGroupMove(-127, intakeM);
		}
		else
		{
			motorGroupMove(intakeHoldingPower, intakeM);
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
			if(tilter.get_position() > 5300)
			{
				tilter.move(81);
			}
			else
			{
				tilter.move(127);
			}
		}
		else if(cVal(DIGITAL_L2))
		{
			tilter.move(-127);
		}
		else
		{
			tilter.move(0);
		}
		//pros::lcd::print(1,"%f", lift.get_position());
		//pros::lcd::print(3,"%d", rightEncoder.get_value());
		//GyroDistances test;
		//getDistances(test, 90);
		pros::lcd::print(4,"%f", actualGyroPosition());
		pros::lcd::print(2,"%d", leftEncoder.get_value()); //regular, no negative, no over anymore
		pros::lcd::print(3,"%d", rightEncoder.get_value());
		pros::lcd::print(1,"%f", lift.get_position());
		//pros::lcd::print(5,"%f", gyro.get_vex_heading());
		//pros::lcd::print(2,"%d", leftEncoder.get_value());

		if(controllerTimer.current() > 110)
		{
			if(clear == false)
			{
				mainController.print(0,0,"%d",(int)((105000-matchTimer.current())/1000));
				mainController.print(1,0,"%f",pros::battery::get_capacity());
				clear = true;
			}
			else 
			{
				clear = false;
				mainController.clear();
			}
			controllerTimer.clear();
		}
		if(rumbleTimer.current() > 30000)
		{
			//mainController.rumble("-");
			rumbleTimer.clear();
		}

		if(autonTest == true)
		{
			if(cVal(DIGITAL_X))
			{
				resetGyro();
				//resetAutonVals();
				//posTest();
			}
			if(cVal(DIGITAL_A))
			{
				resetGyro();
				autonomous();
			}
		}
		oldButtonY = cVal(DIGITAL_Y);
		pros::delay(3);
	}
}