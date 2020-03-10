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

pros::Imu gyroI(20);
pros::ADIPotentiometer pot(2);
pros::ADIEncoder leftEncoder(3, 4, false);
pros::ADIEncoder rightEncoder(7, 8, true);

void opcontrol() //0.0078740157480315 = quadradic value
{
	PidController liftPID = {regLiftP,regLiftI,regLiftD,regLiftMin,regLiftMax};
	lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
	Timer controllerTimer;
	Timer matchTimer;
	Timer rumbleTimer;
	Timer liftTimer;
	liftTimer.clear();
	rumbleTimer.clear();
	matchTimer.clear();
	mainController.clear();
	controllerTimer.clear();
	bool clear = false;
	int oldButtonY = 0;
	int liftTarget;
	//int intakeHoldingPower = 0;
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
		if(cVal(DIGITAL_L1))
		{
			if(tilter.get_position() > 5300)
			{
				tilterMove(81);
			}
			else
			{
				tilterMove(127);
			}
		}
		else if(cVal(DIGITAL_L2))
		{
			tilterMove(-127);
		}
		else 
		{
			if(cVal(DIGITAL_Y))
			{
				driveMotorsSpeed(-35,rightDrive);
				driveMotorsSpeed(-35,leftDrive);
			}
			else
			{
				driveMotorsSpeed(cVal(ANALOG_RIGHT_Y),rightDrive);
				driveMotorsSpeed(cVal(ANALOG_LEFT_Y),leftDrive);
			}
		}


		/*
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
		}*/
		if(cVal(DIGITAL_R1))
		{
			motorGroupMove(127, intakeM);
		}
		else if(cVal(DIGITAL_Y))
		{
			motorGroupMove(-60, intakeM);
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
			motorGroupMove(0, intakeM);
		}

		if(cVal(DIGITAL_DOWN))
		{
			liftTimer.clear();
			lift.move(127);
		}
		else if(cVal(DIGITAL_B))
		{
			liftTimer.clear();
			lift.move(-127);
		}
		else if(pot.get_value() < 300)
		{
			liftTimer.clear();
			lift.move(0);
		}
		else if(liftTimer.current() < 300)
		{
			liftTarget = pot.get_value();
			liftPID.reset();
			lift.move(0);
		}
		else
		{
			int liftSpeed;
			liftSpeed = liftPID.output(pot.get_value(), liftTarget);
			lift.move(liftSpeed);
		}

		//pros::lcd::print(1,"%f", lift.get_position());
		//pros::lcd::print(3,"%d", rightEncoder.get_value());
		//GyroDistances test;
		//getDistances(test, 90);
		pros::lcd::print(4,"%f", actualGyroPosition());
		pros::lcd::print(2,"%d", leftEncoder.get_value()); //regular, no negative, no over anymore
		//pros::lcd::print(2,"%d", leftEncoder.get_value()); //regular, no negative, no over anymore
		pros::lcd::print(3,"%d", rightEncoder.get_value());
		pros::lcd::print(1,"%f", tilter.get_position());
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
				//resetGyro();
				autonomous();
			}
		}
		oldButtonY = cVal(DIGITAL_Y);
		pros::delay(3);
	}
}