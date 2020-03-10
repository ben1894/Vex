#include "main.h"
#include "forwardDeclairations.hpp"

int selected = false;

const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;

Select count = UNDEFINED;

/*
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize
 */
pros::Controller mainController(pros::E_CONTROLLER_MASTER);
pros::Controller secondaryController(pros::E_CONTROLLER_PARTNER);
//front back
//left right
std::array<pros::Motor, 2> rightDrive{pros::Motor(16),pros::Motor(17)};
std::array<pros::Motor, 2>  leftDrive{pros::Motor(14),pros::Motor(15)};
std::array<pros::Motor, 2>     intakeM{pros::Motor(13),pros::Motor(5)};

pros::Motor tilter(8,pros::E_MOTOR_GEARSET_36,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor lift(3,pros::E_MOTOR_GEARSET_36,false,pros::E_MOTOR_ENCODER_COUNTS);

void initialize()
{
	rightDrive[1].set_reversed(true);
	leftDrive[1].set_reversed(true);

	pros::lcd::initialize();
	pros::Imu gyroI(20);
	while(gyroI.is_calibrating())
	{
		pros::delay(5);
	}
	for(int i = 0; i < leftDrive.size(); i++)
	{
		leftDrive[i].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		leftDrive[i].set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
		leftDrive[i].set_gearing(pros::E_MOTOR_GEARSET_18);
	}
		for(int i = 0; i < rightDrive.size(); i++)
	{
		rightDrive[i].set_reversed(true);
		rightDrive[i].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		rightDrive[i].set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
		rightDrive[i].set_gearing(pros::E_MOTOR_GEARSET_18);
	}

	for(int i = 0; i < intakeM.size(); i++)
	{
		intakeM[i].set_gearing(pros::E_MOTOR_GEARSET_18);
	}

	intakeM[1].set_reversed(true);
	intakeM[0].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	intakeM[1].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	tilter.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	tilter.set_reversed(true);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	if(autonTest == true)
	{
		competition_initialize();
	}
    tilter.tare_position();
	lift.tare_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void waitForRelease()
{
	while((pros::lcd::read_buttons() != 0) || cVal(DIGITAL_RIGHT) || cVal(DIGITAL_LEFT) || cVal(DIGITAL_DOWN))
	{
		pros::delay(5);
	}
}

void competition_initialize()
{
	pros::lcd::initialize();
	Select oldCount = UNDER;
	bool onceAcception = false;
	int oldLCD = 0;
	int oldRight = 0;
	int oldLeft = 0;
	mainController.clear();
	pros::Task::delay(50);
	mainController.print(0,0,"Down To Select");
	pros::lcd::print(2,  "Center to Select");
	pros::Task::delay(50);

	while(!selected)
	{
		if((pros::lcd::read_buttons() != oldLCD) || (cVal(DIGITAL_RIGHT) != oldRight) || (cVal(DIGITAL_LEFT) != oldLeft))
		{
			if((pros::lcd::read_buttons() ==  leftButton) || (cVal(DIGITAL_RIGHT)))
			{
				count = (Select)((int)count + 1);
			}
			else if((pros::lcd::read_buttons() == rightButton) || (cVal(DIGITAL_LEFT)))
			{
				count = (Select)((int)count - 1);
			}
		}
		if((((pros::lcd::read_buttons()) == centerButton)||(cVal(DIGITAL_DOWN))) && (count != UNDEFINED))
		{
			selected = true;
		} //count = negative  old = undefined

		if(count != oldCount)
		{
			mainController.clear();
			pros::Task::delay(50);
			switch(count)
			{
				case(UNDEFINED):
					pros::lcd::print(0, "< Auton Select >");
					mainController.print(0,0,"Down To Select");
					break;
				case(SMALLRED):
					pros::lcd::print(0, "Small Red");
					mainController.print(0,0,"Small Red");
					break;
				case(SMALLBLUE):
					pros::lcd::print(0, "Small Blue");
					mainController.print(0,0,"Small Blue");
					break;
				case(THICCRED):
					pros::lcd::print(0, "Thicc Red");
					mainController.print(0,0,"Thicc Red");
					break;
				case(THICCBLUE):
					pros::lcd::print(0, "Thicc Blue");
					mainController.print(0,0,"Thicc Blue");
					break;
				case(MICROCUBE):
					pros::lcd::print(0, "Micro Cube");
					mainController.print(0,0,"Micro Cube");
					break;
				case(UNDER):
					count = (Select)((int)OVER - 1);
					onceAcception = true;
					break;
				default: //OVER
					onceAcception = true;
					count = UNDEFINED;
					break;
			}
			pros::Task::delay(50);
		}

		if(onceAcception == true)
		{
			onceAcception = false;
			if(count == UNDEFINED)
			{
				oldCount = OVER;
			}
			else
			{
				oldCount = UNDER;
			}
		}
		else
		{
			oldCount = count; //always updates at end, needs to have bool to not do this
		}

		oldLCD = pros::lcd::read_buttons();
		oldRight = cVal(DIGITAL_RIGHT);
		oldLeft = cVal(DIGITAL_LEFT);
		pros::lcd::print(4,"%f", fixTarget(gyroI.get_heading()));
		pros::delay(3);
	}
	pros::lcd::print(2,"Auton Selected");
}
