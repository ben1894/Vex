#include "main.h"
#include "forwardDeclairations.hpp"

int selected = false;

const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;

Select count = UNDEFINED;


pros::Controller mainController(pros::E_CONTROLLER_MASTER);
pros::Controller secondaryController(pros::E_CONTROLLER_PARTNER);

/*
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

std::array<pros::Motor, 2> rightDrive{pros::Motor(1),pros::Motor(2)};
std::array<pros::Motor, 2>  leftDrive{pros::Motor(3),pros::Motor(4)};

void initialize() 
{
	pros::lcd::initialize();

	pros::ADIGyro gyro(2);
	pros::Task::delay(2000);
	for(int i = 0; i < leftDrive.size(); i++)
	{
		leftDrive[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		leftDrive[i].set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
		leftDrive[i].set_gearing(pros::E_MOTOR_GEARSET_18);
	}
		for(int i = 0; i < leftDrive.size(); i++)
	{
		rightDrive[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		rightDrive[i].set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
		rightDrive[i].set_gearing(pros::E_MOTOR_GEARSET_18);
	}

	if(autonTest == true)
	{
		competition_initialize();
	}

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
	 Select oldCount = REDBACK;
	 bool onceAcception = false;
	 int oldLCD = 0;
	 int oldRight = 0;
	 int oldLeft = 0;
	 mainController.clear();
	 pros::Task::delay(50);
	 mainController.print(0,0,"Down To Select");
	 while(!selected)
 	{
		if((pros::lcd::read_buttons() != oldLCD) || (cVal(DIGITAL_RIGHT) != oldRight) || (cVal(DIGITAL_LEFT) != oldLeft))
		{
	 if((pros::lcd::read_buttons() ==  leftButton) || (cVal(DIGITAL_RIGHT)))
	 {
		 //waitForRelease();
		 count = (Select)((int)count + 1);
	 }
	 else if((pros::lcd::read_buttons() == rightButton) || (cVal(DIGITAL_LEFT)))
	 {
		 //waitForRelease();
		 count = (Select)((int)count - 1);
	 }
 	}

	 if((((pros::lcd::read_buttons()) == centerButton)||(cVal(DIGITAL_DOWN))) && (count != UNDEFINED))
	 {
		 selected = true;
	 } //count = negative  old = undefined
   if(count != oldCount)
	 {
	 mainController.clear_line(0);
	 pros::Task::delay(70);
	 switch (count)
	 {
	 case UNDEFINED:
		 pros::lcd::print(0, "< Auto Select >");
		 pros::lcd::print(2,  "Center to Select");
		 mainController.print(0,0,"Down To Select");
		 break;
	 case REDFLAG:
		 pros::lcd::print(0, "Red Flag");
		 mainController.print(0,0,"Red Flag");
		 break;
	 case BLUEFLAG:
		 pros::lcd::print(0, "Blue Flag");
		 mainController.print(0,0,"Blue Flag");
		 break;
	 case REDBACKPARK:
		 pros::lcd::print(0, "Red Back Park");
		 mainController.print(0,0,"Red Back Park");
		 break;
	 case BLUEBACKPARK:
		 pros::lcd::print(0, "Blue Back Park");
		 mainController.print(0,0,"Blue Back Park");
		 break;
	 case REDBACK:
		 pros::lcd::print(0, "Red Back");
		 mainController.print(0,0,"Red Back");
		 break;
	 case BLUEBACK:
		 pros::lcd::print(0, "Blue Back");
		 mainController.print(0,0,"Blue Back");
	 	 break;
	 case BLUEDOUBLEBACK:
		 pros::lcd::print(0, "Blue Double Back");
		 mainController.print(0,0,"Blue DBL Back");
	 	 break;
	 case REDDOUBLEBACK:
		 pros::lcd::print(0, "Red Double Back");
		 mainController.print(0,0,"Red DBL Back");
	 	 break;
	 case SKILLZ:
		 pros::lcd::print(0, "Skillz");
		 mainController.print(0,0,"Skillz");
	 	 break;
	 case NEGATIVE:
		 count = BLUEDOUBLEBACK;
		 onceAcception = true;
		 break;
	 default:
	 	 onceAcception = true;
		 count = UNDEFINED;
		 break;
	 }
 }
 	 oldLCD = pros::lcd::read_buttons();
	 oldRight = cVal(DIGITAL_RIGHT);
	 oldLeft = cVal(DIGITAL_LEFT);
	 if(onceAcception == true)
	 {
		 onceAcception = false;
		 if(count == BLUEDOUBLEBACK)
		 {
			 oldCount = NEGATIVE;
		 }
		 else
		 {
			 oldCount = HOLDER2;
		 }
	 }
	 else
	 {
		 oldCount = count;
	 }
	 pros::delay(2);
 }
 pros::lcd::print(2,"Auton Selected");
 }
