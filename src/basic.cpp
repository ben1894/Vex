#include "forwardDeclairations.hpp"

void driveMotorsSpeed(int speed, pros::Motor &motor, bool forceRPM)
{
	if(voltage == true && forceRPM == false)
	{
	 motor.move(speed);
 	}
 	else
 	{ 
     int RPMSpeed = map(speed,0,127,0,getMaxSpeed(motor));
	 motor.move_velocity(RPMSpeed);
	}
}

int getMaxSpeed(pros::Motor &motor)
{
	if(motor.get_gearing() == pros::E_MOTOR_GEARSET_36)
	{
		return 100;
	}
	else if(motor.get_gearing() == pros::E_MOTOR_GEARSET_18)
	{
		return 200;
	}
	else 
	{
		return 600;
	}
}

float map(float value, float istart, float istop, float ostart, float ostop)
{
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

int cVal(pros::controller_digital_e_t button)
{
	return mainController.get_digital(button);
}
int cVal( pros::controller_analog_e_t button)
{
	return mainController.get_analog(button);
}

void resetAutonVals()
{
	tilter.tare_position();
	lift.tare_position();
	gyro.reset();
	autonTimer.clear();
}