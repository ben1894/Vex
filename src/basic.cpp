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

template <size_t N>
void driveMotorsSpeed(int speed, std::array<pros::Motor, N> &motorArray, bool forceRPM)
{
    if(voltage == true && forceRPM == false)
    {
        for(int i = 0; i < motorArray.size(); i++)
        {
            motorArray[i].move(speed);
        }
    }
    else
    { 
        int RPMSpeed = map(speed,0,127,0,getMaxSpeed(motor[0]));
        for(int i = 0; i < rightmotorArray.size(); i++)
        {
            motorArray[i].move_velocity(RPMSpeed);
        }
    }
}

template <size_t N>
void motorGroupMove(int speed, std::array<pros::Motor, N> &motorArray)
{
    for(int i = 0; i < motorArray.size(); i++)
    {
        motorArray[i].move(speed);
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