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

double degToRad(double degrees)
{
	return (degrees*pi) / 180;
}

double radToDeg(double radians)
{
	return (radians*180) / pi;
}

void resetAutonVals()
{
	tilter.tare_position();
	lift.tare_position();
	resetGyro();
	autonTimer.clear();
}

double correctAtan(double y, double x)
{
	y = degToRad(y);
	x = degToRad(x);
	return radToDeg(atan2(y,x));
}

/*
double sind(double x) {
  if (!isfinite(x)) {
    return sin(x);
  }
  if (x < 0.0) {
    return -sind(-x);
  }
  
  int quo;
  double x90 = remquo(fabs(x), 90.0, &quo);
  switch (quo % 4) {
    case 0:
      // Use * 1.0 to avoid -0.0
      return sin(degToRad(x90) * 1.0);
    case 1:
      return cos(degToRad(x90));
    case 2:
      return sin(degToRad(-x90) * 1.0);
    case 3:
      return -cos(degToRad(x90));
  }
  return 0.0;
}*/
/*
double cosd(double x)
{
	return sind(90.0 - x);
}*/
//sin (90° – x) = cos x	cos (90° – x) = sin x

