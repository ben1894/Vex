#include "main.h"
#include "forwardDeclairations.hpp"

double actualGyroPosition()
{
	double fixingGyro = gyroI.get_heading(); //corrects for negative values

	if(gyroUpsidedown == true)
	{
		fixingGyro *= -1.0;
	}
	
	return fixTarget(fixingGyro);
}

void resetGyro()
{
	gyroZero = actualGyroPosition();
}

double fixTarget(double oldTarget)
{
	if(oldTarget < 0)
	{
		return (fmod(oldTarget, 360) + 360);
	}
	else if(oldTarget > 360)
	{ 
		return fmod(oldTarget, 360);
	}
	else
	{
		return oldTarget;
	}
}

void getDistances(GyroDistances &Val, double target)
{
	fixTarget(target);
	double current = actualGyroPosition();
	double R;
	double L;
	if(current > target)
	{
		L = (current - target);
		R = (360 - L);
	}
	else if(current <= target)
	{
		R = (target - current);
		L = (360 - R);
	}
	Val.Right = R;
	Val.Left  = L;
}