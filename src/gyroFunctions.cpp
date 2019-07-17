#include "main.h"
#include "forwardDeclairations.hpp"

int actualGyroPosition()
{
	int fixingGyro = gyro.get_value();

	if(gyroUpsidedown == true)
	{
		fixingGyro = fixingGyro * -1;
	}
	return fixTarget(fixingGyro);
}

 int fixTarget(int oldTarget)
 {
 		if(oldTarget < 0)
 	{
 		return ((oldTarget%3600) + 3600);
   }
   else if(oldTarget > 3600)
   { 
   	return (oldTarget%3600);
   }
   else
   {
   	return oldTarget;
   }
 }

 void getDistances(Both &Val, int target)
 {
 	fixTarget(target);
 	int current = actualGyroPosition();
 	int R;
 	int L;
 		if(current > target)
 	{
 		L = (current - target);
 		R = (3600 - L);
 	}
 	else if(current <= target)
 	{
 		R = (target - current);
 		L = (3600 - R);
 	}
  Val.Right = R;
  Val.Left  = L;
 }