#pragma once
#include "pidPacks.hpp"

template <size_t N>
void driveMotorsSpeed(int speed, std::array<pros::Motor, N> &motorArray, bool forceRPM = false)
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
        int RPMSpeed = map(speed,0,127,0,getMaxSpeed(motorArray[0]));
        for(int i = 0; i < motorArray.size(); i++)
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