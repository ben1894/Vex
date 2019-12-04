#pragma once
#include "main.h"
#include "forwardDeclairations.hpp"

void smallRed()
{
    gyro.reset();
    tilter.tare_position();
all( //ignore error, it compiles
    DRIVE,FORWARDS,1500,0,MMREGPID,60,127,
    DRIVE,FORWARDS,1000,0,NOPID,60,
    DRIVE,TURN,3000,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,3000,3000,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,1200,0,MMREGPID,60,127,
    DRIVE,FORWARDS,1300,0,NOPID,60,
    DRIVE,TURN,1400,NOSTRAIGHT,TURNPID
    );

    /*
        DRIVE,UPLEFTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127,
    DRIVE,DOWNRIGHTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127,
    DRIVE,DOWNLEFTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127,
    DRIVE,UPRIGHTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127
    gyro.reset();
    tilter.tare_position();
all(DRIVE,FORWARDS,1300,NOSTRAIGHT,NOPID,60,
    DRIVE,FORWARDS,450,NOSTRAIGHT,NOPID,60,TIMET,600,
    DRIVE,FORWARDS,450,NOSTRAIGHT,NOPID,60,TIMET,600,
    DRIVE,FORWARDS,450,NOSTRAIGHT,NOPID,60,TIMET,600,
    DRIVE,FORWARDS,580,NOSTRAIGHT,NOPID,60,TIMET,600,
    DRIVE,BACKWARDS,1860,NOSTRAIGHT,TIMET,100,
    DRIVE,TURN,1210,NOSTRAIGHT,TURNPID,
    INTAKE,OUT,127,
    INTAKE,IN,127,DRIVET,1,230);
    driveMotorsSpeed(110,leftDrive);
    driveMotorsSpeed(110,rightDrive);
    pros::delay(520);
    driveMotorsSpeed(0,leftDrive);   /////////////////////red small
    driveMotorsSpeed(0,rightDrive);
    pros::delay(500);
    motorGroupMove(-100,intakeM);
    pros::delay(550);
    motorGroupMove(0,intakeM);
    all(TILTER,POSITION,7100,100);
    pros::delay(2000);
    driveMotorsSpeed(-50,leftDrive);
    driveMotorsSpeed(-50,rightDrive);
    pros::delay(1500);
    driveMotorsSpeed(0,leftDrive);
    driveMotorsSpeed(0,rightDrive);
    */
}
