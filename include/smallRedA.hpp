#pragma once
#include "main.h"
#include "forwardDeclairations.hpp"

//here is basically everything you may need to know... i haven't documented everything yet...
/*
DRIVE,DIRECTION,distance,driveStraight{GyroVal,NOSTRAIGHT}
INTAKE,IN/OUT,speed
TILTER,POSITION,position,speed
trigger{INTAKET,TILTERT,DRIVET},numberCallToReference,valueToStart
trigger{TIMET},timeToWait
endingTrigger{TIMETE},timeToMove
endingTrigger{DRIVETE,INTAKETE,TILTERTE},numberCallToReference,valueToEnd
MMPID,min,max
NOPID,speed
*/
//all the commands go in order of call
//to add a delay before the start of a command add this TIMET,100, for a 100 millisecond delay
//also make sure to go save all and then build all before you upload
//Finally, only make commits after you've made a lot of changes and try to label the appropriately :P
void smallRed()
{
    /*
    gyro.reset();
    tilter.tare_position();
addCommands(
    DRIVE,FORWARDS,500,0,MMREGPID,60,127,
    DRIVE,FORWARDS,1000,0,NOPID,60,
    DRIVE,TURN,3000,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,3000,3000,
    DRIVE,TURN,10,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,1200,0,MMREGPID,60,127,
    DRIVE,FORWARDS,1300,0,NOPID,60,
    DRIVE,TURN,2000,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,3500,2000,
    INTAKE,OUT,127,TIMETE,500,
    INTAKE,IN,127
    );
addCommands(
    INTAKE,OUT,100,TIMETE,550,
    TILTER,POSITION,7100,100,TIMET,500
    );
addCommands(
    DRIVE,BACKWARDS,500,NOSTRAIGHT,TIMET,500
    );
    */
    gyro.reset();
    tilter.tare_position();
addCommands(
    DRIVE,TURN,3000,NOSTRAIGHT,TURNPID
    );
addCommands(
    DRIVE,TURN,10,NOSTRAIGHT,TURNPID
    );
addCommands(
    DRIVE,TURN,3000,NOSTRAIGHT,TURNPID,
    DRIVE,TURN,10,NOSTRAIGHT,TURNPID
    );
}


    //DRIVETE,3,1300
    /*
        DRIVE,UPLEFTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127,
    DRIVE,DOWNRIGHTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127,
    DRIVE,DOWNLEFTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127,
    DRIVE,UPRIGHTSWEEP,10000,300,NOSTRAIGHT,MMREGPID,40,127
    gyro.reset();
    tilter.tare_position();
addCommands(DRIVE,FORWARDS,1300,NOSTRAIGHT,NOPID,60,
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
    addCommands(TILTER,POSITION,7100,100);
    pros::delay(2000);
    driveMotorsSpeed(-50,leftDrive);
    driveMotorsSpeed(-50,rightDrive);
    pros::delay(1500);
    driveMotorsSpeed(0,leftDrive);
    driveMotorsSpeed(0,rightDrive);
    */
