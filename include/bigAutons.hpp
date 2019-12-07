#pragma once
#include "main.h"
#include "forwardDeclairations.hpp"

//All Auton Options
/*
Drive System Call
    <DRIVE>,Direction(FORWARDS,BACKWARDS,TURN,UPLEFTSWEEP,UPRIGHTSWEEP,DOWNLEFTSWEEP,DOWNRIGHTSWEEP),Distance(0+),DriveStraight(0-3600,NOSTRAIGHT,WHEELCORRECT,CURRENTVAL)
    *<DRIVE>,Direction(FORWARDSE,BACKWARDSE,UPLEFTSWEEPE,UPRIGHTSWEEPE,DOWNLEFTSWEEPE,DOWNRIGHTSWEEPE),Speed(0-127),DriveStraight(0-3600,NOSTRAIGHT,WHEELCORRECT,CURRENTVAL)
    NOACCEL

Intake System Call
    <INTAKE>,direction(IN,OUT),Speed 

Tilter System Call
    <TILTER>,POSITION,position(0-7100)
    *<TILTER>,SPEED,Speed(0-127)

Starting Trigger
    StartingTrigger(TIMET),TimeToWait(0milliseconds+)
    StartingTrigger(DRIVET,INTAKET,TILTERTE),NumberCallOfOtherSystem(1+),ValueToStartAt(0+)

Ending Trigger
    EndingTrigger(TIMETE),TimeToWait(0milliseconds+)
    EndingTrigger(DRIVETE,INTAKETE,TILTERTE),NumberCallOfOtherSystem(1+),ValueToGoTill(0+)

Other Options
    REGPID
    TURNPID
    TILTERPID
    DRIVEPID
    INTAKEPID
    NOPID,speed(0-127)
    MMPID,minumum(0-127),maximum(0-127)
    CUSTOMPID,P,I,D,minimum,maximum

*Needs an ending trigger
*/

//Other Notes
/*
All commands go in order of call
*/

void bigRed()
{
    autonTimer.clear();
    gyro.reset();
    tilter.tare_position();
addCommands(
    DRIVE,FORWARDS,900,0,TIMET,450,MMREGPID,50,127,ACCEL,
    DRIVE,FORWARDS,1900,0,NOPID,45,
    DRIVE,TURN,3330,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,2700,NOSTRAIGHT,ACCEL,
    DRIVE,TURN,3570,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,550,3570,MMREGPID,60,127,ACCEL,
    DRIVE,FORWARDS,2540,3570,NOPID,45,
    DRIVE,TURN,1480,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,2880,1480,ACCEL,
    INTAKE,OUT,127,
    INTAKE,IN,127,TIMET,200,
    INTAKE,IN,50,DRIVET,8,600,
    INTAKE,IN,0,DRIVET,9,2700

    //INTAKE,OUT,50,DRIVET,9,2300,
    //INTAKE,IN,0,TIMET,230
    );
addCommands(
    TILTER,POSITION,6000,127,
    TILTER,POSITION,7100,100
    );
addCommands(
    INTAKE,OUT,80,
    INTAKE,OUT,0,TIMET,350,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,INTAKET,3,200,NOPID,60
);
}

void bigBlue()
{
    autonTimer.clear();
    gyro.reset();
    tilter.tare_position();
addCommands(
    DRIVE,FORWARDS,900,0,TIMET,450,MMREGPID,50,127,ACCEL,
    DRIVE,FORWARDS,1900,0,NOPID,45,
    DRIVE,TURN,270,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,2700,NOSTRAIGHT,ACCEL,
    DRIVE,TURN,30,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,550,30,MMREGPID,60,127,ACCEL,
    DRIVE,FORWARDS,2500,30,NOPID,45,
    DRIVE,TURN,2100,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,2780,2100,ACCEL,
    INTAKE,OUT,127,
    INTAKE,IN,127,TIMET,200,
    INTAKE,IN,50,DRIVET,8,3000,
    INTAKE,IN,0,DRIVET,9,2700
    //INTAKE,OUT,50,DRIVET,9,2300,
    //INTAKE,IN,0,TIMET,230
    );
addCommands(
    TILTER,POSITION,6000,127,
    TILTER,POSITION,7100,100
    );
addCommands(
    INTAKE,OUT,80,
    INTAKE,OUT,0,TIMET,350,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,INTAKET,3,200,NOPID,60
);}





/* //values with free rolling encoders
    gyro.reset();
    tilter.tare_position();
addCommands(
    DRIVE,FORWARDS,500,0,MMREGPID,60,127,
    DRIVE,FORWARDS,1000,0,NOPID,60,
    DRIVE,TURN,3000,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,700,NOSTRAIGHT,
    DRIVE,TURN,1,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,400,NOSTRAIGHT,MMREGPID,60,127,
    DRIVE,FORWARDS,900,NOSTRAIGHT,NOPID,60,
    DRIVE,TURN,1500,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,1300,NOSTRAIGHT,
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
      /*  gyro.reset();
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
    );*/
    //DRIVETE,3,1300
    /*
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
