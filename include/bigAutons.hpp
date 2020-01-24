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

Lift System Call
    <LIFT>,POSITION,position(0-7100)
    *<LIFT>,SPEED,Speed(0-127)

Starting Trigger
    StartingTrigger(TIMET),TimeToWait(0milliseconds+)
    StartingTrigger(DRIVET,INTAKET,TILTERTE,LIFTT),NumberCallOfOtherSystem(1+),ValueToStartAt(0+)

Ending Trigger
    EndingTrigger(TIMETE),TimeToWait(0milliseconds+)
    EndingTrigger(DRIVETE,INTAKETE,TILTERTE,LIFTTE),NumberCallOfOtherSystem(1+),ValueToGoTill(0+)

Other Options
    REGPID
    TURNPID
    TILTERPID
    DRIVEPID
    INTAKEPID
    LIFTPID
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
    resetAutonVals();
addCommands(
    LIFT,POSITION,1700,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,250,0,TIMET,600,
    DRIVE,FORWARDS,180,0,MMREGPID,55,127,REGACCEL,NOBRAKE,LIFTT,2,600,//TIMET,1300,
    DRIVE,FORWARDS,850,0,NOPID,44,
    DRIVE,TURN,3230,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1390,3230,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,1180,0,NOPID,44,
    DRIVE,TURN,1450,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,1040,1510,REGACCEL,NOBRAKE,MMREGPID,60,127,
    DRIVE,FORWARDSE,60,1510,TIMETE,500,
    INTAKE,OUT,127,LIFTT,1,800,
    INTAKE,IN,127,TIMET,1100,
    INTAKE,IN,127,DRIVET,8,10000,
    INTAKE,IN,0,TIMET,420,
    INTAKE,IN,48,DRIVET,10,10,
    TILTER,POSITION,1000,90,DRIVET,9,10
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,30,TILTERT,1,2000,
    INTAKE,OUT,0,TILTERT,1,4500,
    TILTER,POSITION,6700,127,
    TILTER,POSITION,7100,90
    );
addCommands(
    INTAKE,OUT,127,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,1000,NOSTRAIGHT,TIMET,170,
    TILTER,POSITION,5000,127,TIMET,50
);
}

void bigBlue()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1700,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,250,0,TIMET,600,
    DRIVE,FORWARDS,180,0,MMREGPID,55,127,REGACCEL,NOBRAKE,LIFTT,2,600,//TIMET,1300,
    DRIVE,FORWARDS,900,0,NOPID,44,
    DRIVE,TURN,365,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1230,365,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,1220,0,NOPID,44,
    DRIVE,TURN,2170,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,1060,2090,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,50,2090,TIMETE,500,
    INTAKE,OUT,127,LIFTT,1,800,
    INTAKE,IN,127,TIMET,1100,
    INTAKE,IN,127,DRIVET,8,10000,
    INTAKE,IN,0,TIMET,420,
    INTAKE,IN,48,DRIVET,10,10,
    TILTER,POSITION,1000,90,DRIVET,9,10
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,30,TILTERT,1,2000,
    INTAKE,OUT,0,TILTERT,1,4500,
    TILTER,POSITION,6700,127,
    TILTER,POSITION,7100,90
    );
addCommands(
    INTAKE,OUT,127,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,1000,NOSTRAIGHT,TIMET,170,
    TILTER,POSITION,5000,127,TIMET,50
);
}