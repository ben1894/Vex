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
    LIFT,POSITION,1600,127,
    LIFT,SPEED,-127,TIMETE,850,
    DRIVE,FORWARDS,-1,NOSTRAIGHT,NOBRAKE,LIFTT,2,500,
    DRIVE,FORWARDS,400,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,450,0,NOPID,46,
    DRIVE,TURN,3030,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,950,3030,REGACCEL,
    DRIVE,TURN,3587,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,3587,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,670,3587,NOPID,53,NOBRAKE, //940
    DRIVE,UPLEFTSWEEP,640,2850,NOSTRAIGHT,NOPID,62,
    DRIVE,BACKWARDS,700,3400,
    DRIVE,TURN,1520,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,400,1580,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,50,1580,TIMETE,350,
    INTAKE,OUT,127,LIFTT,1,1500,
    INTAKE,IN,127,TIMET,700,
    INTAKE,IN,20,DRIVET,10,500,
    INTAKE,OUT,60,
    INTAKE,IN,20,TIMET,460,
    TILTER,SPEED,-127,DRIVET,2,20,TIMETE,300,
    TILTER,POSITION,2750,127,DRIVET,10,200,
    TILTER,POSITION,1900,127,DRIVET,12,0
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,45,TILTERT,1,1900,
    INTAKE,OUT,0,TILTERT,1,3100,
    TILTER,POSITION,6300,127,
    TILTER,POSITION,6800,90
    );
addCommands(
    INTAKE,OUT,63,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,TIMET,340,
    TILTER,POSITION,5000,127,TIMET,500
);
}

void bigBlue()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1600,127,
    LIFT,SPEED,-127,TIMETE,850,
    DRIVE,FORWARDS,-1,NOSTRAIGHT,NOBRAKE,LIFTT,2,500,
    DRIVE,FORWARDS,400,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,450,0,NOPID,46,
    DRIVE,TURN,551,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,920,551,REGACCEL,
    DRIVE,TURN,15,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,15,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,670,15,NOPID,53,NOBRAKE, //940
    DRIVE,UPRIGHTSWEEP,640,640,NOSTRAIGHT,NOPID,62,
    DRIVE,BACKWARDS,700,200,
    DRIVE,TURN,2150,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,400,2090,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,50,2090,TIMETE,350,
    INTAKE,OUT,127,LIFTT,1,1500,
    INTAKE,IN,127,TIMET,700,
    INTAKE,IN,20,DRIVET,10,500,
    INTAKE,OUT,60,
    INTAKE,IN,20,TIMET,460,
    TILTER,SPEED,-127,DRIVET,2,20,TIMETE,300,
    TILTER,POSITION,2750,127,DRIVET,10,200,
    TILTER,POSITION,1900,127,DRIVET,12,0
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,45,TILTERT,1,1900,
    INTAKE,OUT,0,TILTERT,1,3100,
    TILTER,POSITION,6300,127,
    TILTER,POSITION,6800,90
    );
addCommands(
    INTAKE,OUT,63,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,TIMET,340,
    TILTER,POSITION,5000,127,TIMET,500
);
}






















//good
/*
    resetAutonVals();
addCommands(
    LIFT,POSITION,1400,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,-1,NOBRAKE,LIFTT,2,500,
    DRIVE,FORWARDS,430,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,600,0,NOPID,48,
    DRIVE,TURN,505,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1030,505,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,940,0,NOPID,48,
    DRIVE,BACKWARDS,460,0,
    DRIVE,TURN,2280,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,340,2220,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,55,2220,TIMETE,450,
    INTAKE,OUT,127,LIFTT,1,1500,
    INTAKE,IN,127,TIMET,700,
    INTAKE,IN,45,DRIVET,9,15,
    TILTER,POSITION,2000,127,DRIVET,9,10
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,43,TILTERT,1,2000,
    INTAKE,OUT,0,TILTERT,1,4200,
    TILTER,POSITION,6700,127,
    TILTER,POSITION,7060,90
    );
addCommands(
    INTAKE,OUT,65,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,TIMET,280,
    TILTER,POSITION,5000,127,TIMET,50
);
*/