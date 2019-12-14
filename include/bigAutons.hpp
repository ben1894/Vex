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
    LIFT,SPEED,127,TIMET,200,TIMETE,500,
    LIFT,SPEED,-127,TIMETE,500,
    DRIVE,FORWARDS,390,0,REGACCEL,TIMET,300,
    DRIVE,FORWARDS,460,0,MMREGPID,60,127,REGACCEL,NOBRAKE,TIMET,500,
    DRIVE,FORWARDS,1900,0,NOPID,53,
    DRIVE,TURN,3280,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,2550,NOSTRAIGHT,REGACCEL,
    DRIVE,TURN,5,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,500,5,MMREGPID,60,127,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,2100,5,NOPID,50,
    DRIVE,TURN,1470,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,2420,NOSTRAIGHT,REGACCEL,TIMET,300,
    INTAKE,OUT,127,
    INTAKE,IN,127,TIMET,200,
    INTAKE,IN,50,DRIVET,9,600,
    INTAKE,OUT,80,DRIVET,10,2380
    );
addCommands(
    INTAKE,OUT,0,
    TILTER,POSITION,6600,127,
    TILTER,POSITION,7100,100
    );
addCommands(
    INTAKE,OUT,127,
    INTAKE,IN,0,TIMET,500,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,INTAKET,1,290,NOPID,60,
    TILTER,POSITION,5000,127,INTAKET,1,30
);
}

void bigBlue()
{
    resetAutonVals();
addCommands(
    LIFT,SPEED,127,TIMET,200,TIMETE,500,
    LIFT,SPEED,-127,TIMETE,500,
    DRIVE,FORWARDS,390,0,REGACCEL,TIMET,300,
    DRIVE,FORWARDS,460,0,MMREGPID,60,127,REGACCEL,NOBRAKE,TIMET,500,
    DRIVE,FORWARDS,1900,0,NOPID,53,
    DRIVE,TURN,320,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,2550,NOSTRAIGHT,REGACCEL,
    DRIVE,TURN,3595,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,500,5,MMREGPID,60,127,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,2000,5,NOPID,49,
    DRIVE,TURN,2130,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,2340,NOSTRAIGHT,REGACCEL,TIMET,300,
    INTAKE,OUT,127,
    INTAKE,IN,127,TIMET,200,
    INTAKE,IN,50,DRIVET,9,600,
    INTAKE,OUT,80,DRIVET,10,2380
    );
addCommands(
    INTAKE,OUT,0,
    TILTER,POSITION,6600,127,
    TILTER,POSITION,7100,100
    );
addCommands(
    INTAKE,OUT,127,
    INTAKE,IN,0,TIMET,500,
    DRIVE,BACKWARDS,500,NOSTRAIGHT,INTAKET,1,290,NOPID,60,
    TILTER,POSITION,5000,127,INTAKET,1,30
);
}

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