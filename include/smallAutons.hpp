#pragma once
#include "main.h"
#include "forwardDeclairations.hpp"

void smallBlue()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1500,127,
    LIFT,SPEED,-127,TIMETE,600,
    LIFT,POSITION,410,127,DRIVET,3,450,
    LIFT,SPEED,-127,TIMETE,600,DRIVET,5,0,
    DRIVE,FORWARDS,-1,NOSTRAIGHT,NOBRAKE,LIFTT,2,500,
    DRIVE,FORWARDS,430,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,540,0,NOPID,46,
    DRIVE,FORWARDS,200,0,NOPID,35,LIFTT,3,400,
    DRIVE,BACKWARDS,200,0,MMREGPID,20,46,
    DRIVE,DOWNLEFTSWEEP,350,510,10,MMREGPID,127,127,REGACCEL,NOBRAKE,
    DRIVE,DOWNRIGHTSWEEP,350,0,10,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,940,0,NOPID,48,
    DRIVE,BACKWARDS,460,0,NOPID,70,
    DRIVE,TURN,2280,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,340,2220,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,55,2220,TIMETE,450,
    INTAKE,OUT,127,LIFTT,1,1500,
    INTAKE,IN,127,TIMET,700,
    INTAKE,IN,45,DRIVET,10,15,
    TILTER,POSITION,2000,127,DRIVET,10,10
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
}

void smallRed()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1400,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,-1,NOSTRAIGHT,LIFTT,2,500,NOBRAKE,
    DRIVE,FORWARDS,380,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,550,0,MMREGPID,regDriveMin,45,
    DRIVE,FORWARDS,150,0,NOPID,35,TIMET,300,
    DRIVE,TURN,3150,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1080,3150,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,940,0,NOPID,48,
    DRIVE,BACKWARDS,460,0,
    DRIVE,TURN,1320,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,340,1380,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,55,1380,TIMETE,450,
    INTAKE,OUT,127,LIFTT,1,1500,
    INTAKE,IN,127,TIMET,700,
    INTAKE,IN,45,DRIVET,10,15,
    TILTER,POSITION,2000,127,DRIVET,10,10
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
}



/*    resetAutonVals();
addCommands(
    LIFT,POSITION,1400,127,
    LIFT,SPEED,-127,TIMETE,600,
    LIFT,POSITION,990,127,DRIVET,3,530,
    LIFT,SPEED,-127,TIMETE,600,DRIVET,5,0,
    DRIVE,FORWARDS,-1,NOSTRAIGHT,LIFTT,2,500,NOBRAKE,
    DRIVE,FORWARDS,430,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,490,0,NOPID,48,

    DRIVE,FORWARDS,240,0,NOPID,33,LIFTT,3,890,REGACCEL,
    DRIVE,BACKWARDS,240,0,NOPID,48,REGACCEL,
    DRIVE,FORWARDS,240,0,NOPID,48,REGACCEL,

    DRIVE,TURN,3160,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1160,3160,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,940,0,NOPID,48,
    DRIVE,BACKWARDS,460,0,MMREGPID,70,127,
    DRIVE,TURN,1320,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,340,1380,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,55,1380,TIMETE,450,
    INTAKE,OUT,127,LIFTT,1,1500,
    INTAKE,IN,127,TIMET,700,
    INTAKE,IN,45,DRIVET,12,15,
    TILTER,POSITION,2000,127,DRIVET,12,10
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
);*/