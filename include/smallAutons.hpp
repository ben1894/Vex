#pragma once
#include "main.h"
#include "forwardDeclairations.hpp"

void smallBlue()
{

}

void smallRed()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1400,127,
    LIFT,SPEED,-127,TIMETE,600,
    LIFT,POSITION,1000,127,DRIVET,3,530,
    LIFT,SPEED,-127,TIMETE,600,DRIVET,5,0,
    DRIVE,FORWARDS,-1,NOSTRAIGHT,LIFTT,2,500,NOBRAKE,
    DRIVE,FORWARDS,430,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,700,
    DRIVE,FORWARDS,490,0,NOPID,48,

    DRIVE,FORWARDS,200,0,NOPID,40,LIFTT,3,900,REGACCEL,
    DRIVE,BACKWARDS,200,0,NOPID,48,REGACCEL,
    DRIVE,FORWARDS,200,0,NOPID,48,REGACCEL,

    DRIVE,TURN,3120,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1090,3120,REGACCEL,
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
);
}