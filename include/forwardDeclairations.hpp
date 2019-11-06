#pragma once
#include "main.h"
#include <array>
#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>

class Timer
{
private:
	int resetVal = 0;
public:
	int current()
	{
		return pros::millis() - resetVal;
	}
	void clear()
	{
		resetVal = pros::millis();
	}
};

//inside wheel Speed = 2pir
enum AutonFlags
{
	WHEELCORRECTION = -3,
	NOSTRAIGHT = -2,
	CURRENTVAL = -1,

	CUSTOMPID,
	NOPID,
	REGPID,
	DRIVEPID,
	TILTERPID,
	INTAKEPID,
	MMREGPID,
	MMDRIVEPID,
	MMTILTERPID,

	IN,
	OUT,

	POSITION,

	LEFTTURN,
	RIGHTTURN,

	TIMET,
	DRIVET,
	TILTERT,
	INTAKET,
	NONET,

	BACKWARDS,
	FORWARDS,
	TURN,
	ENCODERTURN,
	UPLEFTSWEEP,
	DOWNLEFTSWEEP,
	UPRIGHTSWEEP,
	DOWNRIGHTSWEEP,

	NOACCEL,
	BLANK
};

enum SystemStates
{
	WAITINGFORINSTRUCTIONS,
	WAITINGFORTRIGGER,
	END,
	EXECUTINGINSTRUCTIONS
};

enum Select
{
	UNDER,
	UNDEFINED,
	AUTON1,
	AUTON2,
	OVER
};

enum Ids
{
	DRIVE = 999999999,
	TILTER = 999999998,
	CLAW = 999999997,
	INNER = 999999996,
	INTAKE =999999995,
	NULLOPTION = 999999994
};

const int minEnumValue((int)NULLOPTION-1);

extern bool autonTest;
extern const bool voltage;
extern const bool gyroUpsidedown;
extern const double wheelDistance;
extern const int driveBaseSpeed;
extern const int gyroTurnBaseSpeed;
extern const int encoderTurnBaseSpeed;
extern pros::ADIGyro gyro;

extern std::array<pros::Motor, 2> leftDrive;
extern std::array<pros::Motor, 2> rightDrive;
extern std::array<pros::Motor, 2> intake;

extern pros::Motor tilter;
extern pros::Motor lift;

extern pros::Controller secondaryController;
extern pros::Controller mainController;

extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder rightEncoder;
extern pros::ADIGyro gyro;

struct GyroDistances
{
	int Left;
	int Right;
};

extern int getMaxSpeed(pros::Motor &motor);
extern void correctedMotorSpeed(int speed, pros::Motor &motor, bool forceRPM = false);

extern int actualGyroPosition();
extern int fixTarget(int oldTarget);
extern void getDistances(GyroDistances &Val, int target);

extern float map(float value, float istart, float istop, float ostart, float ostop);
extern int cVal(pros::controller_digital_e_t button);
extern int cVal(pros::controller_analog_e_t button);
extern int actualGyroPosition();
extern int fixTarget(int oldTarget);

#include "templateMotorFunctions.hpp"
