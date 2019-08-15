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
	CUSTOMPID,
	NOPID,
	REGPID,
	DRIVEPID,
	LIFTPID,
	MMREGPID,
	MMDRIVEPID,
	MMLIFTPID,

	TIMET,
	DRIVET,
	LIFTT,
	NONET,

	BACKWARDS,
	FORWARDS,
	TURN,
	SWEEP,

	NOACCEL
};

enum SystemStates
{
	WAITINGFORINSTRUCTIONS,
	WAITINGFORTRIGGER,
	END,
	EXECUTINGINSTRUCTIONS
};

enum Ids
{
	DRIVE = 999999990,
	LIFT = 999999998,
	CLAW = 999999997,
	INNER = 999999996,
	NULLOPTION = 999999995
};

const int minEnumValue((int)NULLOPTION-1);

extern bool autonTest;
extern const bool voltage;
extern const bool gyroUpsidedown;
extern pros::ADIGyro gyro;

extern std::array<pros::Motor, 2> leftDrive;
extern std::array<pros::Motor, 2> rightDrive;

extern pros::Motor hMotor;
extern pros::Motor descore;
extern pros::Motor intake;
extern pros::Motor catapult;
extern pros::Motor flipper;
extern pros::Motor parkM;

extern pros::Controller secondaryController;
extern pros::Controller mainController;
extern pros::ADIEncoder forwardMiddle;
extern pros::ADIEncoder forwardLeft;
extern pros::ADIEncoder forwardRight;

extern pros::ADIGyro gyro;

typedef struct {
	int Left;
	int Right;
} Both;

extern int getMaxSpeed(pros::Motor &motor);
extern void correctedMotorSpeed(int speed, pros::Motor &motor, bool forceRPM = false);

extern void rightSide(int speed, bool forceRPM = false);
extern void leftSide(int speed, bool forceRPM = false);

extern int actualGyroPosition();
extern int fixTarget(int oldTarget);
extern void getDistances(Both &Val, int target);

extern float map(float value, float istart, float istop, float ostart, float ostop);
extern int cVal(pros::controller_digital_e_t button);
extern int cVal(pros::controller_analog_e_t button);
extern int actualGyroPosition();
extern int fixTarget(int oldTarget);

#include "templateMotorFunctions.hpp"
