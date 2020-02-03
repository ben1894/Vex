#pragma once
#include "main.h"
#include <array>
#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>

#define brakeTimeTurn 50
#define brakeTimeDrive 50
#define brakeSpeedTurn 120
#define brakeSpeedDrive 120
#define pi 3.1415926535897932384

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

/*
typedef void (* vFunctionCall)(int args);
void funct(int a, vFunctionCall cVal)
{
   funct2(a);
}
*/
class Button
{
	pros::controller_digital_e_t trackingButton;
	Button(pros::controller_digital_e_t button)
	{

	}
	Button(pros::controller_analog_e_t button)
	{

	}
};


//all uses documented in smallRedA.hpp
enum AutonFlags
{
	PAST = -100000,
	WHEELCORRECTION = -3,
	NOSTRAIGHT = -2,
	CURRENTVAL = -1,

	CUSTOMPID,
	NOPID,
	TURNPID,
	TURNPID2,
	REGPID,
	DRIVEPID,
	TILTERPID,
	INTAKEPID,
	LIFTPID,
	MMREGPID,
	MMDRIVEPID,
	MMTILTERPID,

	IN,
	OUT,

	POSITION,
	SPEED,

	LEFTTURN,
	RIGHTTURN,

	TIMET,
	DRIVET,
	TILTERT,
	INTAKET,
	LIFTT,
	NONET,

	NONETE,
	TIMETE,
	DRIVETE,
	TILTERTE,
	INTAKETE,
	LIFTTE,

	BACKWARDS,
	FORWARDS,
	TURN,
	ENCODERTURNLEFT,
	ENCODERTURNRIGHT,
	UPLEFTSWEEP,
	DOWNLEFTSWEEP,
	UPRIGHTSWEEP,
	DOWNRIGHTSWEEP,

	BACKWARDSE,
	FORWARDSE,
	UPLEFTSWEEPE, //Different setup for when it has an ending trigger
	DOWNLEFTSWEEPE,
	UPRIGHTSWEEPE,
	DOWNRIGHTSWEEPE,

	FORWARDSC,
	BACKWARDSC,
	UPLEFTSWEEPC, //Different setup for when it has an ending trigger
	DOWNLEFTSWEEPC,
	UPRIGHTSWEEPC,
	DOWNRIGHTSWEEPC,
	TURNC,

	NOACCEL,
	ACCEL,
	REGACCEL,
	NOBRAKE,
	INVERSE,
	BLANK
};

enum SystemStates
{
	WAITINGFORINSTRUCTIONS,
	WAITINGFORTRIGGER,
	END,
	EXECUTINGINSTRUCTIONS,
};

enum Select
{
	UNDER,
	UNDEFINED,
	SMALLRED,
	SMALLBLUE,
	THICCRED,
	THICCBLUE,
	MICROCUBE,
	OVER
};

enum Ids
{
	DRIVE = 999999999,
	TILTER = 999999998,
	CLAW = 999999997,
	LIFT = 999999996,
	INTAKE =999999995,

	NULLOPTION = 999999994
};

const int minEnumValue((int)NULLOPTION-2);

extern bool autonTest;
extern double gyroZero;
extern const bool voltage;
extern const bool gyroUpsidedown;
extern const double wheelDistance;
extern const int driveBaseSpeed;
extern const int gyroTurnBaseSpeed;
extern const int encoderTurnBaseSpeed;

extern std::array<pros::Motor, 2> leftDrive;
extern std::array<pros::Motor, 2> rightDrive;
extern std::array<pros::Motor, 2> intakeM;

extern pros::Motor tilter;
extern pros::Motor lift;

extern pros::Controller secondaryController;
extern pros::Controller mainController;
extern Select count;

extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder rightEncoder;
extern pros::Imu gyroI;
extern Timer autonTimer;


struct GyroDistances
{
	double Left;
	double Right;
};

extern int getMaxSpeed(pros::Motor &motor);
extern void correctedMotorSpeed(int speed, pros::Motor &motor, bool forceRPM = false);

extern double actualGyroPosition();
extern void resetGyro();
extern double fixTarget(double oldTarget);
extern void getDistances(GyroDistances &Val, double target);

extern void posTest();
extern int getDriveEncoder();
extern float map(float value, float istart, float istop, float ostart, float ostop);
extern double radToDeg(double radians);
extern double degToRad(double degrees);
extern double correctAtan(double y, double x);
extern int cVal(pros::controller_digital_e_t button);
extern int cVal(pros::controller_analog_e_t button);
extern void resetAutonVals();
template <typename Ts> void addCommands(Ts);

#include "templateMotorFunctions.hpp"