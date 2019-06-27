#pragma once
#include "main.h"
#include <array>
#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>

class timers
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

typedef enum Select
{
	NEGATIVE = -1,
	UNDEFINED = 0,
	REDFLAG = 1,
	BLUEFLAG = 2,
	REDBACKPARK = 3,
	BLUEBACKPARK = 4,
	REDBACK = 5,
	BLUEBACK = 6,
	REDDOUBLEBACK = 7,
	BLUEDOUBLEBACK = 9,
	SKILLZ = 8,
	HOLDER2 = 10
}Select;

extern bool autonTest;
extern const bool voltage;
extern const bool gyroUpsidedown;
extern pros::ADIGyro gyro;

extern std::array<pros::Motor, 3> leftDrive;
extern std::array<pros::Motor, 3> rightDrive;

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

