﻿/*
536E Vex Team Code 2019-2020
All code created by:
 ____                       ____
/\  _`\                    /\  _`\
\ \ \L\ \     __    ___    \ \ \/\ \  _ __    __     __    ____
 \ \  _ <   /'__`\/' _ `\   \ \ \ \ \/\`'__\/'__`\ /'__`\ /',__\
  \ \ \L\ \/\  __//\ \/\ \   \ \ \_\ \ \ \//\  __//\  __//\__, `\
   \ \____/\ \____\ \_\ \_\   \ \____/\ \_\\ \____\ \____\/\____/
    \/___/  \/____/\/_/\/_/    \/___/  \/_/ \/____/\/____/\/___/
*/
#include "main.h"
#include "forwardDeclairations.hpp"
#include "pidPacks.hpp"

Timer autonTimer;
bool autonTest = false;
const bool voltage = true;
const bool gyroTurns = true;
const bool gyroUpsidedown = false;
const double wheelDistance = 200;

const int driveBaseSpeed = 10;
const int gyroTurnBaseSpeed = 10;
const int encoderTurnBaseSpeed = 10;

class Drive;
class Tilter;
class Intake;
class Lift;
class PidController;
class PositionTracking;
Drive *driveObj;
Tilter *tilterObj;
Intake *intakeObj;
Lift *liftObj;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

class PositionTracking
{
    public:
    double xPosition = 0;
    double yPosition = 0;
    double lastGyroPosition = actualGyroPosition();
    double prevAngle = 0;
    int prevLWheel = 0;
    int prevRWheel = 0;

    void updatePosition()
    {
        double currentAngle = fmod((((actualGyroPosition()) - 90) * -1) + 360, 360.0);
        double currentAngleDifference = currentAngle - prevAngle;

        int currentLDifference = leftEncoder.get_value() - prevLWheel;
        int currentRDifference = rightEncoder.get_value() - prevRWheel;
        int combinedDistance = currentLDifference + currentRDifference;

        if(combinedDistance != 0)
        {
            double movement = combinedDistance/(double)2;
            pros::lcd::print(5,"%f", movement);

            if(currentAngleDifference == 0)
            {
                xPosition += movement*cos(degToRad(currentAngle));
                yPosition += movement*sin(degToRad(currentAngle));
            }
            else
            {   //fixes movement from arc to vector   Set formula for arc
                double radAngleDif = degToRad(currentAngleDifference);
                movement = 2*(movement/radAngleDif)*sin(radAngleDif/2);
                xPosition += movement*cos(degToRad(currentAngle) + (radAngleDif/2));
                yPosition += movement*sin(degToRad(currentAngle) + (radAngleDif/2));
            }
        }

        prevLWheel = currentLDifference + prevLWheel;
        prevRWheel = currentRDifference + prevRWheel; 
        prevAngle = currentAngle;
    }

    void encoderReset()
    {
        prevLWheel = 0;
        prevRWheel = 0;
    }

    double targetToAngle(double x, double y)
    {
        //((((target-360) * -1) + 90) * 10)
        return correctAtan(yPosition - y, xPosition - x);
    }

    double angleToTarget(double target)
    {

    }

    void reset()
    {
        *this = PositionTracking();
    }
};

PositionTracking posObj;

void resetAutonVals()
{
	tilter.tare_position();
	lift.tare_position();
	leftEncoder.reset();
	rightEncoder.reset();
    posObj.reset();
	for(int motor = 0; motor < leftDrive.size(); motor++)
	{
		leftDrive[motor].tare_position();
		rightDrive[motor].tare_position();
	}
	//gyroI.reset();
	autonTimer.clear();
}

class PidController
{
    private:
        unsigned long lastTime;
        long combinedIntegral;
        double lastError;

    public:
        double P;
        double I;
        double D;
        double minOutput;
        double maxOutput;

        PidController(double P = 0, double I = 0, double D = 0, double minOutput = 0, double maxOutput = 0)
        { //Initializes the object with default values. Also allows values to be put in on creation
            this->P = P; //Differentiates between the member P and the parameter P. this->P is the member
            this->I = I;
            this->D = D;
            this->minOutput = minOutput;
            this->maxOutput = maxOutput;
            reset();
            pros::delay(1); //Prevent divide by 0 error
        }

        void reset()
        {
            lastTime = pros::millis(); //clears the time
            combinedIntegral = 0; //resets other variables
            lastError = 0;
        }

        double output(double currentSensorData, double target)
        {
            double error = target - currentSensorData;
            unsigned long changeInTime = pros::millis() - lastTime;

            //if(((P*error) < maxOutput) || ((I*combinedIntegral) < maxOutput) || ((P*error) > minOutput) || ((I*combinedIntegral) > minOutput)) //to prevent windup
            if((((P*error) + (I*combinedIntegral)) < maxOutput) || (((P*error) + (I*combinedIntegral)) > minOutput))
            {
                long currentIntegral = error * changeInTime; //calculation for the area under the curve for the latest movement. The faster this updates the more accurate
                combinedIntegral += currentIntegral;         //adds latest to the total integral
            }

            double derivative = (error - lastError) / changeInTime; //formula to calculate the (almost) instantaneous rate of change.
            lastTime = pros::millis(); //put here before the returns
            lastError = error;

            double output = (P * error) + (I * combinedIntegral) + (D * derivative);
            if(output > maxOutput)
            {
                return maxOutput;
            }
            if(output < minOutput)
            {
                return minOutput;
            }
            return output;
        }
};

class VelocityDrive
{
public:
static PidController leftDrivePID;
static PidController rightDrivePID;
static int previousLeftDistance;
static int previousRightDistance;
static unsigned long previousRightTime;
static unsigned long previousLeftTime;


static void leftSide(double targetVelocity)
{
    //Derivative
    double velocity = (leftEncoder.get_value() - previousLeftDistance) / (pros::millis() - previousLeftTime);
    double extraSpeed = rightDrivePID.output(velocity, targetVelocity);
    double velocityToSpeed;
    previousLeftTime = pros::millis();
}

static void rightSide(double targetVelocity)
{
    double velocity = (rightEncoder.get_value() - previousRightDistance) / (pros::millis() - previousRightTime);
    double extraSpeed = rightDrivePID.output(velocity, targetVelocity);
    previousRightTime = pros::millis();
}

static void reset()
{
    leftDrivePID = {leftSideP,leftSideI,leftSideD,leftSideMin,leftSideMax};
    rightDrivePID = {rightSideP,rightSideI,rightSideD,rightSideMin,rightSideMax};
    previousLeftDistance = 0;
    previousRightDistance = 0;
    previousRightTime = pros::millis();
    previousLeftTime = pros::millis();
}

private:
// Disallow creating an instance of this object
VelocityDrive() {}
};

PidController VelocityDrive::leftDrivePID{leftSideP,leftSideI,leftSideD,leftSideMin,leftSideMax};
PidController VelocityDrive::rightDrivePID{rightSideP,rightSideI,rightSideD,rightSideMin,rightSideMax};
static int previousLeftDistance = 0;
static int previousRightDistance = 0;
static unsigned long previousRightTime = pros::millis();
static unsigned long previousLeftTime = pros::millis();

class System
{
protected:
    char triggerNumber;
    int triggerBreak;
    System *triggerSystem;
    AutonFlags trigger = NONET; //Holds the type of trigger system will have    All sub systems will have the option of starting on a trigger
    Timer triggerTimer;

    char triggerNumberE;
    int triggerBreakE;
    System *triggerSystemE;
    AutonFlags triggerE = NONETE;
    Timer triggerTimerE;

public:
    System(int idVal): id(idVal){}
    AutonFlags speedControl = BLANK;
    SystemStates state = END;
    PidController pid; //eventualy initialized to defaults later in inherited classes
    int id;
    double target;
    char numberOfCalls = 0;
    char totalNumberOfCalls = 0;

    virtual bool checkIfDone(double breakVal) = 0;
    virtual void setMember(int &number, AutonFlags &currentFlag, int value) = 0;
    virtual void resetObj() = 0;
    virtual void initializePid(AutonFlags pidPack) = 0;

    int getTriggerNumberProgress()
    {
        //number of calls decreases throughout runtime. Starts at 1 and increases
        //if 0 from beginning, initial update takes care of it
        return (totalNumberOfCalls - numberOfCalls)+1;
    }

    void updateEndingState()
    {
        //if it was on its last run
        if(numberOfCalls == 0)
        {
            state = END;
        }
        else
        {
            state = WAITINGFORINSTRUCTIONS;
        }
    }

    bool triggerCheckE(int breakVal)
    {
        switch(triggerE)
        {
            case(NONETE):
                return true;
            case(TIMETE):
                if(triggerTimerE.current() < breakVal)
                {
                    return false;
                }
                return true;
            default:
                //if the system is already done don't bother checking
                if(triggerSystemE->state == END)
                {
                    return true;
                }
                //if the current call is currently passed the one wanted, break
                else if(triggerNumberE < triggerSystemE->getTriggerNumberProgress())
                {
                    return true;
                }
                //if not, wait
                else if(triggerNumberE > triggerSystemE->getTriggerNumberProgress())
                {
                    return false;
                }
                else //is equal and needs to be checked
                {
                    if(triggerSystemE->checkIfDone(breakVal) == true)
                    {
                        return true;
                    }
                    return false;
                }
        }
    }

    void updateTriggerState()
    {
        switch(trigger)  //have it set at something until everything ends or its set to end
        {
            case(NONET):
                state = EXECUTINGINSTRUCTIONS;
                break;
            case(TIMET):
                if(triggerTimer.current() > triggerBreak)
                {
                    state = EXECUTINGINSTRUCTIONS;
                    break;
                }
                state = WAITINGFORTRIGGER;
                break;
            default:
                if(triggerSystem->state == END)
                {
                    state = EXECUTINGINSTRUCTIONS;
                    break;
                }
                else if(triggerNumber < triggerSystem->getTriggerNumberProgress())
                {
                    state = EXECUTINGINSTRUCTIONS;
                    break;
                }
                else if(triggerNumber > triggerSystem->getTriggerNumberProgress())
                {
                    state = WAITINGFORTRIGGER;
                    break;
                }
                else //is equal and needs to be checked
                {
                    if(triggerSystem->checkIfDone(triggerBreak) == true)
                    {
                        state = EXECUTINGINSTRUCTIONS;
                        break;
                    }
                    state = WAITINGFORTRIGGER;
                    break;
                }
        }
        triggerTimerE.clear();
    }

    //sets system variables scrolling through main parameter vectors, all systems can have these options
    //returns whether or not to check the virtual flag options
    bool setSystemMember(int &number, AutonFlags &currentFlag, int value) //returning true means to check the other flags
    {
        if(number == 0)
        {
            currentFlag = (AutonFlags)value;
            switch(currentFlag)
            {
                case(NONET):
                    trigger = currentFlag;
                    return false;
                case(NONETE):
                    triggerE = currentFlag;
                    return false;
                case(REGPID):
                    speedControl = currentFlag;
                    initializePid(currentFlag); //pure virtual function
                    return false;
                case(TURNPID):
                    speedControl = currentFlag;
                    pid = PidController(regTurnP,regTurnI,regTurnD,regTurnMin,regTurnMax);
                    return false;
                case(TURNPID2):
                    speedControl = currentFlag;
                    pid = PidController(regTurnP2,regTurnI2,regTurnD2,regTurnMin2,regTurnMax2);
                    return false;
                case(DRIVEPID):
                    speedControl = currentFlag;
                    pid = PidController(regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax);
                    return false;
                case(TILTERPID):
                    speedControl = currentFlag;
                    pid = PidController(regTilterP,regTilterI,regTilterD,regTilterMin,regTilterMax);
                    return false;
                case(LIFTPID):
                    speedControl = currentFlag;
                    pid = PidController(regLiftP,regLiftI,regLiftD,regLiftMin,regLiftMax);
                    return false;
                default:
                    return true; //dont plus one yet because it will go through the other flags
            }
        }
        else
        {
            switch(currentFlag)
            {
                case(DRIVET):
                case(TILTERT):
                case(INTAKET):
                case(LIFTT):
                    switch(number)
                    {
                        case(1):       //its never going to be 0 because it only goes in this loops if its not 0;
                            if(currentFlag == DRIVET)
                            {
                                //changes pointers to system pointers making calls be able to be managed by non-virtual functions
                                triggerSystem = reinterpret_cast<System *>(driveObj);
                            }
                            else if(currentFlag == TILTERT)
                            {
                                triggerSystem = reinterpret_cast<System *>(tilterObj);
                            }
                            else if(currentFlag == INTAKET)
                            {
                                triggerSystem = reinterpret_cast<System *>(intakeObj);
                            }
                            else if(currentFlag == LIFTT)
                            {
                                triggerSystem = reinterpret_cast<System *>(liftObj);
                            }
                            trigger = currentFlag;
                            triggerNumber = value; //distance
                            number++;
                            return false;
                        case(2):
                            number = 0;
                            triggerBreak = value;
                            return false;
                    }
                case(TIMET):
                    trigger = currentFlag;
                    triggerBreak = value;
                    triggerTimer.clear();
                    number = 0;
                    return false;
                case(DRIVETE):
                case(TILTERTE):
                case(INTAKETE):
                case(LIFTTE):
                    switch(number)
                    {
                        case(1):
                            if(currentFlag == DRIVETE)
                            {
                                triggerSystemE = reinterpret_cast<System *>(driveObj);
                            }
                            else if(currentFlag == TILTERTE)
                            {
                                triggerSystemE = reinterpret_cast<System *>(tilterObj);
                            }
                            else if(currentFlag == INTAKETE)
                            {
                                triggerSystemE = reinterpret_cast<System *>(intakeObj);
                            }
                            else if(currentFlag == LIFTTE)
                            {
                                triggerSystemE = reinterpret_cast<System *>(liftObj);
                            }
                            triggerE = currentFlag;
                            triggerNumberE = value; //distance
                            number++;
                            return false;
                        case(2):
                            number = 0;
                            triggerBreakE = value;
                            return false;
                    }
                case(TIMETE):
                    triggerE = currentFlag;
                    triggerBreakE = value;
                    number = 0;
                    return false;
                case(NOPID):
                    initializePid(REGPID);////////////
                    pid.minOutput = value; //reuse these values as single speed holders
                    pid.maxOutput = value;
                    speedControl = currentFlag;
                    number = 0;
                    return false;
                case(MMREGPID):
                    switch(number)
                    {
                        case(1):
                            speedControl = currentFlag;
                            initializePid(currentFlag);
                            number++;
                            return false;
                        case(2):
                            pid.minOutput = value;
                            number++;
                            return false;
                        case(3):
                            pid.maxOutput = value;
                            number = 0;
                            return false;
                    }
                case(CUSTOMPID):
                    switch(number)
                    {
                        case(1):
                            speedControl = currentFlag;
                            pid.minOutput = value;
                            number++;
                            return false;
                        case(2):
                            pid.maxOutput = value;
                            number++;
                            return false;
                        case(3):
                            pid.P = value;
                            number++;
                            return false;
                        case(4):
                            pid.I = value;
                            number++;
                            return false;
                        case(5):
                            pid.D = value;
                            number = 0;
                            return false;
                    }
                default:
                    return true;
            }
        }

    }

    void initialUpdate(int &i, std::vector<int> &parameters)
    {
        if(parameters[i] == id)
        {
            if(state == END) //if the object call in uninitialized
            {
                parameters[i] = (int)NULLOPTION; //sets it back to NULLOPTION so it doesn't get called again
                AutonFlags currentFlag;
                int subFlag = 0;
                state = WAITINGFORTRIGGER;
                do //loops through the data to give to the specific system
                {
                    ++i;
                    bool checkSecondaryFlags = setSystemMember(subFlag,currentFlag,parameters[i]);
                    if(checkSecondaryFlags == true)
                    {
                        setMember(subFlag,currentFlag,parameters[i]);
                    }
                } while(parameters[i+1] < minEnumValue);
            }
            else
            {
                ++totalNumberOfCalls;
                ++numberOfCalls;
            }
        }
    }

    void update(std::vector<int> &parameters)
    {
        if(state == WAITINGFORINSTRUCTIONS)
        {
            for(int i = 0; i < parameters.size()-1; i++)
            {
                if(parameters[i] == id)
                {
                    if(state == WAITINGFORINSTRUCTIONS) //Checks again just in case there are multiple calls.
                    {
                        resetObj(); //resets the object
                        parameters[i] = (int)NULLOPTION;
                        AutonFlags currentFlag;
                        int subFlag = 0;

                        do //loops through the data to give to the specific system
                        {
                            ++i;
                            bool checkSecondaryFlags = setSystemMember(subFlag,currentFlag,parameters[i]);
                            if(checkSecondaryFlags == true)
                            {
                                setMember(subFlag,currentFlag,parameters[i]);
                            }
                        } while(parameters[i+1] < minEnumValue);

                        state = WAITINGFORTRIGGER; //trigger obj might not be in scope yet so cant check its process.
                    }
                    else
                    {
                        ++numberOfCalls;
                    }
                }
            }
        }
    }
};

class Drive : public System
{
    private:
    double correctTo = -1;
    bool stopDeacceleration = false;
    bool brake = true;
    bool runBrake = false;
    bool inverse = false;
    int accelerationMin;
    int xTarget = 0;
    int yTarget = 0;
    PidController straightDrivePID = {straightDriveP, straightDriveI,straightDriveD,straightDriveMin,straightDriveMax};
    AutonFlags accelerationControl = NOACCEL;
    GyroDistances turnStats;
    AutonFlags direction;
    Timer accelerationTimer;
    Timer brakeTimer;

    public:
    int speed;
    bool rightTurn;
    double outerInnerRatio = 1;
    Drive(int idVal = DRIVE)
        : System((int)idVal)
        {
            pid = {regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax};
            clearEncoders();
            triggerTimer.clear();
            accelerationTimer.clear();
            if(driveObj == nullptr)
            {
                driveObj = this; //resets the global pointer to the reset object
            }
            else
            {
                xTarget = driveObj->xTarget;
                yTarget = driveObj->yTarget;
                correctTo = driveObj->correctTo;
                totalNumberOfCalls = driveObj->totalNumberOfCalls; //passes on the totalNumberOfCalls after reset
            }
        };

    void clearEncoders()
    {
        posObj.updatePosition();
        leftEncoder.reset();
        rightEncoder.reset();
        posObj.encoderReset();
        for(int motor = 0; motor < leftDrive.size(); motor++)
        {
            leftDrive[motor].tare_position();
            rightDrive[motor].tare_position();
        }
    }

    void verifySpeedOutput(float &leftCorrect, float &rightCorrect)
    {
        if(abs(speed*rightCorrect) < pid.minOutput)
        {
            double constantToMinimum = pid.minOutput/abs(speed*rightCorrect);
            rightCorrect *= constantToMinimum;
        }
        if(abs(speed*leftCorrect) < pid.minOutput)
        {
            double constantToMinimum = pid.minOutput/abs(speed*leftCorrect);
            leftCorrect *= constantToMinimum;
        }
    }

    bool checkIfDone(double breakVal)
    {
        if(triggerE == NONETE)
        {
            if((direction == FORWARDSC) || (direction == BACKWARDSC))
            {
                //simple distance formula based of the pythagorean theorum
                int distanceToTarget = sqrt(pow(xTarget - posObj.xPosition, 2) + pow(yTarget - posObj.yPosition, 2));
                if(distanceToTarget < breakVal)
                {
                    return true;
                }
                return false;
            }
            if((direction != TURN) && (direction != TURNC))
            {
                if(getDriveEncoder() > breakVal)
                {
                    return true;
                }
                return false;
            }
            GyroDistances gyroVals;

            if(direction == TURNC)
            {
                double turncTarget = correctAtan(posObj.yPosition - yTarget, posObj.xPosition - xTarget);
                turncTarget = ((turncTarget - 360) * -1) + 90;

                if(inverse == true)
                {
                    turncTarget += 180;
                }
                //correctTo = fixTarget(turncTarget);
                getDistances(gyroVals, fixTarget(turncTarget));
            }
            else 
            {
                getDistances(gyroVals, target);
            }
            
            if((gyroVals.Left < breakVal) || (gyroVals.Right < breakVal))
            {   /*pros::lcd::print(1,"%f", gyroVals.Left);
                pros::lcd::print(2,"%f", target);
                pros::lcd::print(3,"%f", actualGyroPosition());
                pros::lcd::print(4,"%f", gyroI.get_heading());
                driveMotorsSpeed(0, leftDrive);
                driveMotorsSpeed(0, rightDrive);
                while(1)
                {
                    pros::delay(10);
                }*/
                return true;
            }
            
            return false;
        }
        else
        { //ending triggers are universal so a system function can be used.
            return triggerCheckE(breakVal);
        }
    }

    void gyroCorrections(float &leftCorrect, float &rightCorrect)
    {
        GyroDistances off;
        getDistances(off, correctTo);

        if(direction == FORWARDS || direction == FORWARDSE || direction == FORWARDSC)
        {
            if(off.Right < off.Left)
            {
                if(off.Right > 0.3)
                {
                    rightCorrect *= 1.0 - straightDrivePID.output(-off.Right+0.3, 0);
                }
            }
            else
            {
                if(off.Left > 0.3)
                {
                    leftCorrect *= 1.0 + straightDrivePID.output(off.Left+0.3, 0);
                }
            }
        }
        else 
        {
            if(off.Right < off.Left)
            {
                if(off.Right > 0.3)
                {
                    leftCorrect *= 1.0 - straightDrivePID.output(-off.Right+0.3, 0);
                }
            }
            else
            {
                if(off.Left > 0.3)
                {
                    rightCorrect *= 1.0 + straightDrivePID.output(off.Left+0.3, 0);
                }
            }
        }
    }

    double accelerationOutput()
    {
        double output = (((pid.maxOutput-accelerationMin)/150) * accelerationTimer.current()) + accelerationMin;
        if(accelerationTimer.current() > 150)
        {
            accelerationControl = NOACCEL;
            return pid.maxOutput;
        }

        if((direction == FORWARDSC) || (direction == BACKWARDSC))
        {
            int distanceToTarget = sqrt(pow(xTarget - posObj.xPosition, 2) + pow(yTarget - posObj.yPosition, 2));
            if(output > pid.output(-distanceToTarget, 0))
            {
                accelerationControl = NOACCEL;
                return pid.output(-distanceToTarget, 0);
            }
        }
        else
        {
            if(output > pid.output(getDriveEncoder(), target))
            {
                accelerationControl = NOACCEL;
                return pid.output(getDriveEncoder(), target);
            }
        }
        return output;
    }

    int getOutsideEncoder()
    {
        switch(direction)
        {
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
            case(UPLEFTSWEEPE):
            case(DOWNLEFTSWEEPE):
                return abs(rightEncoder.get_value());
            default:
                return abs(leftEncoder.get_value());
        }
    }

    int getInsideEncoder()
    {
        switch(direction)
        {
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
            case(UPLEFTSWEEPE):
            case(DOWNLEFTSWEEPE):
                return abs(leftEncoder.get_value());
            default:
                return abs(rightEncoder.get_value());
        }
    }

    int getSweepDifference() //positive, outside is faster. negative, inside is faster
    {
        double offSet = (double)getOutsideEncoder()*outerInnerRatio;
        (double)getOutsideEncoder() - (double)(getInsideEncoder() + offSet);
    }

    void wheelCorrections(float &leftCorrect, float &rightCorrect)
    {
        int difference;
        if(direction == TURN || direction == FORWARDSC || direction == BACKWARDSC || direction == TURNC || direction == FORWARDS || direction == FORWARDSE || direction == BACKWARDS || direction == BACKWARDSE)
        {
            difference = abs(rightEncoder.get_value()) - abs(leftEncoder.get_value());
        }
        else
        {
            difference = getSweepDifference(); //reduces call amount
        }

        if(difference > 4)
        {
            if(direction == DOWNRIGHTSWEEP || direction == DOWNRIGHTSWEEPE || direction == DOWNLEFTSWEEP || direction == DOWNLEFTSWEEPE)
            {
                leftCorrect *= ((float).99 - ((float)abs(difference)/(float)100));
            }
            else 
            {
                rightCorrect *= ((float).99 - ((float)abs(difference)/(float)100));
            }
        }
        else if(difference < -4)
        {
            if(direction == DOWNRIGHTSWEEP || direction == DOWNRIGHTSWEEPE || direction == DOWNLEFTSWEEP || direction == DOWNLEFTSWEEPE)
            {
                rightCorrect *= ((float).99 - ((float)abs(difference)/(float)100));
            }
            else 
            {
                leftCorrect *= ((float).99 - ((float)abs(difference)/(float)100));
            }
        }
    }

    void resetObj()
    {
        *this = Drive();
    }

    void initializePid(AutonFlags pidPack)
    {
        pid = PidController(regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax);
    }

    void setMember(int &number, AutonFlags &currentFlag, int value);
    void move();

    int getDriveEncoder()
    {
        switch(direction)
        {
            case(UPRIGHTSWEEP):
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
            case(DOWNRIGHTSWEEP):
            case(UPRIGHTSWEEPE):
            case(UPLEFTSWEEPE):
            case(DOWNLEFTSWEEPE):
            case(DOWNRIGHTSWEEPE):
                return getOutsideEncoder();
            default:
                //return abs(leftEncoder.get_value()); //////////////////abs
                return abs(leftEncoder.get_value());
        }
    }
};

void Drive::setMember(int &number, AutonFlags &currentFlag, int value)
{
    if(number == 0)
    {
        currentFlag = (AutonFlags)value;
        switch(currentFlag) //for if
        {
            case(NOACCEL):
                accelerationControl = currentFlag;
                break;
            case(REGACCEL):
                accelerationControl = currentFlag;
                accelerationMin = regDriveMin;
                break;
            case(NOBRAKE):
                brake = false;
                break;
            case(INVERSE):
                inverse = true;
                break;
            default:
                number++; //Just sets currentFlag, will get next values when increasead next time
                break;
        }
    }
    else
    {
        switch(currentFlag)
        {
            case(ACCEL):
                accelerationControl = currentFlag;
                accelerationMin = value;
                number = 0;
                break;
            case(TURN):
            case(FORWARDS):
            case(BACKWARDS):
            case(FORWARDSE):
            case(BACKWARDSE):
                switch(number) //use same straight drive for turns and straight drive
                {
                    case(1):
                        direction = currentFlag;
                        if(direction == TURN)
                        {
                            target = (double)value / 10.0;
                        }
                        else
                        {
                            target = (double)value;
                        }
                        number++;
                        break;
                    case(2):
                        if(value == CURRENTVAL)
                        {
                            correctTo = actualGyroPosition();
                        }
                        else if(value == PAST)
                        {
                            //value already set so don't do anything
                        }
                        else if(value == NOSTRAIGHT)
                        {
                            correctTo = NOSTRAIGHT;
                        }
                        else
                        {
                            correctTo = value / 10.0;
                        }
                        number = 0;
                        break;
                }
                break;
            case(UPRIGHTSWEEP):
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
            case(DOWNRIGHTSWEEP):
            case(UPRIGHTSWEEPE):
            case(UPLEFTSWEEPE):
            case(DOWNLEFTSWEEPE):
            case(DOWNRIGHTSWEEPE):
                switch(number)
                {
                    case(1):
                        direction = currentFlag;
                        target = value; //distance
                        number++;
                        break;
                    case(2):
                        number++; //radius
                        outerInnerRatio = (double)(value - wheelDistance) / value;
                        break;
                    case(3):
                        if(value == NOSTRAIGHT)
                        {
                            correctTo = NOSTRAIGHT;
                        }
                        else
                        {
                            correctTo = WHEELCORRECTION;
                        }
                        number = 0;
                        break;
                }
                break;
            case(TURNC):
                switch(number) //use same straight drive for turns and straight drive
                {
                    case(1):
                        direction = currentFlag;
                        xTarget = value;
                        number++;
                        break;
                    case(2):
                        yTarget = value;
                        number++;
                        break;
                    case(3):
                        correctTo = value; //posibility for wheelCorrect
                        number = 0;
                        break;
                }
                break;
            case(FORWARDSC):
            case(BACKWARDSC):
                switch(number)
                {
                    case(1):
                        direction = currentFlag;
                        target = value; //distance
                        number++;
                        break;
                    case(2):
                        if(value == PAST)
                        {
                            number = 0;
                        }
                        else
                        {
                            number++;
                            xTarget = value;
                        }
                        break;
                    case(3):
                        yTarget = value;
                        number = 0;
                        //number++;
                        break;
                        /*
                    case(4): //Only would be used if the wheels could eventually be used for coordinate straight driving. Possible but gyro is better
                        if(value == NOSTRAIGHT)
                        {
                            correctTo = NOSTRAIGHT;
                        }
                        else if(value == WHEELCORRECTION)
                        {
                            correctTo = WHEELCORRECTION;
                        }
                        number = 0;
                        break;*/
                }
                break;

        }
    }
}

class Tilter : public System  //very quick acceleration
{
    private:
    int speed;

    public:
    bool underTarget;
    Tilter(int id = TILTER)
        : System((int)id)
        {
            pid = {regTilterP,regTilterI,regTilterD,regTilterMin,regTilterMax};
            if(tilterObj == nullptr) //if declairing this for the first time
            {
                tilterObj = this;
            }
            else //global is declaired meaning the object is being reset. Keep the state and total number of calls.
            {
                totalNumberOfCalls = tilterObj->totalNumberOfCalls;
            }
        }

    int getPosition()
    {
        return (int)fabs(tilter.get_position());
    }

    bool checkIfDone(double breakVal)
    {
        if(triggerE == NONETE)
        {
            if(underTarget == true)
            {
                if(getPosition() > breakVal)
                {
                    return true;
                }
                return false;
            }
            else
            {
                if(getPosition() < breakVal)
                {
                    return true;
                }
                return false;
            }
        }
        else
        {
            return triggerCheckE(breakVal);
        }
    }

    void resetObj()
    {
        *this = Tilter();
    }

    void initializePid(AutonFlags pidPack)
    {
        pid = PidController(regTilterP,regTilterI,regTilterD,regTilterMin,regTilterMax);
    }

    void setMember(int &number, AutonFlags &currentFlag, int value)
    {
        if(number == 0)
        {
            currentFlag = (AutonFlags)value;
            number++;
            return;
        }
        else
        {
            switch(currentFlag)
            {
                case(POSITION):
                    switch(number) //use same straight drive for turns and straight drive
                    {
                        case(1):
                            target = value;
                            number++;
                            break;
                        case(2):
                            speed = value;
                            number = 0;
                            break;
                    }
                    break;
                case(SPEED):
                    switch(number) //use same straight drive for turns and straight drive
                    {
                        case(1):
                            speed = value;
                            number = 0;
                            break;
                    }
                    break;
            }
        }
    }

    void move()
    {
        int breakVal = (triggerE == NONETE) ? target : triggerBreakE;
        if(checkIfDone(breakVal) == false)
        {
            if(triggerE != NONETE)
            {
                tilter.move(speed);
            }
            else
            {
                if(getPosition() < target)
                {
                    tilter.move(speed);
                }
                else
                {
                    tilter.move(-speed);
                }
            }
        }
        else
        {
            tilter.move(0);
            updateEndingState();
        }
    }
};

class Intake : public System
{
    private:
    int speed;

    public:
    AutonFlags direction;
    Intake(int id = INTAKE)
        : System((int)id)
        {
            pid = {regIntakeP,regIntakeI,regIntakeD,regIntakeMin,regIntakeMax};
            if(intakeObj == nullptr) //if declairing this for the first time
            {
                intakeObj = this;
            }
            else //global is declaired meaning the object is being reset. Keep the state and total number of calls.
            {
                totalNumberOfCalls = intakeObj->totalNumberOfCalls;
            }
        }

    bool checkIfDone(double breakVal)
    {
        return triggerCheckE(breakVal);
    }

    void resetObj()
    {
        *this = Intake();
    }

    void initializePid(AutonFlags pidPack)
    {
        pid = PidController(regIntakeP,regIntakeI,regIntakeD,regIntakeMin,regIntakeMax);
    }

    void setMember(int &number, AutonFlags &currentFlag, int value)
    {
        if(number == 0)
        {
            currentFlag = (AutonFlags)value;
            ++number;
        }
        else
        {
            switch(currentFlag)
            {
                case(IN):
                case(OUT):
                    direction = currentFlag;
                    speed = value;
                    number = 0;
                    break;
            }
        }
    }

    void move()
    {
        if(direction == OUT)
        {
            speed *= -1;
        }
        motorGroupMove(speed, intakeM);

        if(triggerE == NONETE) //timete
        {
            updateEndingState();
        }
        else
        {
            if(checkIfDone(triggerBreakE) == true)
            {
                motorGroupMove(0, intakeM); //only goes to zero after an ending trigger, this allows for the setting of the intake
                updateEndingState();        //without a specific time or place to stop since it will most likely be going throught the
                                            //entire time just about
            }
        }
    }
};

class Lift : public System
{
    private:
    int speed;

    public:
    bool underTarget;
    Lift(int id = LIFT)
        : System((int)id)
        {
            pid = {regLiftP,regLiftI,regLiftD,regLiftMin,regLiftMax};
            if(liftObj == nullptr) //if declairing this for the first time
            {
                liftObj = this;
            }
            else //global is declaired meaning the object is being reset. Keep the state and total number of calls.
            {
                totalNumberOfCalls = liftObj->totalNumberOfCalls;
            }
        }

    int getPosition()
    {
        return (int)fabs(lift.get_position());
    }

    bool checkIfDone(double breakVal)
    {
        if(triggerE == NONETE)
        {
            if(underTarget == true)
            {
                if(getPosition() > breakVal)
                {
                    return true;
                }
                return false;
            }
            else
            {
                if(getPosition() < breakVal)
                {
                    return true;
                }
                return false;
            }
        }
        else
        {
            return triggerCheckE(breakVal);
        }
    }

    void resetObj()
    {
        *this = Lift();
    }

    void initializePid(AutonFlags pidPack)
    {
        pid = PidController(regLiftP,regLiftI,regLiftD,regLiftMin,regLiftMax);
    }

    void setMember(int &number, AutonFlags &currentFlag, int value)
    {
        if(number == 0)
        {
            currentFlag = (AutonFlags)value;
            number++;
            return;
        }
        else
        {
            switch(currentFlag)
            {
                case(POSITION):
                    switch(number) //use same straight drive for turns and straight drive
                    {
                        case(1):
                            target = value;
                            number++;
                            break;
                        case(2):
                            speed = value;
                            number = 0;
                            break;
                    }
                    break;
                case(SPEED):
                    switch(number) //use same straight drive for turns and straight drive
                    {
                        case(1):
                            speed = value;
                            number = 0;
                            break;
                    }
                    break;
            }
        }
    }

    void move()
    {
        int breakVal = (triggerE == NONETE) ? target : triggerBreakE;
        if(checkIfDone(breakVal) == false)
        {
            if(triggerE != NONETE)
            {
                lift.move(speed);
            }
            else
            {
                if(getPosition() < target)
                {
                    lift.move(speed);
                }
                else
                {
                    lift.move(-speed);
                }
            }
        }
        else
        {
            lift.move(0);
            updateEndingState();
        }
    }
};

void Drive::move()
{
    //Corrections are reset to 1 every call
    float leftCorrection = 1;
    float rightCorrection = 1;

    double breakVal;
    if(triggerE != NONETE)
    {
        breakVal = triggerBreakE;
    }
    else if((direction == TURN) || (direction == TURNC)) //how far away it is from the target, not on specific value.
    {
        breakVal = 0.6;
    }
    else if((direction == FORWARDSC) || (direction == BACKWARDSC))
    {
        breakVal = 4;
    }
    else
    {
        breakVal = target;
    }

    if(runBrake == true)
    {
        int brakeTime;
        int brakeSpeed;
        if(direction == TURN)
        {
            brakeTime = brakeTimeTurn;
            brakeSpeed = brakeTimeTurn;
        }
        else
        {
            brakeTime = brakeTimeDrive;
            brakeSpeed = brakeTimeDrive;
        }

        if(brakeTimer.current() < brakeTime)
        {
            int leftDirection;
            int rightDirection;
            switch(direction)
            {
                case(DOWNLEFTSWEEP):
                case(DOWNRIGHTSWEEP):
                case(BACKWARDS):
                case(DOWNLEFTSWEEPE):
                case(DOWNRIGHTSWEEPE):
                case(BACKWARDSE):
                case(BACKWARDSC):
                    break;
                case(TURN):
                    if(rightTurn == true)
                    {
                        leftDirection *= -1;
                    }
                    else
                    {
                        rightDirection *= -1;
                    }
                    break;
                default:
                    leftDirection *= -1;
                    rightDirection *= -1;
                    break;
            }
            driveMotorsSpeed(rightDirection*brakeSpeed,rightDrive);
            driveMotorsSpeed(leftDirection*brakeSpeed,leftDrive);
        }
        else
        {
            driveMotorsSpeed(0,rightDrive);
            driveMotorsSpeed(0,leftDrive);
            updateEndingState();
        }
    }
    else
    {
        /*
                  /|
                 / |
                /  |
               /   | y
              /    |
             /     |
            /______|
               x
        */
        //Maybe have a forward option that just takes the value needed instead of having to find the slope involved with going a straight line.
        //past coordinates, update with the total number of calls

        //(xPosition-targetX)^2 + (yPosition-targetY)^2
        //sqr(ans) //Distance to target
        //(opposite is sin)(adjacent is cos)
        //correctAtan((yPosition-yTarget)/(xPosition-xTarget)) //angle needed for movement / set to correct to and inbeginning turn

        if(checkIfDone(breakVal) == false)
        {
            if((direction != TURN) && (direction != TURNC))
            {
                if(speedControl == NOPID)
                {
                    speed = pid.maxOutput;
                }
                else if(triggerE != NONETE)
                {
                    speed = target;
                }
                else
                {
                    if((direction == FORWARDSC) || (direction == BACKWARDSC))
                    {
                        if(accelerationControl != NOACCEL)
                        {
                            speed = accelerationOutput();
                        }
                        else
                        {
                            int distanceToTarget = sqrt(pow(xTarget - posObj.xPosition, 2) + pow(yTarget - posObj.yPosition, 2));
                            speed = pid.output(-distanceToTarget, 0);
                        }
                    }
                    else
                    {
                        if(accelerationControl != NOACCEL)
                        {
                            speed = accelerationOutput();
                        }
                        else
                        {
                            speed = pid.output(getDriveEncoder(), target);
                        }
                    }
                }

                //Deals with sweepturns and sets the speed according to the radius specified
                switch(direction)
                {
                    case(UPLEFTSWEEP):
                    case(UPRIGHTSWEEP):
                    case(DOWNLEFTSWEEP):
                    case(DOWNRIGHTSWEEP):
                    case(UPLEFTSWEEPE):
                    case(UPRIGHTSWEEPE):
                    case(DOWNLEFTSWEEPE):
                    case(DOWNRIGHTSWEEPE):
                        switch(direction) //sweep turn stuffz
                        {
                            case(UPLEFTSWEEP):
                            case(DOWNLEFTSWEEP):
                            case(UPLEFTSWEEPE):
                            case(DOWNLEFTSWEEPE):
                                leftCorrection *= outerInnerRatio;
                                break;
                            default:
                                rightCorrection *= outerInnerRatio;
                                break;
                        }
                        break;
                }

                //Applies the autocorrect if specified
                if(correctTo == WHEELCORRECTION)
                {
                    wheelCorrections(leftCorrection, rightCorrection); //break into two, trigger system, other other
                }
                else if(correctTo != NOSTRAIGHT)
                {
                    if((direction == FORWARDSC) || (direction == BACKWARDSC))
                    {
                        correctTo = correctAtan(posObj.yPosition - yTarget, posObj.xPosition - xTarget);
                        correctTo = fixTarget(((correctTo - 360) * -1) + 90);
                    }
                    gyroCorrections(leftCorrection, rightCorrection);
                }

                //controls wheels going forwards or backwards
                switch(direction)
                {
                    case(DOWNLEFTSWEEP):
                    case(DOWNRIGHTSWEEP):
                    case(BACKWARDS):
                    case(DOWNLEFTSWEEPE):
                    case(DOWNRIGHTSWEEPE):
                    case(BACKWARDSE):
                    case(BACKWARDSC):
                        leftCorrection *= -1;
                        rightCorrection *= -1;
                        break;
                }

                //check to make sure the speed won't be under the movable rate
                verifySpeedOutput(leftCorrection, rightCorrection);

                //Sets the drive motors at speed multiplied by the respective correction
                //Corrections start at 1 by default
                driveMotorsSpeed(speed*rightCorrection, rightDrive);
                driveMotorsSpeed(speed*leftCorrection, leftDrive);
            }
            else
            {
                if(direction == TURNC)
                {
                    target = correctAtan(posObj.yPosition - yTarget, posObj.xPosition - xTarget);
                    target = (((target-360) * -1) + 90);

                    if(inverse == true)
                    {
                        target += 180;
                    }
                }
                
                target = fixTarget(target); //so you can put in negative values
                GyroDistances Dist;
                getDistances(Dist, target);

                if(Dist.Right < Dist.Left)
                {
                    rightCorrection *= -1;
                    speed = pid.output(-Dist.Right, 0);
                }
                else
                {
                    leftCorrection *= -1;
                    speed = pid.output(-Dist.Left, 0);
                }
                if(correctTo == WHEELCORRECTION)
                {
                    wheelCorrections(leftCorrection, rightCorrection);
                }

                driveMotorsSpeed((float)speed*leftCorrection,leftDrive);
                driveMotorsSpeed((float)speed*rightCorrection,rightDrive);
            }
        }
        else
        {
            if(brake == true)
            {
                runBrake = true;
                brakeTimer.clear();
            }
            else
            {
                driveMotorsSpeed(0,rightDrive);
                driveMotorsSpeed(0,leftDrive);
                updateEndingState();
            }
        }
    }
}

template <typename... Ts>
void addCommands(Ts... input)
{
    std::vector<int> parameters = {(int)input...,NULLOPTION};

    Drive drive{};
    Tilter tilter{};
    Intake intake{};
    Lift lift{};

    for(int i = 0; i < parameters.size()-1; i++)
    {
        drive.initialUpdate(i, parameters);
        tilter.initialUpdate(i, parameters);
        intake.initialUpdate(i, parameters);
        lift.initialUpdate(i, parameters);
    }

    if(tilter.getPosition() > tilter.target)
    {
        tilter.underTarget = false;
    }
    else
    {
        tilter.underTarget = true;
    }

    if(lift.getPosition() > lift.target)
    {
        lift.underTarget = false;
    }
    else
    {
        lift.underTarget = true;
    }
    drive.pid.minOutput /= drive.outerInnerRatio;/////
    while(((tilter.state != END) || (drive.state != END) || (intake.state != END) || (lift.state != END)) && (autonTimer.current() < 15000))
    {
        pros::lcd::print(3,"%f", actualGyroPosition());
        //posObj.updatePosition();
        switch(drive.state)
        {
            case(WAITINGFORINSTRUCTIONS):
                drive.update(parameters);
                drive.pid.minOutput /= drive.outerInnerRatio; //has to be put here so it doesn't get overwritten by the pid initialization
                break;
            case(WAITINGFORTRIGGER):
                drive.updateTriggerState();
                GyroDistances Dist;
                getDistances(Dist, drive.target);
                if(Dist.Right < Dist.Left)
                {
                    drive.rightTurn = true;
                }
                else
                {
                    drive.rightTurn = false;
                }
                break;
            case(EXECUTINGINSTRUCTIONS):
                drive.move();
                break;
            case(END):
                driveMotorsSpeed(0,rightDrive);
                driveMotorsSpeed(0,leftDrive);
                break; //adding retain position later if needed
        }

        switch(tilter.state)
        {
            case(WAITINGFORINSTRUCTIONS):
                tilter.update(parameters);
                if(tilter.getPosition() > tilter.target)
                {
                    tilter.underTarget = false;
                }
                else
                {
                    tilter.underTarget = true;
                }
                break;
            case(WAITINGFORTRIGGER):
                tilter.updateTriggerState();
                break;
            case(EXECUTINGINSTRUCTIONS):
                tilter.move();
                break;
            case(END):
                break;
        }

        switch(lift.state)
        {
            case(WAITINGFORINSTRUCTIONS):
                lift.update(parameters);
                if(lift.getPosition() > lift.target)
                {
                    lift.underTarget = false;
                }
                else
                {
                    lift.underTarget = true;
                }
                break;
            case(WAITINGFORTRIGGER):
                lift.updateTriggerState();
                break;
            case(EXECUTINGINSTRUCTIONS):
                lift.move();
                break;
            case(END):
                break;
        }

        switch(intake.state)
        {
            case(WAITINGFORINSTRUCTIONS):
                intake.update(parameters);
                break;
            case(WAITINGFORTRIGGER):
                intake.updateTriggerState();
                break;
            case(EXECUTINGINSTRUCTIONS):
                intake.move();
                break;
            case(END):
                break;
        }
        pros::delay(2);
    }

    driveObj = nullptr;  //while the actual object gets "deleted" the space in the memory might still hold the old data causing it to falsly read a valid address
    intakeObj = nullptr;
    tilterObj = nullptr;
    liftObj = nullptr;
}

#include "bigAutons.hpp"

void smallBlue()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1700,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,250,0,TIMET,600,
    DRIVE,FORWARDS,-1,0,NOBRAKE,LIFTT,2,600,//TIMET,1300,
    DRIVE,FORWARDS,180,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,500,//TIMET,1300,
    DRIVE,FORWARDS,550,0,NOPID,44,
    DRIVE,TURN,520,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,900,520,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,55,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,970,0,NOPID,44,
    DRIVE,BACKWARDS,460,0,
    DRIVE,TURN,2250,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,340,2250,REGACCEL,NOBRAKE,MMREGPID,50,127,
    DRIVE,FORWARDSE,55,2250,TIMETE,450,
    INTAKE,OUT,127,LIFTT,1,800,
    INTAKE,IN,127,TIMET,1100,
    INTAKE,IN,45,DRIVET,10,15,
    TILTER,POSITION,1000,90,DRIVET,10,10
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,36,TILTERT,1,1400,
    INTAKE,OUT,0,TILTERT,1,3800,
    TILTER,POSITION,6700,127,
    TILTER,POSITION,7100,90
    );
addCommands(
    INTAKE,OUT,127,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,1000,NOSTRAIGHT,TIMET,200,
    TILTER,POSITION,5000,127,TIMET,50
);
}

void smallRed()
{
    resetAutonVals();
addCommands(
    LIFT,POSITION,1700,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,250,0,TIMET,600,
    DRIVE,FORWARDS,-1,0,NOBRAKE,LIFTT,2,600,//TIMET,1300,
    DRIVE,FORWARDS,180,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,500,//TIMET,1300,
    DRIVE,FORWARDS,550,0,NOPID,44,
    DRIVE,TURN,3120,NOSTRAIGHT,TURNPID,
    DRIVE,BACKWARDS,1080,3120,REGACCEL,
    DRIVE,TURN,0,NOSTRAIGHT,TURNPID,
    DRIVE,FORWARDS,150,0,MMREGPID,50,100,REGACCEL,NOBRAKE,
    DRIVE,FORWARDS,970,0,NOPID,40,
    DRIVE,BACKWARDS,550,0,
    DRIVE,TURN,1300,NOSTRAIGHT,TURNPID2,
    DRIVE,FORWARDS,340,1350,REGACCEL,NOBRAKE,MMREGPID,60,127,
    DRIVE,FORWARDSE,60,1350,TIMETE,450,
    INTAKE,OUT,127,LIFTT,1,800,
    INTAKE,IN,127,TIMET,1100,
    INTAKE,IN,45,DRIVET,10,15,
    TILTER,POSITION,1000,90,DRIVET,10,10
    );
addCommands(
    INTAKE,OUT,0,
    INTAKE,OUT,36,TILTERT,1,1400,
    INTAKE,OUT,0,TILTERT,1,3800,
    TILTER,POSITION,6700,127,
    TILTER,POSITION,7100,90
    );
addCommands(
    INTAKE,OUT,127,
    INTAKE,IN,0,TIMET,1000,
    DRIVE,BACKWARDS,1000,NOSTRAIGHT,TIMET,280,
    TILTER,POSITION,5000,127,TIMET,50
);
}

void microCube()
{
addCommands(
    LIFT,POSITION,1700,127,
    LIFT,SPEED,-127,TIMETE,600,
    DRIVE,FORWARDS,250,0,TIMET,600,
    DRIVE,FORWARDS,-1,0,NOBRAKE,LIFTT,2,600,//TIMET,1300,
    DRIVE,FORWARDS,180,0,MMREGPID,55,127,REGACCEL,NOBRAKE,TIMET,500,//TIMET,1300,
    DRIVE,BACKWARDS,500,0,
    INTAKE,OUT,127,LIFTT,1,800,
    INTAKE,OUT,127,TIMET,3000
);
}

void posTest()
{
    posObj.reset();
    while(true)
    {
        posObj.updatePosition();
        pros::lcd::print(2,"%f", posObj.xPosition);
        pros::lcd::print(3,"%f", posObj.yPosition);
        driveMotorsSpeed(cVal(ANALOG_RIGHT_Y),rightDrive);
		driveMotorsSpeed(cVal(ANALOG_LEFT_Y),leftDrive);
        pros::delay(1);
    }
}

void autonomous()
{
    resetAutonVals();
    //posTest();
    switch(count)
    {
        case(SMALLRED):
            smallRed();
            break;
        case(SMALLBLUE):
            smallBlue();
            break;
        case(THICCRED):
            bigRed();
            break;
        case(THICCBLUE):
            bigBlue();
            break;
        case(MICROCUBE):
            microCube();
            break;
    }
}

//pros::lcd::print(2,"%d", drive->speed);
//pros::lcd::print(3,"%d", drive->pid.maxOutput);
//pros::lcd::print(4,"%f", gyro.get_value());
