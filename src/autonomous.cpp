#include "main.h"
#include "forwardDeclairations.hpp"
#include "pidPacks.hpp"
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
class PidController;
Drive *driveObj;
Tilter *tilterObj;
Intake *intakeObj;


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

class PidController
{
    private:
        unsigned long lastTime;
        long combinedIntegral;
        int lastError;

    public:
        double P;
        double I;
        double D;
        unsigned char minOutput;
        unsigned char maxOutput;

        PidController(double P = 0, double I = 0, double D = 0, unsigned char minOutput = 0, unsigned char maxOutput = 0) 
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

        int output(int currentSensorData, int target)
        {
            int error = currentSensorData - target;
            unsigned long changeInTime = lastTime - pros::millis();

            if((P*error) < maxOutput) //to prevent windup
            {
                long currentIntegral = error * changeInTime; //calculation for the area under the curve for the latest movement. The faster this updates the more accurate
                combinedIntegral += currentIntegral;         //adds latest to the total integral
            }
            
            double derivative = (error - lastError) / changeInTime; //formula to calculate the (almost) instantaneous rate of change.                                                 //affected by how quickly the rate is changing (maybe reduce rate)
            lastTime = pros::millis(); //put here before the returns
            lastError = error;

            int output = (P * error) + (I * combinedIntegral) + (D * derivative);
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
 
class System
{
protected:
    char triggerNumber; 
    int triggerBreak;
    System *triggerSystem;
    AutonFlags trigger = NONET; //Holds the type of trigger system will have    All sub systems will have the option of starting on a trigger
    Timer triggerTimer;

public:
    AutonFlags speedControl = BLANK;
    SystemStates state = END;
    PidController pid; //eventualy initialized to defaults later in inherited classes
    int id;
    int target;
    char numberOfCalls = 0;
    char totalNumberOfCalls = 0;
    
    System(int idVal): id(idVal){}
    virtual bool checkIfDone(int breakValue) = 0;
    virtual void setMember(int &number, AutonFlags &currentFlag, int value) = 0; 
    virtual void resetObj() = 0;                                                
    virtual void initializePid(AutonFlags pidPack) = 0;
    int getTriggerNumberProgress()
    {
        return (totalNumberOfCalls - numberOfCalls)+1; //number of calls decreases throughout runtime. //Starts at 1 and increases
    } //if 0 from beginning, initial update takes care of it
    void updateEndingState()
    {
        if(getTriggerNumberProgress() == 1) //if it was on its last run
        {
            state = END;
        }
        else
        {
            state = WAITINGFORINSTRUCTIONS;
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
                else if(triggerNumber > triggerSystem->getTriggerNumberProgress())
                {
                    state = EXECUTINGINSTRUCTIONS;
                    break;
                }
                else if(triggerNumber < triggerSystem->getTriggerNumberProgress())
                {
                    state = WAITINGFORTRIGGER;
                    break;
                }
                else //is equal and needs to be checked
                {
                    if(triggerSystem->checkIfDone(triggerBreak) == false)
                    {
                        state = EXECUTINGINSTRUCTIONS;
                        break;
                    }
                    state = WAITINGFORTRIGGER;
                    break;
                }
        }
    }

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
                case(REGPID):
                    speedControl = currentFlag;
                    initializePid(currentFlag); //pure virtual function
                    return false;
                case(DRIVEPID):
                    speedControl = currentFlag;
                    pid = PidController(regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax);
                    return false;
                case(TILTERPID):
                    speedControl = currentFlag;
                    pid = PidController(regTilterP,regTilterI,regTilterD,regTilterMin,regTilterMax);
                    return false;
                default:
                    return true; //dont plus one yet because it will go through the other flags
            }
        }
        else
        {
            switch(currentFlag)
            {
                case(DRIVET): //(int = target) or otherwise specified by ther break when testing for trigger
                case(TILTERT): //cant have a trigger based off the intake... yet (maybe idk I don't think there will be a use)
                    switch(number)
                    {
                        case(1):       //its never going to be 0 because it only goes in this loops if its not 0;
                            if(currentFlag == DRIVET)
                            {
                                triggerSystem = reinterpret_cast<System *>(driveObj);
                            }
                            else if(currentFlag == TILTERT)
                            {
                                triggerSystem = reinterpret_cast<System *>(tilterObj);
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
                case(NOPID):
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
            if(state == END)
            {
                parameters[i] = (int)NULLOPTION;
                AutonFlags currentFlag;
                int subFlag = 0;

                do
                {
                    ++i;
                    bool checkSecondaryFlags = setSystemMember(subFlag,currentFlag,parameters[i]);
                    if(checkSecondaryFlags == true)
                    {
                        setMember(subFlag,currentFlag,parameters[i]);
                    }
                } while(parameters[i+1] < minEnumValue);

                state = WAITINGFORTRIGGER; //changed because it wouldnt ever check trigger 
            }
            else
            {
                ++totalNumberOfCalls;
            }
        }
    }

    void update(std::vector<int> &parameters)
    {
        if(state == WAITINGFORINSTRUCTIONS)
        {
            numberOfCalls = 0;
            for(int i = 0; i < parameters.size()-1; i++)
            {
                if(parameters[i] == id)
                {
                    if(state == WAITINGFORINSTRUCTIONS) //Checks again just in case there are multiple calls.
                    {
                        resetObj(); //have to change this 
                        parameters[i] = (int)NULLOPTION;
                        AutonFlags currentFlag;  //DRIVE,FORWARD,1000,TILTER,FLAG
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
    bool stopAcceleration = false;
    bool stopDeacceleration = false;
    int correctTo = -1;
    int speed;
    int offSet; //used for sweep Turns
    GyroDistances turnStats;
    AutonFlags direction;

    public:
    double outerInnerRatio = 1;
    Drive(int idVal = DRIVE)
        : System((int)idVal)
        {
            pid = {regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax};
            leftEncoder.reset(); ///////////
	        rightEncoder.reset();
            if(driveObj == nullptr)
            {
                driveObj = this;
            }
            else
            {
                totalNumberOfCalls = driveObj->totalNumberOfCalls;
            }
        };

    bool checkIfDone(int breakVal = driveObj->target)
    {
        if(direction != TURN)
        {
            if(getDriveEncoder() > breakVal)
            {
                return true;
            }
            return false;
        }

        GyroDistances gyroVals;
        getDistances(gyroVals, breakVal);
        if(gyroVals.Left < gyroVals.Right)
        {
            if(gyroVals.Left < 4)
            {
                return true;
            }
            return false;
        }
        else
        {
            if(gyroVals.Left < 4)
            {
                return true;
            }
            return false;
        }
    }

    void gyroCorrections(float &leftCorrect, float &rightCorrect)
    {
        GyroDistances off;
        getDistances(off, correctTo);

        if(off.Right < off.Left) //keep!!
        {
            if(off.Right > 3)
            {
                rightCorrect *= ((float).97 - ((float)off.Right/(float)70));
            }
        }
        else
        {
            if(off.Left > 3)
            {
                leftCorrect *= ((float).97 - ((float)off.Left/(float)70)); //left decrease
            }
        }
    }

    int getOutsideEncoder()
    {
        switch(direction)
        {
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
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
                return abs(leftEncoder.get_value());
            default:
                return abs(rightEncoder.get_value());
        }
    }

    int getSweepDifference() //positive, outside is faster. negative, inside is faster
    {
        (double)getOutsideEncoder() - (double)(getInsideEncoder() + offSet);
    }

    void wheelCorrections(float &leftCorrect, float &rightCorrect)
    {
        int difference;
        if(direction == TURN)
        {
            difference = abs(rightEncoder.get_value()) - abs(leftEncoder.get_value());
        }
        else 
        {
            difference = getSweepDifference(); //reduces call amount
        }

        if(difference > 4)
        {
            rightCorrect *= ((float).97 - ((float)abs(difference)/(float)100));
        }
        else if(difference < -4)
        {
            leftCorrect  *= ((float).97 - ((float)abs(difference)/(float)100));
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
    int getCurrentDistance()
    {
        if(direction == TURN) 
        {
            getDistances(turnStats, target);
            if(turnStats.Left <= turnStats.Right)
            {
                return turnStats.Left;
            }
            return turnStats.Right;
        }
        return getDriveEncoder();
    }

    int getDriveEncoder()
    {
        switch(direction)
        {
            case(UPRIGHTSWEEP):
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
            case(DOWNRIGHTSWEEP):
                return getOutsideEncoder();
            default:
                return abs(leftEncoder.get_value()); //////////////////abs
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
                stopAcceleration = true;
                break;
            default:
                number++; //Just sets currentFlag, will get next values when increasead next time
        }
    }
    else
    {
        switch(currentFlag)
        {
            case(TURN):
                direction = currentFlag;
                target = value; //distance
                number = 0;
                break;
            case(FORWARDS):
            case(BACKWARDS):
                switch(number) //use same straight drive for turns and straight drive
                {
                    case(1): 
                        direction = currentFlag;
                        target = value;
                        number++;
                        break;
                    case(2):
                        if(value == CURRENTVAL)
                        {
                            correctTo = fixTarget(gyro.get_value());
                        }
                        else
                        {
                            correctTo = value;
                        }
                        number = 0;
                        break;
                }
                break;
            case(UPRIGHTSWEEP):
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
            case(DOWNRIGHTSWEEP):
                switch(number)
                {
                    case(1):
                        direction = currentFlag;
                        target = value; //distance
                        number++;
                        break;
                    case(2):
                        number = 0; //radius
                        outerInnerRatio = (value - wheelDistance) / value;
                        break;
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
        return tilter.get_position();
    }

    bool checkIfDone(int breakVal = tilterObj->target)
    {
        if(underTarget == true) //i
        {
            if(target < breakVal)
            {
                return true;
            }
            return false;
        }
        else
        {
            if(target > breakVal)
            {
                return true;
            }
            return false;
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
            }
        }
    }

    void move()
    {
        if(checkIfDone() == false)
        {
            if(getPosition() > target)
            {
                tilter.move(speed);
            }
            else
            {
                tilter.move(-speed);
            }
        }
        else 
        {
            updateEndingState();
        }
    }
};

class Intake : public System  //very quick acceleration
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

    bool checkIfDone(int breakVal = intakeObj->target)
    {
        return true;
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
        motorGroupMove(speed, intake);
        updateEndingState();
    }
};

void Drive::move()
{
    float leftCorrection = 1; //reset every call
    float rightCorrection = 1;
    if(checkIfDone() == false)
    {
        if(direction != TURN)
        {
            switch(speedControl) //pid or nah
            {
                case(NOPID):
                    speed = pid.maxOutput;
                    break; //reset integral at 0
                default:
                    speed = pid.output(getDriveEncoder(), target);
                    break;
            }
            
            switch(direction) //drive straight
            {
                case(UPLEFTSWEEP): 
                case(UPRIGHTSWEEP):
                case(DOWNLEFTSWEEP):
                case(DOWNRIGHTSWEEP):
                    wheelCorrections(leftCorrection, rightCorrection);
                    switch(direction) //sweep turn stuffz
                    {
                        case(UPLEFTSWEEP):
                        case(DOWNLEFTSWEEP):
                            leftCorrection *= outerInnerRatio;
                            break;
                        default:
                            rightCorrection *= outerInnerRatio;
                            break;
                    }
                    break;
                default:
                    if(correctTo != NOSTRAIGHT)
                    {
                        gyroCorrections(leftCorrection, rightCorrection);
                    }
                    break;
            }

            switch(direction)
            {
                case(DOWNLEFTSWEEP):
                case(DOWNRIGHTSWEEP):
                case(BACKWARDS):
                    leftCorrection *= -1;
                    rightCorrection *= -1;
                    break;
            }

            driveMotorsSpeed(speed*rightCorrection, rightDrive);
            driveMotorsSpeed(speed*leftCorrection, leftDrive);
        }
        else 
        {
            target = fixTarget(target); //so you can put in negative values
            GyroDistances Dist;
            getDistances(Dist, target);

            if(Dist.Right<Dist.Left)
            {
                rightCorrection *= -1;
                speed = pid.output(Dist.Left, target);
            }
            else 
            {
                leftCorrection *= -1;
                speed = pid.output(Dist.Right, target);
            }
            
            wheelCorrections(leftCorrection, rightCorrection);
            
            driveMotorsSpeed(speed*leftCorrection,leftDrive);
            driveMotorsSpeed(speed*rightCorrection,rightDrive);
        }
    }
    else 
    {
        updateEndingState();
    }
}

template <typename... Ts>
void all(Ts... input)
{
    std::vector<int> parameters = {(int)input...,NULLOPTION};

    Drive drive{};
    Tilter tilter{};

    for(int i = 0; i < parameters.size()-1; i++)
    {
        drive.initialUpdate(i, parameters);
        tilter.initialUpdate(i, parameters);
    }
    drive.pid.minOutput /= drive.outerInnerRatio;

    while(tilter.state != END || drive.state != END)
    {
        switch(drive.state)
        {
            case(WAITINGFORINSTRUCTIONS):
                drive.update(parameters);
                drive.pid.minOutput /= drive.outerInnerRatio; //has to be put here so it doesn't get overwritten by the pid initialization
                if(drive.speedControl == BLANK)
                {
                    drive.initializePid(REGPID); //if left default
                }
                break;
            case(WAITINGFORTRIGGER):
                drive.updateTriggerState();
                break;
            case(EXECUTINGINSTRUCTIONS):
                drive.move();
                break;
            case(END):
                driveMotorsSpeed(0,rightDrive);
                driveMotorsSpeed(0,leftDrive);
                break; //adding retain position later
        }

        switch(tilter.state)
        {
            case(WAITINGFORINSTRUCTIONS):
                tilter.update(parameters);
                if(tilter.speedControl == BLANK)
                {
                    tilter.initializePid(REGPID); //if left default
                }
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
                break; //adding retain position later
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
                break; //adding retain position later
        }

        pros::delay(5);
    }
}

//DRIVE, distance, correctTo(value,CURRENTVAL,NOSTRAIGHT,WHEELCORRECTION), {speedControl - REGPID(NOPID, MMPID)}

void autonomous()
{

    all(DRIVE,FORWARDS,1000,NOSTRAIGHT);
}
