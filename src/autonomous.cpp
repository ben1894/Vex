
#include "main.h"
#include "forwardDeclairations.hpp"
#include "pidPacks.hpp"
bool autonTest = false;
const bool voltage = false;
const bool gyroTurns = true;
const double wheelDistance = 200;
class Drive;
class Lift;
class PidController;
Drive *driveObj;
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
//a = A();   //clears back to defaults

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
        {
            this->P = P;
            this->I = I;
            this->D = D;
            this->minOutput = minOutput;
            this->maxOutput = maxOutput;
            reset();
            pros::delay(1); //Prevent divide by 0 error
        }

        void reset()
        {
            lastTime = pros::millis();
            combinedIntegral = 0;
            lastError = 0;
        }

        int output(int currentSensorData, int target)
        {
            int error = currentSensorData - target;
            unsigned long changeInTime = lastTime - pros::millis();

            long currentIntegral = error * changeInTime;
            combinedIntegral += currentIntegral;
            double derivative = (error - lastError) / changeInTime;

            lastTime = pros::millis();
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
    char triggerNumber; //So it can be called on the second process of something
    int triggerBreak;   //
    System *triggerSystem;
    AutonFlags trigger; //Holds the type of trigger system will have    All sub systems will have the option of starting on a trigger
    Timer triggerTimer;

    AutonFlags speedControl;
    PidController pid; //eventualy initialized to defaults later in inherited classes
    int target;

    SystemStates state = END;

public:
    int id;
    char numberOfCalls = 0;
    char totalNumberOfCalls = 0;
    
    System(int idVal, double b, double c, double d, unsigned char e, unsigned char f): id(idVal), pid(b, c, d, e, f){}
    virtual int getProgress() = 0; //return absolute value of difference
    virtual void setMember(int &number, AutonFlags &currentFlag, int value) = 0; //pure virtual functions
    virtual void resetObj() = 0;                                                 //a system object is never going to exist
    virtual void initializePid(AutonFlags pidPack) = 0;
    virtual bool checkSystemTrigger() = 0; 
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
        switch(triggerBreak)
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
                    if(triggerSystem->getProgress() >= triggerBreak)
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
                    initializePid(currentFlag); //virtual function
                    return false;
                case(DRIVEPID):
                    speedControl = currentFlag;
                    pid = PidController(regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax);
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
                case(LIFTT):
                    switch(number)
                    {
                        case(1):       //its never going to be 0 because it only goes in this loops if its 0;
                            if(currentFlag == DRIVET)
                            {
                                triggerSystem = reinterpret_cast<System *>(driveObj);
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
                    setMember(subFlag,currentFlag,parameters[i]);
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
                        AutonFlags currentFlag;  //DRIVE,FORWARD,1000,LIFT,FLAG
                        int subFlag = 0;

                        do //loops through the data to give to the specific system
                        {
                            ++i;
                            bool checkSecondaryFlags = setSystemMember(subFlag,currentFlag,parameters[i]);
                            setMember(subFlag,currentFlag,parameters[i]);
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
    int target;
    double outerInnerRatio = 1;
    int speed;
    int offSet; //used for sweep Turns
    GyroDistances turnStats;
    AutonFlags direction;

    public:
    Drive(int idVal = DRIVE)
        : System((int)idVal,regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax)
        {
            if(driveObj == nullptr)
            {
                driveObj = this;
            }
            else
            {
                totalNumberOfCalls = driveObj->totalNumberOfCalls;
            }
            
        };
    
    void gyroCorrections(float &leftCorrect, float &rightCorrect)
    {
        GyroDistances off;
        getDistances(off, correctTo);

        if(off.Right < off.Left)
        {
            if(off.Right > 3)
            {
                rightCorrect = ((float).97 - ((float)off.Right/(float)70));          /////////////////////
                leftCorrect  = 1;
            }
            else
            {
                rightCorrect  = 1;
                leftCorrect = 1;
            }
        }
        else
        {
            if(off.Left > 3)
            {

                leftCorrect = ((float).97 - ((float)off.Left/(float)70)); //left decrease
                rightCorrect = 1;
            }
            else
            {
                leftCorrect  = 1;
                rightCorrect = 1;
            }
        }
    }

    int getOutsideEncoder()
    {
        switch(direction)
        {
            case(UPLEFTSWEEP):
            case(DOWNLEFTSWEEP):
                return abs(rigthEncoder.get_value());
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
                return abs(rigthEncoder.get_value());
        }
    }

    int getSweepDifference() //positive, outside is faster. negative, inside is faster
    {
        (double)getOutsideEncoder() - (double)(getInsideEncoder() + offSet);
    }

    void wheelCorrections(float &leftCorrect, float &rightCorrect)
    {
        int difference = getSweepDifference(); //reduces call amount
        if(difference > 4)
        {
            rightCorrect = ((float).97 - ((float)abs(difference)/(float)100));
            leftCorrect  = 1;
        }
        else if(difference < 4)
        {
            rightCorrect = 1;
            leftCorrect  = ((float).97 - ((float)abs(difference)/(float)100));
        }
        else
        {
            leftCorrect = 1;
            rightCorrect = 1;
        }
    }

    int getProgress()
    {
        return abs(getDriveEncoder() - target);
    }

    bool checkSystemTrigger()
    {
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
                return leftEncoder.get_value();
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
                        correctTo = value;
                        number = 0;
                        break;
                }
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
        }
    }
}

class Lift : public System  //very quick acceleration
{
    private:
    int target;
    int speed;
    AutonFlags speedControl;

    public:
    Lift(int id = LIFT)
        : System((int)id,regLiftP,regLiftI,regLiftD,regLiftMin,regLiftMax)
        {
            if(liftObj == nullptr) //if declairing this for the first time
            {
                liftObj = this;
            }
            else //global is declaired meaning the object is being reset. Keep the state and total number of calls.
            {
                totalNumberOfCalls = liftObj->totalNumberOfCalls;
            }
        }

    int getProgress()
    {
    }

    bool checkSystemTrigger()
    {
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
    {}

    void move();
};

void Drive::move()
{
    float leftCorrection = 1;
    float rightCorrection = 1;
    if(getProgress() <= target)
    {
        switch(speedControl) //pid or nah
        {
            case(NOPID):
                speed = pid.maxOutput;
                break;
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
                gyroCorrections(leftCorrection, rightCorrection);
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
        updateEndingState();
    }
}

void Lift::move()
{
    
}

/*
drive::actions // or is equal to end
{
        Do stuff
        if(conditionsaremet)
        {
            if(getNumberofCalls() == totalNumberOfCalls)
            {
                totalNumberOfCalls
                state = END;
            }
            else
            {
                state = WAITINGFORINSTRUCTIONS
            }
*/
template <typename... Ts>
void all(Ts... all)
{
    std::vector<int> parameters = {(int)all...,NULLOPTION};

    Drive drive{};
    Lift lift{};

    for(int i = 0; i < parameters.size()-1; i++)
    {
        drive.initialUpdate(i, parameters);
        lift.initialUpdate(i, parameters);
    }
    drive.pid.minOutput /= drive.outerInnerRatio;
    while(lift.state != END || drive.state != END)
    {
        switch(drive.state)
        {
            case(EXECUTINGINSTRUCTIONS):
                drive.move();
                break;
            case(WAITINGFORTRIGGER):
                drive.updateTriggerState();
                break;
            case(WAITINGFORINSTRUCTIONS):
                drive.update(parameters);
                drive.pid.minOutput /= drive.outerInnerRatio; //has to be put here so it doesn't get overwritten by the pid initialization
                break;
            case(END):
                break; //adding retain position later
        }

        switch(lift.state)
        {
            case(EXECUTINGINSTRUCTIONS):
                lift.move();
                break;
            case(WAITINGFORTRIGGER):
                lift.updateTriggerState();
                break;
            case(END):
                break; //adding retain position later
            case(WAITINGFORINSTRUCTIONS):
                lift.update(parameters);
                break;
        }

        pros::delay(5);
    }
}

void autonomous()
{
    all(DRIVE,BACKWARDS,1000,10);
}
