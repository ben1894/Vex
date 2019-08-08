
#include "main.h"
#include "forwardDeclairations.hpp"
#include "pidPacks.hpp"
bool autonTest = false;
const bool voltage = true;
const bool gyroTurns = true;
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
/*a = A();   //clears back to defaults
 */
class PidController
{
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
            reset();pros::delay(1); //Prevent divide by 0 error
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
    private:
        unsigned long lastTime;
        long combinedIntegral;
        int lastError;
};

class System
{
protected:
    char triggerNumber; //So it can be called on the second process of something
    int triggerBreak;   //
    System *triggerSystem;
    AutonFlags trigger; //Holds the name of the object that will trigger it                  All sub systems will have the option of starting on a trigger

    AutonFlags speedControl;
    PidController pid{};
    int target;

public:
    int id;
    char numberOfCalls = 0;
    virtual int getProgress() = 0;
    virtual void setMember(int &number, AutonFlags &currentFlag, int value) = 0; //pure virtual functions
    virtual void resetObj() = 0;                                                 //a system object is never going to exist
    virtual void initializePid(AutonFlags pidPack) = 0;

    SystemStates state = END;
    System(int idVal, double b, double c, double d, unsigned char e, unsigned char f): id(idVal), pid(b, c, d, e, f){}

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
                    initializePid(currentFlag);
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

                state = EXECUTINGINSTRUCTIONS;
            }
            else
            {
                ++numberOfCalls;
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
                    if(state == WAITINGFORINSTRUCTIONS)
                    {
                        resetObj();
                        parameters[i] = (int)NULLOPTION;
                        AutonFlags currentFlag;  //DRIVE,FORWARD,1000,LIFT,FLAG
                        int subFlag = 0;

                        do //loops through the data to give to the specific system
                        {
                            ++i;
                            bool checkSecondaryFlags = setSystemMember(subFlag,currentFlag,parameters[i]);
                            setMember(subFlag,currentFlag,parameters[i]);
                        } while(parameters[i+1] < minEnumValue);

                        state = EXECUTINGINSTRUCTIONS;
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
    bool turn;
    int target;
    int radius;
    Both turnStats;
    AutonFlags direction;

    public:
    Drive(int idVal = DRIVE)
        : System((int)idVal,regDriveP,regDriveI,regDriveD,regDriveMin,regDriveMax)
        {
            if(driveObj == nullptr)
            {
                driveObj = this;
            }
        };

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
        return;
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
                number++;
        }
    }
    else
    {
        switch(currentFlag)
        {
            case(FORWARDS):
            case(BACKWARDS):
            case(TURN):
                target = value; //distance
                number = 0;
                break;
            case(SWEEP):
                switch(number)
                {
                    case(1):
                        target = value; //distance
                        number++;
                        break;
                    case(2):
                        number = 0;
                        radius = value;
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
            if(liftObj == nullptr)
            {
                liftObj = this;
            }
        }

    void initializePid(AutonFlags pidPack)
    {
        pid = PidController(regLiftP,regLiftI,regLiftD,regLiftMin,regLiftMax);
    }

    void setMember(int &number, AutonFlags &currentFlag, int value)
    {}

    void move();
};

//if eqal get encoder count to move. If less than dont do anything. If greater than.. first check number

void Drive::move()
{
}

void Lift::move()
{

}
/*
drive::actions // or is equal to end
{
    if(state = EXECUTINGINSTRUCTIONS)
    {
        Do stuff
        if(conditionsaremet)
        {
            if(number > 0)
            {
                state = WAITINGFORINSTRUCTIONS;
                number--;
            }
            else
            {
                state = END;
            }
        }
    }
    else
    {
    }
};
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

    while(lift.state != END || drive.state != END)
    {
        drive.move();
        lift.move();
        pros::delay(5);
        drive.update(parameters);
        lift.update(parameters);
    }
}

void autonomous()
{
   // all(LIFT,0,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,99,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,DRIVE);
}
