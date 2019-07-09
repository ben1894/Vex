#include "main.h"
#include "forwardDeclairations.hpp"
bool autonTest = false;
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
/*a = A();   
 */

class drive
{ 
    public:
    int parameters[1] {};
    //Distance
    //
    void move()
    {
    };
};

class lift
{
    public:
    int parameters[1] {};
};

template <typename T>
void initialUpdate(T system, int &i, std::vector<int> &parameters)
{
    if(parameters[i] == systems.id)
    {
        if(system.state() == END)
        {
            while(parameters[i+1] > minEnumValue) //double check this +1
            {
                int x = 0;
                system.setMember(x,parameters[i]);
                ++i;
                ++x;
            }
            system.state = EXECUTINGINSTRUCTIONS;
        }
        else
        {
            system.multiple = true;
        }
    }
}

void checkForUpdate(T system, int &i, std::vector<int> &parameters)
{
    if(state != END)
    {
        if(parameters[i] == systems.id)
        {
            if(system.state() == END)
            {
                while(parameters[i+1] > minEnumValue)
                {
                    int x = 0;
                    system.setMember(x,parameters[i]);
                    ++i;
                    ++x;
                }
                system.state = EXECUTINGINSTRUCTIONS;
            }
            else
            {
                system.multiple = true;
            }
        }
    }
}

drive::actions
{
    if(state = END)
    {

    }
    else
    {
        if(conditionsaremet)
        {
            if(multiple == true)
            {
                state = WAITINGFORINSTRUCTIONS;
                multiple = false;
            }
            else
            {
                state = END;
            }
        }
    }
};

template <typename... Ts>
int all(Ts... all)
{
    std::vector<int> parameters = {all...};

    Drive drive;
    Claw claw;

    for(int i = 0; i < parameters.size(); i++)
    {
        initialUpdate(drive, i, parameters)
        initialUpdate(claw, i, parameters)
    }

    while(!(claw.state == END && drive.state == END))
    {
        drive.move();
        claw.move();
        pros::delay(5);
    }
    if(obj.state == WAITINGFORINSTRUCTIONS)
    {
        for(int i = 0; i < parameters.size(); i++)
        {
            if(parameters[i] == DRIVE)
            {
                if(drive.recievedInstructions() == 0)
                {
                    while(parameters[i+1] > minEnumValue)
                    {
                        int x = 0;
                        if()
                        obj.setMember(x,parameter[i]);
                        ++i;
                        ++x;
                    }
                }
            }
        }
    }
        scroll through
    }
    for(int i = 0; i < parameters.size(); i++)
    {
        if(parameters[i] == DRIVE)
        {
            if(drive.recievedInstructions() == 0)
            {
                while(parameters[i+1] > minEnumValue)
                {
                    int x = 0;
                    if()
                    obj.setMember(x,parameter[i]);
                    ++i;
                    ++x;
                }
            }
        }
    }
}

class pidController //fix divide by 0 error;
{
    public:
        double P;
        double I;
        double D;
        double minOutput;
        double maxOutput;

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

void autonomous() {}
