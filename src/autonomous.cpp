#include "main.h"
#include "forwardDeclairations.hpp"
bool autonTest = false;
const bool voltage = true;
const bool gyroTurns = true;
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
class System 
{ 
public: 
    const int id;
    char numberOfCalls = 0;
    SystemStates state = END;
    System(int id)
        : id(id)
    {
    }
    virtual void setMember(int number, int value) = 0; 

    void initialUpdate(int &i, std::vector<int> &parameters)
    {
        if(parameters[i] == id)
        {
            if(state == END)
            {
                parameters[i] = (int)NULLOPTION;
                while(parameters[i+1] > minEnumValue)
                {
                    int x = 0;
                    setMember(x,parameters[i]);
                    ++i;
                    ++x;
                }
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
            for(int i = 0; i < parameters.size(); i++)
            {
                if(parameters[i] == id)
                {
                    if(state == WAITINGFORINSTRUCTIONS)
                    {
                        parameters[i] = (int)NULLOPTION;
                        while(parameters[i+1] > minEnumValue) //plus one here to it doesn't loop over the next value. 
                        {
                            int x = 0;
                            setMember(x,parameters[i]);
                            ++i;
                            ++x;
                        }
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
    public:
    Drive(int id = DRIVE)
        : System((int)id)//, // call Person(std::string, int) to initialize these fields   //m_battingAverage(battingAverage), m_homeRuns(homeRuns) to initialize Drive members
    {
    }
    bool stopAcceleration = 0;
    bool stopDeacceleration = 0;
    Directions direction;
    bool noPID;
    int distanceToGo;
    int maxSpeed;
    int minSpeed;
    void setMember(int number, int value);
    void move();
};

void Drive::setMember(int number, int value)
{
    if(number == 0)
    {
        direction = (Directions)value;
    }
    else
    {
        if(direction == FORWARDS || direction == BACKWARDS)
        {
            switch(number)
            {
                case(1) :

                break;
            }
        }
        else
        {
            if(gyroTurns == true)
            {
                switch(number)
                {
                    case(1) :

                    break;
                }
            }
            else
            {
                switch(number)
                {
                    case(1) :

                    break;
                }    
            }
        }
    }
}

class Claw : public System
{ 
    public:
    Claw(int id = CLAW)
        : System((int)id)
    {
    }
    void setMember(int number, int value){}
    void move();
};

class Claw : public System
{ 
    public:
    Claw(int id = CLAW)
        : System((int)id)
    {
    }
    void setMember(int number, int value){}
    void move();
};



//if eqal get encoder count to move. If less than dont do anything. If greater than

void Drive::move()
{

}

void Claw::move()
{

}
/*
drive::actions
{
    if(state = EXECUTINGINSTRUCTIONS)
    {
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
    std::vector<int> parameters = {(int)all...};

    Drive drive{DRIVE};
    Claw claw{CLAW};

    for(int i = 0; i < parameters.size(); i++)
    {
        drive.initialUpdate(i, parameters);
        claw.initialUpdate(i, parameters);
    }

    while(!(claw.state == END && drive.state == END))
    {
        drive.move();
        claw.move();
        pros::delay(5);
        drive.update(parameters);
        claw.update(parameters);
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

        pidController(){reset();pros::delay(1);} //Prevent divide by 0 error 
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

void autonomous()
{
    all(CLAW,0,9,9,9,9,9,9);
}
