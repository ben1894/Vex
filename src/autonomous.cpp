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
    virtual int getProgress(System &obj)
    {

    };
    SystemStates state = END;
    System(int id)
        : id(id)
    {
    }
    virtual void setMember(int &number, int value) = 0; //

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
    void setMember(int &number, int value);
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

    private:
    bool stopAcceleration = 0;
    bool stopDeacceleration = 0;
    bool noPID;
    bool turn;
    Both turnStats;
    Directions direction;
    unsigned int distance;
    int maxSpeed;
    int minSpeed;
    int target;
    int radius;
};

void Drive::setMember(int &number, int value)
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
                case(1):
                    distance = value;
                break;
            }
        }
        else if(direction == SWEEP) //sweep turns
        {
            switch(number)
            {
                case(1):
                    radius = value;
                    break;
                case(2):

            }
        }
        else      //regular turns
        {        
            if(gyroTurns == true) //with gyro
            {
                switch(number)
                {
                    case(1) :
                        target = value;
                    break;
                }
            }
            else //emergency no gyro turns
            {
                switch(number)
                {
                    case(1) :
                        distance = value;
                    break;
                }    
            }
        }
    }
}

class Lift : public System  //very quick acceleration
{ 
    private:
    Triggers trigger;
    SpeedControl speedControl;
    char triggerNumber;
    int target;
    int triggerBreak;
    int speed;
    //swint 
    public:
    Lift(int id = LIFT)
        : System((int)id)
    { 
    }
    void setMember(int &number, int value)
    {
        static int subNumber = 0;
        switch(number) //have a static sub counter that yeah.
        {
            case(0):
                if(value == DRIVET || trigger == DRIVET)
                {
                    switch(subNumber)
                    {
                        case(0):
                            trigger = (Triggers)value;
                            ++subNumber;
                            --number;
                            break;
                        case(1):
                            triggerNumber = value;
                            ++subNumber;
                            --number;
                            break;
                        case(2):
                            triggerBreak = value;
                            subNumber = 0;
                            break;
                    }
                }
                else if(value == TIME || trigger == TIME)
                {
                    switch(subNumber)
                    {
                        case(0):
                            trigger = (Triggers)value;
                            ++subNumber;
                            --number;
                            break;
                        case(1):
                            triggerBreak = value;
                            subNumber = 0;
                            break;
                    }
                }
                else if(value == NONE || trigger == NONE)
                {
                    trigger = (Triggers)value;
                }
                break;
            case(1):
                target = value;
                break;
            case(2):
                if(value == REGPID || speedControl == REGPID)
                {
                    speedControl = (SpeedControl)value;
                }
                else if(value == NOPID || speedControl == NOPID)
                {
                    switch(subNumber)
                    {
                        case(0):
                            speedControl = (SpeedControl)value;
                            ++subNumber;
                            --number;
                            break;
                        case(1):
                            triggerBreak = value;
                            subNumber = 0;
                            break;
                    }
                }
                break;

        }
    }
    void move();
};

/*
class Lift : public System  //very quick acceleration
{ 
    public:
    Lift(int id = LIFT)
        : System((int)id)
    {
    }
    void setMember(int &number, int value){}
    void move();
};
*/

//if eqal get encoder count to move. If less than dont do anything. If greater than

void Drive::move()
{

}

void Lift::move()
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

    Drive drive{};
    Lift lift{};

    for(int i = 0; i < parameters.size(); i++)
    {
        drive.initialUpdate(i, parameters);
        lift.initialUpdate(i, parameters);
    }

    while(lift.state != END || drive.state != END))
    {
        drive.move();
        lift.move();
        pros::delay(5);
        drive.update(parameters);
        lift.update(parameters);
    }

}

class PidController
{
    public:
        double P;
        double I;
        double D;
        unsigned char minOutput;
        unsigned char maxOutput;

        PidController(){reset();pros::delay(1);} //Prevent divide by 0 error 
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
    all(LIFT,0,9,9,9,9,9,9);
}
