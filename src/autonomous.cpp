#include "main.h"
#include "forwardDeclairations.hpp"
bool autonTest = false;
const bool voltage = true;
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
    bool multiple = false;
    systemStates state = END;
    System(int id)
        : id(id)
    {
    }
    virtual void setMember(int number, int value){} 

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
                multiple = true;
            }
        }
    }

    void update(std::vector<int> &parameters)
    {
        if(state == WAITINGFORINSTRUCTIONS)
        {
            for(int i = 0; i < parameters.size(); i++)
            {
                if(parameters[i] == id)
                {
                    if(state == WAITINGFORINSTRUCTIONS)
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
                        multiple = true;
                    }
                }
            }
        }
    }
}; 

class Drive : public System
{ 
    public:
    Drive(int id)
        : System(id)//, // call Person(std::string, int) to initialize these fields   //m_battingAverage(battingAverage), m_homeRuns(homeRuns) to initialize Drive members
    {
    }
    void setMember(int number, int value){}
    void move(){}
};

class Claw : public System
{ 
    public:
    Claw(int id)
        : System(id)//, // call Person(std::string, int) to initialize these fields   //m_battingAverage(battingAverage), m_homeRuns(homeRuns) to initialize Drive members
    {
    }
    void setMember(int number, int value){}
    void move(){}
};
/*
drive::actions
{
    if(state = EXECUTINGINSTRUCTIONS)
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
    else
    {
    }
};
*/
template <typename... Ts>
int all(Ts... all)
{
    std::vector<int> parameters = {all...};

    Drive drive{};
    Claw claw{};

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
