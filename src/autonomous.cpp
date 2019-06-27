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

std::vector<std::string> parameters;
enum current;
enum old;

bool Output() 
{
    do the main stuff here
}
 
template<typename First, typename ... parameters> 
void Output(First arg, const parameters&... rest)
{
    static type hi;
    static type hi;
    static currentEnum;
    static prevEnum;
    static int prevArrayLoc;
    if (typeid(arg) == typeid(enum))
    {
        currentEnum = arg;
    }
    else
    {
        switch(currentEnum)
        {
        }
    }
    parameters.push_back(arg);
    Output(rest...);
}
 
int main() {
    Output("I","am","a","sentence");
    //Output("Let's","try",1,"or",2,"digits");
    return 0;
}
class pidController 
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
