#include "main.h"
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
}




     currentTime = _getSystemTime();

      //Calculate time since last tick() cycle.
      long deltaTime = currentTime - lastTime;

      //Calculate the integral of the feedback data since last cycle.
      int cycleIntegral = (lastError + error / 2) * deltaTime;

      //Add this cycle's integral to the integral cumulation.
      integralCumulation += cycleIntegral;

      //Calculate the slope of the line with data from the current and last cycles.
      cycleDerivative = (error - lastError) / deltaTime;

      //Save time data for next iteration.
      lastTime = currentTime;

         if(integralCumulation > maxCumulation) integralCumulation = maxCumulation;
    if(integralCumulation < -maxCumulation) integralCumulation = -maxCumulation;
        output = (int) ((error * _p) + (integralCumulation * _i) + (cycleDerivative * _d));
  lastFeedback = currentFeedback;
    lastError = error;
      if(outputBounded)
    {
      if(output > outputUpperBound) output = outputUpperBound;
      if(output < outputLowerBound) output = outputLowerBound;
    }

    _pidOutput(output);

void autonomous() {}
