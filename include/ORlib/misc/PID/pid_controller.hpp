/**
 * @file pid_controller.hpp
 * @author Adin De'Rosier (adin.derosier@oit.edu)
 * @brief Header file for the PIDController class.
 * @version 0.2
 * @date 2023-09-27
 *
 * This file defines the PIDController class, which implements a
 * Proportional-Integral-Derivative controller.
 *
 * The class provides methods to set the controller constants, calculate
 * the PID output based on the target, current value, and time
 * difference, and reset the controller.
 *
 * The PIDController class supports managing both angle and non-angle
 * targets.
 *
 * The class also includes private member variables to store the
 * controller constants, max and min output values, max accumulated
 * value for the integral term, and previous error and integral values.
 */

#ifndef _PID_CONTROLLER_HPP_
#define _PID_CONTROLLER_HPP_

namespace ORlib::misc::PID
{
    class PIDController
    {
    protected:
        // The PID constants
        float kP_; 
        float kI_; 
        float kD_; 

        // The max and min values output
        float kMin_;
        float kMax_;

        // The max accumulated value for the I term
        float kIMax_; 

        // If true, we are managing an angle
        bool isAngle_; 

        // If true, we have a last error value
        bool hasLastError_; 

        // The last error
        float lastError_; 

        // The accumulated I term
        float integral_; 

        // The value at which to begin integration
        float startI_;

        float dOut_; 
        float iOut_; 
        float pOut_; 

        // The error values for settling
        float settleError_;
        float settleTime_;
        float timeSpentSettled_;
        float timeSpentRunning_;
        float timeout_;

        /**
         * @brief Calculates the error between the target and current 
         *        value.
         * @param target Desired target value.
         * @param current Current value.
         * @return Error value.
         */
        float calculateError(float target, float current);

    public:
        /**
         * @brief Default constructor for the PIDController class.
         * 
         */
        PIDController();

        /**
         * @brief Constructor for the PIDController class.
         * @param isAngle Flag indicating whether the target is an 
         *        angle.
         */
        PIDController(bool isAngle);

        /**
         * @brief Constructor for the PIDController class.
         * @param p Proportional constant.
         * @param i Integral constant.
         * @param d Derivative constant.
         * @param minOut Minimum output value.
         * @param maxOut Maximum output value.
         * @param maxInt Maximum integral value.
         * @param isAngle Flag indicating whether the target is an 
         *        angle.
         */
        PIDController(float p, float i, float d, float minOut,
                      float maxOut, float maxInt, bool isAngle,
                      float startI, float settleError, float settleTime, 
                      float timeout);

        /**
         * @brief Sets the proportional constant.
         * @param p Proportional constant value.
         */
        void setP(float p);

        /**
         * @brief Sets the integral constant.
         * @param i Integral constant value.
         */
        void setI(float i);


        /**
         * @brief Sets the derivative constant.
         * @param d Derivative constant value.
         */
        void setD(float d);

        /**
         * @brief Gets the proportional component of the PID output.
         * @return Proportional component.
         */
        float getPComponent() const;

        /**
         * @brief Gets the integral component of the PID output.
         * @return Integral component.
         */
        float getIComponent() const;

        /**
         * @brief Gets the derivative component of the PID output.
         * @return Derivative component.
         */
        float getDComponent() const;

        /**
         * @brief Calculates the PID output based on the target, current 
         *        value, and time difference.
         * @param target Desired target value.
         * @param current Current value.
         * @param dt Time difference (delta time).
         * @return PID output.
         */
        virtual float getOutput(float target, float current, float dt);

        /**
         * @brief Checks if the PID controller has settled.
         * @return True if the PID controller has settled, false 
         *         otherwise.
         */
        virtual bool isSettled();

        /**
         * @brief Resets the PID controller by clearing the error 
         *        history and integral.
         */
        void reset();
    };
} // namespace ORlib::misc::PID

#endif // _PID_CONTROLLER_HPP_