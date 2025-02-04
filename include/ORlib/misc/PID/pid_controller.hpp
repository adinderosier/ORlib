/**
 * @file pid_controller.hpp
 * @author Adin De'Rosier
 * @brief Header file for the PIDController class.
 * @version 1.0
 * @date 2025-02-04
 *
 * This file defines the PIDController class, which implements a
 * Proportional-Integral-Derivative controller.
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

        // The output values for the PID controller
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
         * @param measured Measured value.
         * @return Error value.
         */
        float calculateError(const float target, const float measured) const;

    public:
        /**
         * @brief Default constructor for the PIDController class.
         */
        PIDController();

        /**
         * @brief Constructor for the PIDController class.
         */
        PIDController(const float p, const float i, const float d,
                      const float minOut = 0.0, const float maxOut = 6.0,
                      const float maxInt = 0.01, const bool isAngle = false,
                      const float settleError = 1, const float settleTime = 10,
                      const float timeout = 5000);

        /**
         * @brief Sets the proportional constant.
         */
        void setP(const float p);

        /**
         * @brief Sets the integral constant.
         */
        void setI(const float i);

        /**
         * @brief Sets the derivative constant.
         */
        void setD(const float d);

        /**
         * @brief Sets the constants for the PID controller.
         */
        void setConstants(const float p, const float i, const float d);

        /**
         * @brief Gets the proportional component of the PID output.
         */
        float getPComponent() const;

        /**
         * @brief Gets the integral component of the PID output.
         */
        float getIComponent() const;

        /**
         * @brief Gets the derivative component of the PID output.
         */
        float getDComponent() const;

        /**
         * @brief Calculates the PID output.
         */
        virtual float getOutput(const float target, const float current, const float dt);

        /**
         * @brief Checks if the PID controller has settled.
         */
        virtual bool isSettled() const;

        /**
         * @brief Resets the PID controller.
         */
        void reset();
    };
} // namespace ORlib::misc::PID

#endif // _PID_CONTROLLER_HPP_
