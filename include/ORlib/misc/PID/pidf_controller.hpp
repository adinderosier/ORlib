/**
 * @file pidf_controller.hpp
 * @author Adin De'Rosier (adin.derosier@oit.edu)
 * @brief Header file for the PIDFController class.
 * @version 0.2
 * @date 2023-09-27
 *
 * This file defines the PIDFController class, which implements a
 * Proportional-Integral-Derivative-Feedback controller.
 *
 * The class provides methods to set the controller constants, calculate
 * the PIDF output based on the target, current value, and time
 * difference, and reset the controller.
 *
 * The PIDFController class supports managing both angle and non-angle
 * targets.
 *
 * The class also includes private member variables to store the
 * controller constants, max and min output values, max accumulated
 * value for the integral term, and previous error and integral values.
 */

#ifndef _PIDF_CONTROLLER_HPP_
#define _PIDF_CONTROLLER_HPP_

namespace ORlib::misc::PID
{
    class PIDFController : protected PIDController
    {
    private:
        // The PIDF constants
        float kF_;

        float fOut_;

    public:
        /**
         * @brief Default constructor for the PIDFController class.
         * 
         */
        PIDFController();

        /**
         * @brief Constructor for the PIDFController class.
         * @param isAngle Flag indicating whether the target is an
         *        angle.
         */
        PIDFController(bool isAngle);

        /**
         * @brief Constructor for the PIDFController class.
         * @param p Proportional constant.
         * @param i Integral constant.
         * @param d Derivative constant.
         * @param f Feedforward constant.
         * @param minOut Minimum output value.
         * @param maxOut Maximum output value.
         * @param maxInt Maximum integral value.
         * @param isAngle Flag indicating whether the target is an
         *        angle.
         */
        PIDFController(float p, float i, float d, float f, float minOut,
                       float maxOut, float maxInt, bool isAngle);

        /**
         * @brief Gets the feedforward component of the PID output.
         * @return Feedforward component.
         */
        float getFComponent();

        /**
         * @brief Calculates the PID output based on the target, current
         *        value, and time difference.
         * @param target Desired target value.
         * @param current Current value.
         * @param dt Time difference (delta time).
         * @return PID output.
         */
        float getOutput(float target, float current, float dt);

    };
} // namespace ORlib::misc::PID

#endif // _PIDF_CONTROLLER_HPP_