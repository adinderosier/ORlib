/**
 * @file pida_controller.hpp
 * @author Adin De'Rosier (adin.derosier@oit.edu)
 * @brief Header file for the PIDAController class.
 * @version 0.2
 * @date 2023-09-27
 *
 * This file defines the PIDAController class, which implements a
 * Proportional Integral Derivative Acceleration controller.
 *
 * The class provides methods to set the controller constants, calculate
 * the PIDA output based on the target, current value, and time
 * difference, and reset the controller.
 *
 * The PIDAController class supports managing both angle and non-angle
 * targets.
 *
 * The class also includes private member variables to store the
 * controller constants, max and min output values, max accumulated
 * value for the integral term, and previous error and integral values.
 */

#ifndef _PIDA_CONTROLLER_HPP_
#define _PIDA_CONTROLLER_HPP_

namespace ORlib::misc::PID
{
    class PIDAController : protected PIDController
    {
    private:
        // The PIDA constants
        float kA_;
        float kV_;

        float aOut_;
        float vOut_;

    public:
        /**
         * @brief Default constructor for the PIDAController class.
         *
         */
        PIDAController();

        /**
         * @brief Constructor for the PIDAController class.
         * @param p Proportional constant.
         * @param i Integral constant.
         * @param d Derivative constant.
         * @param a Acceleration constant.
         * @param v Velocity constant.
         * @param minOut Minimum output value.
         * @param maxOut Maximum output value.
         * @param maxInt Maximum integral value.
         * @param isAngle Flag indicating whether the target is an
         *        angle.
         */
        PIDAController(float p, float i, float d, float a, float v,
                       float minOut, float maxOut, float maxInt, bool isAngle);

        /**
         * @brief Gets the acceleration component of the PID output.
         * @return Acceleration component.
         */
        float getAComponent() const;

        /**
         * @brief Gets the velocity component of the PID output.
         * @return Velocity component.
         */
        float getVComponent() const;

        /**
         * @brief Calculates the PID output based on the target, current
         *        value, and time difference.
         * @param target Desired target value.
         * @param current Current value.
         * @param dt Time difference (delta time).
         * @return PID output.
         */
        float getOutput(float target, float current, float a, float v, float dt);

    };
} // namespace ORlib::misc::PID

#endif // _PIDA_CONTROLLER_HPP_