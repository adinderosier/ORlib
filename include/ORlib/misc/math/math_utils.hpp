/**
 * @file math_utils.hpp
 * @brief Contains utility functions for mathematical operations.
 * @author Adin De'Rosier (adin.derosier@oit.edu)
 * @version 0.3
 * @date 2023-09-27
 *
 * @details
 * This file provides a collection of utility functions for common
 * mathematical operations.
 *
 * The functions include angle bounding, value comparison within a
 * percentage margin, value clamping, and deadband checking for both
 * scalar and vector values.
 *
 * @note
 * This file depends on the "vector2d.hpp" header file.
 *
 * @warning
 * The functions in this file assume that the input values are within
 * the expected range. No bounds checking is performed.
 *
 * @remark
 * This file is part of the ORlib::misc::math namespace.
 *
 * @remark
 * The functions in this file are inline for performance optimization.
 */

#ifndef _MATH_UTILS_HPP_
#define _MATH_UTILS_HPP_

#include <array>
#include <cmath>

#include "ORlib/misc/math/vector2d.hpp"

namespace ORlib::misc::math
{
    /**
     * @brief A small number used for floating point comparison
     *
     */
    const float SMALL_NUMBER = 0.00000001;

    inline float toVoltage(float percent)
    {
        return (percent * 12 / 100.0);
    }

    /**
     * @brief Converts an angle given in degrees to radians
     *
     * @param angle The angle in degrees to be converted
     * @return float The angle in radians
     */
    inline float toRadians(float angle)
    {
        return angle * M_PI / 180.0;
    }

    /**
     * @brief Wraps an angle given in degrees to the range
     *        [0, 360)
     *
     * @param angle The angle in degrees to be normalized.
     * @return float The wrapped angle in degrees
     */
    inline float wrapAngleDegrees(float angle)
    {
        while (angle < 0.0)
            angle += 360.0;

        while (angle >= 360.0)
            angle -= 360.0;

        return angle;
    }

    /**
     * @brief Wraps an angle given in radians to the range
     *        [0, 2*pi)
     *
     * @param angle The angle in radians to be normalized.
     * @return float The wrapped angle in radians
     */
    inline float wrapAngleRadians(float angle)
    {
        while (angle < 0.0)
            angle += 2.0 * M_PI;

        while (angle >= 2.0 * M_PI)
            angle -= 2.0 * M_PI;

        return angle;
    }

    /**
     * @brief Binds an angle given in degrees to the range
     *        [-180, 180]
     *
     * @param angle The angle in degrees to be normalized.
     * @return float The bound angle in degrees
     */
    inline float boundHalfAngleDegrees(float angle)
    {
        while (angle < -180.0)
            angle += 360.0;

        while (angle >= 180.0)
            angle -= 360.0;

        return angle;
    }

    /**
     * @brief Binds an angle given in radians to the range
     *        [-pi, pi]
     *
     * @param angle The angle in radians to be normalized.
     * @return float The bound angle in radians
     */
    inline float boundHalfAngleRadians(float angle)
    {
        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        while (angle >= M_PI)
            angle -= 2.0 * M_PI;

        return angle;
    }

    /**
     * @brief Binds an angle given in native motor ticks to the range
     *        [ @code -ticksPerRevolution / 2, @code ticksPerRevolution / 2]
     *
     * @param ticks The encoder count to adjust in native ticks
     * @param ticksPerRevolution The number of native encoder ticks per
     *        revolution.
     * @return float The bound angle in native ticks
     */
    inline float boundHalfAngleNative(float ticks, float ticksPerRevolution)
    {
        while (ticks < -ticksPerRevolution / 2.0)
            ticks += ticksPerRevolution;

        while (ticks >= ticksPerRevolution / 2.0)
            ticks -= ticksPerRevolution;

        return ticks;
    }

    /**
     * @brief Checks if a measured value is within a percentage margin
     *        of a target value
     *
     * @param target The target value
     * @param measured The measured value
     * @param percentage The allowable percentage margin
     * @return true if the measured value is within the percentage
     *         margin of the target value, false otherwise
     */
    inline bool equalWithinPercentMargin(float target, float measured, float percentage)
    {
        return std::abs(100 * (measured - target) / target) <= percentage;
    }

    /**
     * @brief Clamps a value between a specified minimum and maximum
     *        value
     *
     * @param value The value to be clamped
     * @param min The minimum allowed value. Default is -127.0
     * @param max The maximum allowed value. Default is 127.0
     * @return float The clamped value
     */
    inline float clamp(float value, float min = -127.0, float max = 127.0)
    {
        return std::max(min, std::min(max, value));
    }

    /**
     * @brief Checks if a value is outside of a specified deadband
     *
     * @param value The value to be checked
     * @param deadband The deadband threshold. Default is 0.1
     * @return true if the value is outside of the deadband, false
     *         otherwise
     */
    inline bool outOfDeadband(float value, float deadband = 0.1)
    {
        return std::abs(value) > deadband;
    }

    /**
     * @brief Checks if any component of a 2D vector is outside of a
     *        specified deadband
     *
     * @param value The 2D vector to be checked
     * @param deadband The deadband threshold for each component
     * @return true if any component of the vector is outside of the
     *         deadband, false otherwise
     */
    inline bool outOfDeadband(Vector2D value, Vector2D deadband)
    {
        return std::abs(value.x) > deadband.x || std::abs(value.y) > deadband.y;
    }
} // namespace ORlib::misc::math

#endif // _MATH_UTILS_HPP_
