/**
 * @file odom.hpp
 * @author Adin De'Rosier
 * @brief Header file for the class that defines an odometry controller.
 * @version 1.1
 * @date 2025-02-10
 *
 * This file defines the drivetrain.
 */

#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include "ORlib/misc/math/math_utils.hpp"

namespace ORlib::misc::Odometry
{
    class OdometryController
    {
    private:
        float forward_tracker_center_distance_;
        float sideways_tracker_center_distance_;
        float forward_tracker_position_;
        float sideways_tracker_position_;
        float x_position_;
        float y_position_;
        float heading_deg_;

    public:
        // Constructor
        OdometryController(float forwardTrackerCenterDistance = 0.0f, float sidewaysTrackerCenterDistance = 0.0f);

        // Getters
        [[nodiscard]] float getX() const noexcept { return x_position_; }
        [[nodiscard]] float getY() const noexcept { return y_position_; }
        [[nodiscard]] float getOrientation() const noexcept { return heading_deg_; }
        [[nodiscard]] float getForwardTrackerPosition() const noexcept { return forward_tracker_position_; }
        [[nodiscard]] float getSidewaysTrackerPosition() const noexcept { return sideways_tracker_position_; }
        [[nodiscard]] float getForwardTrackerCenterDistance() const noexcept { return forward_tracker_center_distance_; }
        [[nodiscard]] float getSidewaysTrackerCenterDistance() const noexcept { return sideways_tracker_center_distance_; }

        // Setters
        void setPosition(float x, float y, float orientation, float forwardTrackerPos, float sidewaysTrackerPos) noexcept;
        void updatePosition(float forwardTrackerPos, float sidewaysTrackerPos, float heading) noexcept;
        void setPhysicalDistances(float forwardTrackerCenterDistance, float sidewaysTrackerCenterDistance) noexcept;
    };
}

#endif // _ODOMETRY_HPP_
