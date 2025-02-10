/**
 * @file drivetrain.hpp
 * @author Adin De'
 * @brief Header file for the abstract class that defines a drivetrain.
 * @version 1.0
 * @date 2025-02-04
 *
 * This file defines the drivetrain.
 */

#ifndef _DRIVETRAIN_HPP_
#define _DRIVETRAIN_HPP_

#include "ORlib/misc/math/math_utils.hpp"
#include "ORlib/misc/PID/pid_controller.hpp"
#include "ORlib/misc/Odometry/odom.hpp"
#include "pros/imu.hpp"

using ORlib::misc::PID::PIDController;

namespace ORlib::base::subsystems::chassis
{

    /**
     * @brief Abstract base class for robot drive systems.
     */
    class Drivetrain
    {
    protected:
        // Drive constants
        float wheel_diameter_;
        float wheel_ratio_;
        float drive_in_to_deg_ratio_;

    public:
        Drivetrain() = default;
        Drivetrain(const float wheel_diameter, const float wheel_ratio)
            : wheel_diameter_(wheel_diameter), wheel_ratio_(wheel_ratio) {}
        virtual ~Drivetrain() = default;

        // Pure virtual methods to be implemented by derived classes
        virtual void drive_with_voltage(const float left_voltage, const float right_voltage) = 0;
        virtual void drive_distance(const float distance) = 0;
    };

    /**
     * @brief Abstract base class for robot drive systems with PID.
     */
    class DrivetrainWithPID
    {
    protected:
        // Inertial sensor and gyro scale
        pros::v5::Imu *imu_;
        float imu_scale_;

        // Turn PID constants
        float turn_max_voltage_, turn_kp_, turn_ki_, turn_kd_;
        float turn_starti_, turn_settle_error_, turn_settle_time_, turn_timeout_;

        // Drive PID constants
        float drive_max_voltage_, drive_kp_, drive_ki_, drive_kd_;
        float drive_starti_, drive_settle_error_, drive_settle_time_, drive_timeout_;

        // Heading PID constants
        float heading_max_voltage_, heading_kp_, heading_ki_, heading_kd_;
        float heading_starti_, heading_settle_error_, heading_settle_time_, heading_timeout_;

        // Swing PID constants
        float swing_max_voltage_, swing_kp_, swing_ki_, swing_kd_;
        float swing_starti_, swing_settle_error_, swing_settle_time_, swing_timeout_;

        // Boomerang constants
        float boomerang_lead_, boomerang_setback_;

        // Drive constants
        float wheel_diameter_;
        float wheel_ratio_;
        float drive_in_to_deg_ratio_;

        // PID Controllers
        PIDController turn_controller_;
        PIDController drive_controller_;
        PIDController heading_controller_;
        PIDController swing_controller_;

    public:
        DrivetrainWithPID(pros::v5::Imu &imu)
            : imu_(&imu),
              turn_controller_(turn_kp_, turn_ki_, turn_kd_, true),
              drive_controller_(drive_kp_, drive_ki_, drive_kd_),
              heading_controller_(heading_kp_, heading_ki_, heading_kd_),
              swing_controller_(swing_kp_, swing_ki_, swing_kd_) {}

        DrivetrainWithPID(pros::v5::Imu &imu, const float imu_scale,
                          const float wheel_diameter, const float wheel_ratio)
            : imu_(&imu), imu_scale_(imu_scale),
              wheel_diameter_(wheel_diameter), wheel_ratio_(wheel_ratio),
              turn_controller_(turn_kp_, turn_ki_, turn_kd_, true),
              drive_controller_(drive_kp_, drive_ki_, drive_kd_),
              heading_controller_(heading_kp_, heading_ki_, heading_kd_),
              swing_controller_(swing_kp_, swing_ki_, swing_kd_) {}

        virtual ~DrivetrainWithPID() = default;

        // Pure virtual methods to be implemented by derived classes
        virtual void drive_with_voltage(const float left_voltage, const float right_voltage) = 0;
        virtual void drive_distance(const float distance) = 0;
        virtual void turn_to_angle(const float angle) = 0;

        // Common methods shared across all drive systems
        float get_absolute_heading() const;
        void set_turn_constants(const float kp, const float ki, const float kd);
        void set_drive_constants(const float kp, const float ki, const float kd);
        void set_heading_constants(const float kp, const float ki, const float kd);
        void set_swing_constants(const float kp, const float ki, const float kd);
        void set_boomerang_constants(const float lead, const float setback);
    };

    /**
     * @brief Abstract base class for robot drive systems with PID.
     */
    class DrivetrainWithOdometry
    {
    protected:
        // Inertial sensor and gyro scale
        pros::v5::Imu *imu_;
        float imu_scale_;

        // Turn PID constants
        float turn_max_voltage_, turn_kp_, turn_ki_, turn_kd_;
        float turn_starti_, turn_settle_error_, turn_settle_time_, turn_timeout_;

        // Drive PID constants
        float drive_max_voltage_, drive_kp_, drive_ki_, drive_kd_;
        float drive_starti_, drive_settle_error_, drive_settle_time_, drive_timeout_;

        // Heading PID constants
        float heading_max_voltage_, heading_kp_, heading_ki_, heading_kd_;
        float heading_starti_, heading_settle_error_, heading_settle_time_, heading_timeout_;

        // Swing PID constants
        float swing_max_voltage_, swing_kp_, swing_ki_, swing_kd_;
        float swing_starti_, swing_settle_error_, swing_settle_time_, swing_timeout_;

        // Boomerang constants
        float boomerang_lead_, boomerang_setback_;

        // Drive constants
        float wheel_diameter_;
        float wheel_ratio_;
        float drive_in_to_deg_ratio_;

        // PID Controllers
        PIDController turn_controller_;
        PIDController drive_controller_;
        PIDController heading_controller_;
        PIDController swing_controller_;

        // Odometry
        OdometryController odometry_controller_;

    public:
        DrivetrainWithOdometry(pros::v5::Imu &imu)
            : imu_(&imu),
              turn_controller_(turn_kp_, turn_ki_, turn_kd_, true),
              drive_controller_(drive_kp_, drive_ki_, drive_kd_),
              heading_controller_(heading_kp_, heading_ki_, heading_kd_),
              swing_controller_(swing_kp_, swing_ki_, swing_kd_) {}

        DrivetrainWithOdometry(pros::v5::Imu &imu, const float imu_scale,
                               const float wheel_diameter, const float wheel_ratio)
            : imu_(&imu), imu_scale_(imu_scale),
              wheel_diameter_(wheel_diameter), wheel_ratio_(wheel_ratio),
              turn_controller_(turn_kp_, turn_ki_, turn_kd_, true),
              drive_controller_(drive_kp_, drive_ki_, drive_kd_),
              heading_controller_(heading_kp_, heading_ki_, heading_kd_),
              swing_controller_(swing_kp_, swing_ki_, swing_kd_) {}

        virtual ~DrivetrainWithOdometry() = default;

        // Pure virtual methods to be implemented by derived classes
        virtual void drive_with_voltage(const float left_voltage, const float right_voltage) = 0;
        virtual void drive_distance(const float distance) = 0;
        virtual void turn_to_angle(const float angle) = 0;

        // Common methods shared across all drive systems
        float get_absolute_heading() const;
        void set_turn_constants(const float kp, const float ki, const float kd);
        void set_drive_constants(const float kp, const float ki, const float kd);
        void set_heading_constants(const float kp, const float ki, const float kd);
        void set_swing_constants(const float kp, const float ki, const float kd);
        void set_boomerang_constants(const float lead, const float setback);
    };

} // namespace ORlib::base::subsystems::chassis

#endif // _DRIVETRAIN_HPP_
