/**
 * @file tank_drive.hpp
 * @author Adin De'Rosier (adinderosier@live.com)
 * @brief Header file for the class that defines a tank drive style 
 *        drivetrain.
 * @version 1.0
 * @date 2025-1-27
 */

#ifndef _TANK_DRIVE_HPP_
#define _TANK_DRIVE_HPP_

#include "ORlib/base/subsystems/chassis/drivetrain.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"

namespace ORlib::base::subsystems::chassis::tank
{
    /**
     * @brief TankDrive class supporting tank drive without odometry or 
     *        PID.
     */
    class TankDrive : public Drivetrain
    {
    private:
        pros::v5::MotorGroup *left_motors_;
        pros::v5::MotorGroup *right_motors_;

    public:
        TankDrive(pros::v5::MotorGroup &left_motors, 
                  pros::v5::MotorGroup &right_motors,
                  float wheel_diameter, float wheel_ratio) : Drivetrain(wheel_diameter, wheel_ratio), 
                                                             left_motors_(&left_motors), 
                                                             right_motors_(&right_motors) {}

        void drive_with_voltage(float left_voltage, float right_voltage);
        void drive_distance(float distance, float velocity);

        // Driver Control methods
        void tank_drive(float left_voltage, float right_voltage);
        void arcade_drive(float forward_voltage, float turn_voltage);
    };

    class TankDriveWithPID : public Drivetrain
    {
    private:
        pros::v5::MotorGroup *left_motors_;
        pros::v5::MotorGroup *right_motors_;
        pros::Imu *imu_;

    public:
        TankDriveWithPID(pros::v5::MotorGroup &left_motors, 
                         pros::v5::MotorGroup &right_motors,
                         pros::Imu &imu,
                         float wheel_diameter, float wheel_ratio) : Drivetrain(wheel_diameter, wheel_ratio), 
                                                                    left_motors_(&left_motors), 
                                                                    right_motors_(&right_motors),
                                                                    imu_(&imu) {}

        void drive_with_voltage(float left_voltage, float right_voltage);
        void drive_distance(float distance, float velocity);

        // Driver Control methods
        void tank_drive(float left_voltage, float right_voltage);
        void arcade_drive(float forward_voltage, float turn_voltage);
    };

    class TankDriveWithOdometry : public Drivetrain
    {
    private:
        pros::v5::MotorGroup *left_motors_;
        pros::v5::MotorGroup *right_motors_;
        pros::Imu *imu_;

    public:
        TankDriveWithOdometry(pros::v5::MotorGroup &left_motors, 
                              pros::v5::MotorGroup &right_motors,
                              pros::Imu &imu,
                              float wheel_diameter, float wheel_ratio) : Drivetrain(wheel_diameter, wheel_ratio), 
                                                                     left_motors_(&left_motors), 
                                                                     right_motors_(&right_motors),
                                                                     imu_(&imu) {}

        void drive_with_voltage(float left_voltage, float right_voltage);
        void drive_distance(float distance, float velocity);

        // Driver Control methods
        void tank_drive(float left_voltage, float right_voltage);
        void arcade_drive(float forward_voltage, float turn_voltage);
    };
}

#endif // _TANK_DRIVE_HPP_