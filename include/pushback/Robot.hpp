#pragma once
#include "api.h"
// Add lemlib headers
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"


namespace pushback {
    class Piston;
    class Intake;
/**
 * @brief Holds all sensors and motors on robot for easy access
 * @param color 0 is blue
 */
class Robot {
    public:
        Robot(lemlib::Chassis& chassis, pros::Motor* intake1, pros::Motor* intake2,
              pros::Motor* intake3, pushback::Piston* piston1, pushback::Piston* piston2, 
              pushback::Piston* piston3, pushback::Piston* piston4, pros::Distance* distance_sensor_left_side, pros::Distance* distance_sensor_right_side, pros::Imu* imu,
              pros::Controller& controller, pros::Optical* optical, int color);

        /**
         * @brief Get distance from wall, accounts for non perfect orientations(not perfect alligned with wall) with side sensor
         * 
         * @return float Distance in inches from wall
         */
        float get_distance_left_side();

                /**
         * @brief Get distance from wall, accounts for non perfect orientations(not perfect alligned with wall) with side sensor
         * 
         * @return float Distance in inches from wall
         */
        float get_distance_right_side();

        /**
         * @brief gets distance of front sensor, this is just a temporary function until after UC Berkly when I will integrate both wall distance to reset
         * @return float Distance in inches from wall
         */
        float get_distance_front();
        /**
         * @brief goes until that desired wall sensor distance is met
         * @param distance from the wall to the robot
         * @param speed how fast you want to go
         * @param timeout maximum amount of time allocated to this movement
         * @param turn how much to turn the robot
         * @attention assumes you are lined up with wall
         */
        void go_until_front(float distance, int speed, int timeout, int resistance, float desired_theta, bool initBurst, int burst_time);

        /**
         * @brief turn to a point using wall sensor for accuracy
         * @param target is pose that it will turn towards
         * @param timeout max time allowed for this movement
         */
        void turnToPointSide(lemlib::Pose target, int timeout);
        /**
         * @brief Set distance offset for distance from wall
         * 
         * @param offset Offset in inches
         */
        void set_distance_offset(float offset);
    
        /**
         * @brief Reset/correct the x using the wall distance sensor
         */
        void reset_x();

        /**
         * @brief Reset/correct the y using the wall distance sensor
         */
        void reset_y();
        
        /**
         * @brief Make sure robot and wall distance coords are relatively close(8 inches) for x
         * @return if distance sensor is reporting way off, return false, if good, return true
         */
        bool filter_x();

        /**
         * @brief Make sure robot and wall distance coords are relatively close(8 inches) for y
         * @details First checks if robot is facing the right wall to reset y by finding closest cardinal angle
         * @details Second checks if robot is close enough to the wall without losing accuracy
         * @details Third checks if robot is relatively parallel to the wall(30 degrees)
         * @details Last checks if any field obstacles block distance sensors beam(match loading section)
         * @return if distance sensor is reporting way off, return false, if good, return true
         */
        bool filter_y();

        /**
         * @brief Assesses whether the robot is within range and orientation to utilize wall sensors for resetting x position
         * @details First checks if robot is facing the right wall to reset x by finding closest cardinal angle
         * @details Second checks if robot is close enough to the wall without losing accuracy
         * @details Third checks if robot is relatively parallel to the wall(30 degrees)
         * @details Last checks if any field obstacles block distance sensors beam(match loading section)
         * @return If robot is adequately close enough to the wall and has a relatively straight orientation relative to the wall, return true
         */
        bool safe_to_reset_x();

        /**
         * @brief Assesses whether the robot is within range and orientation to utilize wall sensors for resetting y position
         * @return If robot is adequately close enough to the wall and has a relatively straight orientation relative to the wall, return true
         */
        bool safe_to_reset_y();
        /**
         * @brief move the robot forward and backward
         *  @param magnitude speed from 0 to 127
         *  @param time time to move in milliseconds
         *  @param cycle_time time per movement cycle(back and forth) in milliseconds
         *  @details usually used to dislodge blocks from loader
         */
        void jiggle(float magnitude, int cycle_time, int time);
        /**
         * @brief Drive forward with a set ramming speed for a set time
         * @param magnitude speed from -127 to 127
         */
        void ram(int magnitude, int time);

        lemlib::Chassis& chassis; // Public chassis field
        pros::Motor* intake_1;    // intake motors, pointers as they can be nullptr
        pros::Motor* intake_2;
        pros::Motor* intake_3;
        pushback::Piston* piston_1; // pistons
        pushback::Piston* piston_2;
        pushback::Piston* piston_3;
        pushback::Piston* piston_4;
        pros::Distance* distance_sensor_left_side;
        pros::Distance* distance_sensor_right_side;
        pros::Imu* inertial;
        pros::Controller& controller;
        pros::Optical* optical;
        int color; //0 = blue, 1 = red
        float wall_distance_offset;
        bool color_sort; //whether to color sort or not
};
} // namespace pushback