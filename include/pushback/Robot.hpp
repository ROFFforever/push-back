#pragma once
#include "api.h"
// Add lemlib headers
#include "Wall_Sen.hpp"
#include "lemlib/chassis/chassis.hpp"


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
              pushback::Piston* piston3, pushback::Piston* piston4,std::vector<pushback::Wall_Sen> distance_sensors, pros::Imu* imu,
              pros::Controller& controller, pros::Optical* optical, int color);

        /**
         * @brief Given sensor direction, return the corresponding sensor
         * 
         * @param TYPE 
         * @return Wall_Sen* 
         */
        Wall_Sen* find_sensor(int const TYPE);
        /**
         * @brief Get the side object
         * 
         * @param angle 
         * @return a constant from Wall_Sen, either front,back,left,right. front is when theta is 315-45
         */
        int get_side(float angle);
        /**
         * @brief Get the dist from center of bot to wall using a wall sensor
         * 
         * @param sen 
         */
        float get_dist_from_wall(pushback::Wall_Sen* sen);
        /**
         * @brief Reset robot X position using wall sensors
         * @return true if reset was successful, false if not (if no appropriate sensor was found)
         * 
         */
        bool reset_x();
           /**
         * @brief Reset robot Y position using wall sensors
         * @return true if reset was successful, false if not (if no appropriate sensor was found)
         */
        bool reset_y();
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
        std::vector<pushback::Wall_Sen> distance_sensors; // vector of distance sensors for easy access, can be empty if no distance sensors
        pros::Imu* inertial;
        pros::Controller& controller;
        pros::Optical* optical;
        int color; //0 = blue, 1 = red
        bool color_sort; //whether to color sort or not
};
} // namespace pushback