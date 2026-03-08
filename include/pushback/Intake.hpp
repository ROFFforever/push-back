#pragma once
#include "pros/motors.hpp"
#include "pros/misc.hpp"

namespace pushback {
    class Robot;
/**
 * @class Intake
 * @brief Manages the intake mechanism of the robot.
 *
 * This class encapsulates the motors and controller inputs for the
 * intake system. It handles mapping controller inputs to motor actions
 * and ensures all intake motors work in sync.
 *
 * @note Must call `runIntake()` every loop during the main opcontrol loop
 *       to ensure the intake responds to controller input.
 */

class Intake {
    public:
        /**
         * @brief Function responsible for mapping controller inputs to intake motions
         * @attention Must run every loop of main opcontrol loop
         */
        void runIntake();
        /**
         * @brief Constructor for Intake
         * @param robot contains all hardware
         */
        Intake(Robot& robot); 
        /**
         * @brief Intake blocks
         */
        void intake();
        /**
         * @brief Stops intake
         */
        void stop();
        /**
         * @brief Starts intake for mid goal
         */
        void mid_goal();
         /**
         * @brief Starts intake for mid goal but very fast
         */
        void mid_goal_strong();
         /**
         * @brief Starts intake for mid goal, however does so with less voltage as balls may have lots of force
         */
        void mid_goal_weak();
        /**
         * @brief Scores tall goals
         */
        void tall_goal();
        /**
         * @brief Color sorts intake, uses middle intake to outtake ball
         * @attention meant to be run every cycle in a loop
         */
        void color_sort();
        /**
         * @brief color sorting for skills 
         * 
         */
        void color_sort_skills();
        /**
         * @brief Checks whether the optical sensor is detecting blocks(either color hence should only be in skills)
         * @attention Meant to be run in a cycle
         */
        void color_check_skills();
        /**
         * @brief outakes the intake
         */
        void outake();
        /**
         * @brief Checks if intake is jammed, if so initiates outaking
         * @attention Meant to be run in a loop
         */
        void anti_jam();
        /**
         * @brief checks whether block is wrong color
         * 
         */
        void color_check();
        bool detected; //when can still see opposite color block
                bool opcontrol_intake = false; //intake opcontrol spinning right
    private:
        pushback::Robot& robot;
        bool inMotion; //set to true for actions that use intake that are not opcontrol
        int COLOR1;
        int COLOR2;
};
} 