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
   void intake_weak_super();
        void intake_weak();
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
         * @brief sorta weak, weaker than max but stronger than weak setting
         * 
         */
        void weak_kinda();
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
         * @brief color sorting for skills. checks color in input(true for checking if there's red and vice versa for blue)
         * @param true for blue
         * @param false for red
         * 
         */
        void color_check(bool color);
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
         * @brief outake weak for low goal
         * 
         */
        void outake_weak();
    /**
     * @brief outake for last balls in low goal for skills
     * 
     */
        void outake_super_weak();
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