#pragma once
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

namespace pushback {
class Robot;

/**
 * @class Piston
 * @brief This class encapsulates the vex pneumatic piston and provides methods for firing the piston.
 */
class Piston {
    public:
        /**
         * @brief fires piston
         * @param activate(fire state)
         */
        void firePiston(bool activate);
        /**
         * @brief Piston constructor
         * @param port takes port of piston (A --> 1, B --> 2)
         */
        Piston(int port);
        /**
         * @brief Piston constructor
         * @param port takes port of piston (A --> 1, B --> 2)
         * @param button takes controller button which you want to toggle piston in opcontrol
         * @param controller takes controller which you are using
         *
         * Example usage:
         * @code
          pushback::Piston(2, pros::E_CONTROLLER_DIGITAL_A) will init piston with port 2 and track button A.
          You can call toggleCheck() to toggle piston based off button.
          @endcode
         */
        Piston(int port, pros::controller_digital_e_t button);

        /**
         * @brief add a controller for input tracking for this piston
         * @details This function is needed as the goal is to centralize hardware in Robot class. Including a param for
         * Robot in constructor wouldn't work as to make a Robot object you need Piston objects. But when creating
         * Piston objects you need Robot objects... This is why register_controller exists.
         * @param robot is the pointer to the robot object
         */
        void register_controller(Robot* robot);
        /**
         * @brief Gets state of button registered(or not) with piston
         * @return true if registered button is being pressed, false elsewise
         */
        bool getButton();
        /**
         * @brief checks current controller inputs and if registered button is pressed, will toggle piston
         * @attention must run every cycle of opcontrol loop
         */
        void toggleFire();
        bool buttonState = false; // Initialize the toggle state
    private:
        pros::ADIDigitalOut* pneumatic; // piston
        bool lastButtonState = false; // Track the previous button state
        pros::controller_digital_e_t button;
        pushback::Robot* robot; // can be null because user may not be interesed in registering this for opcontrol
};
} // namespace pushback