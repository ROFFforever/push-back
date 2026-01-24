#include "pushback/Piston.hpp"
#include "Robot.hpp"

    void pushback::Piston::firePiston(bool activate){
        pneumatic->set_value(activate);
    }
    pushback::Piston::Piston(int port){
        this->pneumatic = new pros::ADIDigitalOut(port);
        this->robot = nullptr;
    }
    pushback::Piston::Piston(int port, pros::controller_digital_e_t button){
        this->pneumatic = new pros::ADIDigitalOut(port);
        this->button = button;
    }
    bool pushback::Piston::getButton(){
        if(robot != nullptr){ //make sure controller exists
        if(robot->controller.get_digital(button) == 1){ //check if clicking
            return true;
        }else{
            return false;
        }
        }
        return false;
    }
    void pushback::Piston::register_controller(Robot* robot){
        this->robot = robot;
    }
    void pushback::Piston::toggleFire(){
        if(robot != nullptr){
        bool currentButtonState = getButton();
        if(currentButtonState && !this->lastButtonState){
            buttonState = !buttonState;
            if(buttonState){
                Piston::pneumatic->set_value(true);
                pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
            }else if(!buttonState){
                Piston::pneumatic->set_value(false);
            }
        }
        this->lastButtonState = currentButtonState;
    }
    }