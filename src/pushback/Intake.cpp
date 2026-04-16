#include "pushback/intake.hpp"
#include "pushback/Robot.hpp"
#include "Piston.hpp"

namespace pushback {
Intake::Intake(Robot& robot)
    : robot(robot) {
    if (robot.color == 0) { 
        COLOR1 = 0;
        COLOR2 = 17;
    } else {
        COLOR1 = 200;
        COLOR2 = 230;
    }
}

void Intake::runIntake() {
    if (!inMotion) {
        if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // mid goal
            robot.intake_1->move_voltage(12000);
            robot.intake_2->move_voltage(-12000);
            robot.intake_3->move_voltage(-12000);
            robot.piston_4->firePiston(true);
            //4000 for midle goal in skills
            //9000 for regular matches
            opcontrol_intake = true; // opcontrol intake is currently running
        } else if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // outake
            robot.intake_1->move_voltage(-12000);
            robot.intake_3->move_voltage(-12000);
            robot.intake_2->move_voltage(12000);
            robot.piston_4->firePiston(false);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // intake
            robot.intake_1->move_voltage(12000);
            robot.intake_2->move_voltage(-12000);
            robot.intake_3->move_voltage(12000);
            robot.piston_4->firePiston(false);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // weaker outake
                robot.intake_1->move_voltage(-8000);
            robot.intake_3->move_voltage(-8000);
            robot.intake_2->move_voltage(8000);
            robot.piston_4->firePiston(false);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else {
            robot.piston_4->firePiston(false);
            robot.intake_2->move_voltage(0);
            robot.intake_1->move_voltage(0);
            robot.intake_3->move_voltage(0);
            opcontrol_intake = false; // intake not running
        }
    } else {
        opcontrol_intake = false; // since another motion requires the intake the intake can't run
    }
    if (inMotion || opcontrol_intake) {
        robot.optical->set_led_pwm(100);
    } else {
        robot.optical->set_led_pwm(100);
    }
}

void Intake::intake() {
    robot.intake_1->move_voltage(12000);
    robot.intake_2->move_voltage(-12000);
    robot.intake_3->move_voltage(12000);
}
void Intake::intake_weak() {
    robot.intake_1->move_voltage(7000);
    robot.intake_2->move_voltage(-7000);
    robot.intake_3->move_voltage(8000);
}
void Intake::intake_weak_super() {
    robot.intake_1->move_voltage(5000);
    robot.intake_2->move_voltage(-5000);
    robot.intake_3->move_voltage(5000);
}
void Intake::outake_weak() {
    robot.intake_1->move_voltage(-6000);
    robot.intake_3->move_voltage(-6000);
    robot.intake_2->move_voltage(5500);
}

void Intake::outake_super_weak() {
    robot.intake_1->move_voltage(-4000);
    robot.intake_3->move_voltage(-4000);
    robot.intake_2->move_voltage(4000);
}

void Intake::stop() {
    robot.intake_2->move_voltage(0);
    robot.intake_1->move_voltage(0);
    robot.intake_3->move_voltage(0);
}

void Intake::mid_goal() {
    robot.intake_1->move_voltage(12000);
    robot.intake_2->move_voltage(-5000);
    robot.intake_3->move_voltage(-3500);
}
void Intake::mid_goal_strong() {
    robot.intake_1->move_voltage(12000);
    robot.intake_2->move_voltage(-9000);
    robot.intake_3->move_voltage(-4000);
}

void Intake::mid_goal_weak() {
    robot.intake_1->move_voltage(5000); //old vlaue 5000
    robot.intake_2->move_voltage(-3800); //old value -4500
    robot.intake_3->move_voltage(-2500); //old value -2500
}

void Intake::tall_goal() {
    robot.piston_1->firePiston(true);
    robot.intake_1->move_voltage(12000);
    robot.intake_2->move_voltage(12000);
    robot.intake_3->move_voltage(-12000);
}

void Intake::outake() {
    robot.intake_1->move_voltage(-12000);
    robot.intake_3->move_voltage(-12000);
    robot.intake_2->move_voltage(12000);
}

void Intake::anti_jam() {
    float torque = robot.intake_2->get_torque();
    float past_volt = robot.intake_2->get_voltage();
    float torque_2 = robot.intake_3->get_torque();
    float past_volt_2 = robot.intake_3->get_voltage();
    if (torque >= 1.01 || torque_2 >= 1.01) { // if torque is above threshold, intake is likely jammed
        inMotion = true; // set inMotion to true to prevent opcontrol intake from interfering
        robot.intake_1->move_voltage(-9000); // reverse intake to unjam
        pros::delay(250); // give intake time to unjam
        robot.intake_1->move_voltage(past_volt); // restore past direction and power of intake
        inMotion = false; // set inMotion to false as unjamming is complete
    }
}
void Intake::color_check_skills(){
     float hue = robot.optical->get_hue(); // get color reading
        if ((hue > 0 && hue < 8) || (hue > 218 && hue < 243)) { // if hue is within a certain range(blue and red)
            this->detected = true; // detected can be used for color sorting
        } else {
            this->detected = false;
        }
}
void Intake::color_check(){
     int hue = robot.optical->get_hue(); // get color reading
        if ((hue > COLOR1 && hue < COLOR2)) { // if hue is within a certain range(blue and red)
            this->detected = true; // detected can be used for color sorting
        } else {
            this->detected = false;
        }
}
void Intake::color_check(bool color){
     int hue = robot.optical->get_hue(); // get color reading
    int color11;
    int color22;
     if(color){
        color11 = 200;
        color22 = 230;
     }else{
        color11 = 0;
        color22 = 6;
     }
        if ((hue > color11 && hue < color22)) { // if hue is within a certain range(blue and red)
            this->detected = true; // detected can be used for color sorting
        } else {
            this->detected = false;
        }
}
void Intake::color_sort() {
    if (robot.color_sort) {
        color_check(); // update whether a block is detected
        if (detected) { // if detected set inMotion to true to cancel opcontrol intake and start outtaking
            int volt_1 = robot.intake_1->get_voltage(); // store current voltage to restore later
            int volt_2 = robot.intake_2->get_voltage(); // store current voltage to restore later
            int volt_3 = robot.intake_3->get_voltage(); // store current voltage to restore later
            inMotion = true;
            pros::delay(60); // do whatever was before and also push block a little more forward
            mid_goal_strong(); // outtake blue block
            pros::delay(400); //wait \to push out ball
            robot.intake_1->move_voltage(volt_1); // restore past voltage
            robot.intake_2->move_voltage(volt_2);
            robot.intake_3->move_voltage(volt_3);
        } else if (!detected && !opcontrol_intake) { // if not detected stop intake and motion is now done
            inMotion = false;
        }
    }
}
void Intake::color_sort_skills() {
        color_check(); // update whether a block is detected
        if (detected) { // if detected set inMotion to true to cancel opcontrol intake and start outtaking
            int volt_1 = robot.intake_1->get_voltage(); // store current voltage to restore later
            int volt_2 = robot.intake_2->get_voltage(); // store current voltage to restore later
            int volt_3 = robot.intake_3->get_voltage(); // store current voltage to restore later
            inMotion = true;
            pros::delay(60); // do whatever was before and also push block a little more forward
            mid_goal_strong(); // outtake blue block
            pros::delay(400); //wait \to push out ball
            robot.intake_1->move_voltage(volt_1); // restore past voltage
            robot.intake_2->move_voltage(volt_2);
            robot.intake_3->move_voltage(volt_3);
        } else if (!detected && !opcontrol_intake) { // if not detected stop intake and motion is now done
            inMotion = false;
        }
}
} // namespace pushback
