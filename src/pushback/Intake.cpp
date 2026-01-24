#include "intake.hpp"
#include "Robot.hpp"
#include "Piston.hpp"

namespace pushback {
Intake::Intake(Robot& robot)
    : robot(robot) {
    if (robot.color == 0) { // if blue
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
            robot.intake_1->move_voltage(9000);
            robot.intake_2->move_voltage(-9000);
            robot.intake_3->move_voltage(-9000);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // outake
            robot.intake_1->move_voltage(-12000);
            robot.intake_3->move_voltage(-12000);
            robot.intake_2->move_voltage(12000);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // intake
            robot.intake_1->move_voltage(12000);
            robot.intake_2->move_voltage(-12000);
            robot.intake_3->move_voltage(12000);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else if (robot.controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // weaker outake
            robot.intake_1->move_voltage(-6000);
            robot.intake_3->move_voltage(-12000);
            robot.intake_2->move_voltage(12000);
            opcontrol_intake = true; // opcontrol intake is currently running
        } else {
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
        robot.optical->set_led_pwm(0);
    }
}

void Intake::intake() {
    robot.intake_1->move_voltage(12000);
    robot.intake_2->move_voltage(-12000);
    robot.intake_3->move_voltage(12000);
}

void Intake::stop() {
    robot.intake_2->move_voltage(0);
    robot.intake_1->move_voltage(0);
    robot.intake_3->move_voltage(0);
}

void Intake::mid_goal() {
    robot.intake_1->move_voltage(5000);
    robot.intake_2->move_voltage(-9000);
    robot.intake_3->move_voltage(-3500);
}
void Intake::mid_goal_strong() {
    robot.intake_1->move_voltage(12000);
    robot.intake_2->move_voltage(-9000);
    robot.intake_3->move_voltage(-4000);
}

void Intake::mid_goal_weak() {
    robot.intake_1->move_voltage(5000);
    robot.intake_2->move_voltage(-4500);
    robot.intake_3->move_voltage(-2500);
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

void Intake::color_sort() {
    if (robot.color_sort) {
        int hue = robot.optical->get_hue(); // get color reading
        if (hue > COLOR1 && hue < COLOR2) { // if hue is within a certain range(either blue or red)
            detected = true; // detected can be used for color sorting
        } else {
            detected = false;
        }
        if (detected) { // if detected set inMotion to true to cancel opcontrol intake and start outtaking
            int volt_1 = robot.intake_1->get_voltage(); // store current voltage to restore later
            int volt_2 = robot.intake_2->get_voltage(); // store current voltage to restore later
            int volt_3 = robot.intake_3->get_voltage(); // store current voltage to restore later
            inMotion = true;
            detected = true;
            pros::delay(0); // do whatever was before and also push block a little more forward
            mid_goal_strong(); // outtake blue block
            pros::delay(200);
            robot.intake_1->move_voltage(volt_1); // restore past voltage
            robot.intake_2->move_voltage(volt_2);
            robot.intake_3->move_voltage(volt_3);
        } else if (!detected && !opcontrol_intake) { // if not detected stop intake and motion is now done
            inMotion = false;
        }
    }
}

} // namespace pushback
