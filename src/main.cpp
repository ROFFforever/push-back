#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pushback/api.hpp"

#include <cstdio>
#include <unistd.h>   
#include "pros/apix.h"



// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain motors
pros::MotorGroup leftMotors({-13, -14, -15},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({16, 17, 18},
                             pros::MotorGearset::blue); // right motor group - ports 6 (reversed), 7, 9

// These are all the intake motors
pros::Motor intake_1(19, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees); // first stage
pros::Motor intake_2(20, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees); // mid stage
pros::Motor intake_3(11, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees); // high stage
pros::Optical optical(3);
pros::Imu imu(12);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(10);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(9);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.0, 0);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2.0, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.8, // 10 inch track width
                              3.25, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(8.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                            9,// derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angularController(3.95, // proportional gain (kP)
                                               0, // integral gain (kI)
                                               29, // derivative gain (kD)
                                               0, // anti windup
                                               0, // small error range, in degrees
                                               0, // small error range timeout, in milliseconds
                                               0, // large error range, in degrees
                                               0, // large error range timeout, in milliseconds
                                               0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
// Wall Distance Sensors
pros::Distance distance_sensor_left_side(2);
pros::Distance distance_sensor_right_side(10);
// Pistons
pushback::Piston unloader(2, pros::E_CONTROLLER_DIGITAL_DOWN);
pushback::Piston score_toggle(3, pros::E_CONTROLLER_DIGITAL_B);
pushback::Piston descore(1, pros::E_CONTROLLER_DIGITAL_Y);
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
pushback::Robot robot(chassis, &intake_1, &intake_2, &intake_3, &descore, &unloader, &descore, nullptr,
                      &distance_sensor_left_side, &distance_sensor_right_side, &imu, controller, &optical, 0);

// Intake mechanism
pushback::Intake intake(robot);

// TODO after comp move all these auton and helper functions into seperate files, no time now
void moveStraight(float length, int timeout, lemlib::MoveToPointParams params, bool async = false) {
    if (chassis.isInMotion()) chassis.waitUntilDone();
    params.forwards = length > 0;
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPoint(pose.x + length * sin(lemlib::degToRad(pose.theta)),
                        pose.y + length * cos(lemlib::degToRad(pose.theta)), timeout, params, async);
}

void wait(int millis) { pros::delay(millis); }

void real_brake() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    wait(300);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

// Use pose opposed to exact coordinates for better readability
void mP(lemlib::Pose target, int timeout, lemlib::MoveToPoseParams params, bool async = true) {
    chassis.moveToPose(target.x, target.y, target.theta, timeout, params, async);
}

void brake() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    wait(300);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

float reset_x_debug = 0;
float reset_y_debug = 0;
bool wall_reset_enabled = true; // toggle for wall resets
bool color_sort_auton = true;

void reset_robot() {
    while (true) {
        if (wall_reset_enabled) {
            if (robot.safe_to_reset_x()) {
                robot.reset_x();
                reset_x_debug++;
            };
            if (robot.safe_to_reset_y()) {
                robot.reset_y();
                reset_y_debug++;
            }
        }
        if (color_sort_auton) { intake.color_sort(); }
        wait(20); // wait as to not hog CPU resources
    }
}
void recieve_data(){
      std::string input_data;
    // Optional: Clear the screen once at the start
    pros::screen::erase();

    while (true) {
        // Listen for data from the PC
        if (std::cin >> input_data) {
          std::cout << "Brain received: " << input_data << std::endl;
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Data: %s", input_data.c_str());
        }
        wait(20);
    }

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// Pure pursuit paths
ASSET(top_left_alligner_to_quad_txt);
ASSET(top_left_central_goal_to_top_right_quad_txt);
ASSET(get_to_bottom_right_goal_txt);
ASSET(go_to_blue_parking_txt);
ASSET(last_goal_skills_txt)
ASSET(elimsLeft_txt);
ASSET(elimsRight_txt);
ASSET(elimLeftButOnRight_txt);
ASSET(elimRightButLeft_txt);
ASSET(sevenBallRight_txt);
ASSET(elimRightTrio_txt);

// debug task to print position to screen
void print_pos() {
    float wall_distance = 0;
    float torque = intake_2.get_torque();
    while (true) {
        torque = intake_2.get_torque();
        wall_distance = robot.get_distance_front();
        const auto pos = chassis.getPose();
        intake.color_sort();

        char buf1[48], buf2[48], buf3[48], buf4[48], buf5[48], buf6[48], buf7[48];
        std::snprintf(buf1, sizeof(buf1), "X: %.3f", pos.x);
        std::snprintf(buf2, sizeof(buf2), "Y: %.3f", pos.y);
        std::snprintf(buf3, sizeof(buf3), "Theta: %.3f", pos.theta);
        std::snprintf(buf4, sizeof(buf4), "Wall Distance: %.3f", wall_distance);
        std::snprintf(buf5, sizeof(buf5), "Reset times X: %.3f", reset_x_debug);
        std::snprintf(buf6, sizeof(buf6), "Reset times Y: %.3f", reset_y_debug);
        std::snprintf(buf7, sizeof(buf7), "Intake(1) torque: %.3f", torque);

        pros::screen::print(pros::E_TEXT_MEDIUM, 1, buf1);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, buf2);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, buf3);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, buf4);
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, buf5);
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, buf6);
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, buf7);

        pros::delay(50);
    }
}

void logData(){
    while(true){
        lemlib::Pose pos = chassis.getPose();
        printf("%.2f,%.2f,%.2f\n", pos.x, pos.y, pos.theta);
        pros::delay(20); //20 ms delay
    }
}






/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() { 
    chassis.setPose(-48,0,90);
    pros::Task debug(logData);
    chassis.moveToPose(-22.56, 25.711, 13.75, 1400);
    chassis.waitUntilDone();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    pros::Task t(recieve_data);
    
    wait(1000000); //can never stop
}
