#include "main.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pushback/api.hpp"

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
                                            9, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angularController(3.95, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             29, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             300, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             900, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
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

void rb(float wait_time) {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    wait(wait_time);
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors
    unloader.register_controller(&robot);
    descore.register_controller(&robot);
    score_toggle.register_controller(&robot);
    robot.set_distance_offset(4); // set distance sensor offset in inches
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
        torque = imu.get_roll();
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
        std::snprintf(buf7, sizeof(buf7), "Pitch: %.3f", torque);

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

void checkColorGaps() {
    while (true) {
        optical.set_led_pwm(100);
        intake.color_check_skills();
        float hue = robot.optical->get_hue();
        if (intake.detected) {
            printf("%.2f, %d", hue, intake.opcontrol_intake);
            printf(" - Detected\n");
        } else {
            printf("%.2f, %d", hue, intake.opcontrol_intake);
            printf(" - Not Detected\n");
        }
        wait(10);
    }
}

void logData() {
    while (true) {
        lemlib::Pose pos = chassis.getPose();
        printf("%.2f,%.2f,%.2f\n", pos.x, pos.y, pos.theta);
        pros::delay(50);
    }
}

// AUTONS
void left_side_red() {
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    color_sort_auton = false;
    chassis.setPose(-51.711, 14.489, 90);
    intake.intake(); // start intake to intake first trio of balls
    chassis.moveToPose(-22.088, 20.234, 55.546, 2400); // get first trio of balls
    chassis.turnToHeading(-43.050, 800); // turn toward central goal
    chassis.moveToPose(-8.90, 5.1, -44.5, 1300, {.forwards = false});
    intake.outake();
    wait(300);
    intake.stop();
    wait(550);
    intake.mid_goal();
    wait(1700); // give time to score
    chassis.moveToPose(-49.5, 45.7, -87.6, 2500, {.lead = 0.32, .earlyExitRange = 4}); // get to matchloaders
    intake.intake(); // change intake modes to intake
    wait(1200);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    robot.ram(85, 1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    moveStraight(-35, 1900, {.maxSpeed = 90}, false);
    descore.firePiston(true);
    robot.ram(-85, 2000); // move back to get into alligner
    descore.firePiston(false);
    moveStraight(9, 360, {}, false);
    robot.ram(-85, 3200); // push balls deeper into goal also alligning robot via alligner
    wait(4000);
}

float brakeSmart() {
    lemlib::Pose speed = lemlib::getLocalSpeed(true);
    float current_speed = sqrt(speed.x * speed.x + speed.y * speed.y);
    return current_speed;
}

void skills() {
    // Starting Position
    lemlib::Pose start_pose(-47.25, 16.3, 90);
    float error = -10000;
    float goal_x;
    float goal_y;
    float angError;

    // #1
    chassis.setPose(-47, 16.3, 90);
    descore.firePiston(true);
    intake.intake(); // Start intake
    moveStraight(10, 500, {}, false);
    chassis.turnToHeading(63, 540, {.maxSpeed = 110}, false);
    moveStraight(16.5, 950, {.maxSpeed = 100}, false); // go get the quad balls for two, one red and one blue
    brake();
    intake.stop(); // stop intake to prep for outtake
    chassis.turnToHeading(318, 790, {.maxSpeed = 65}, false); // turn towards midgoal
    chassis.moveToPoint(-12, 10.8, 800, {.forwards = false, .maxSpeed = 80}, true); // move to midgoal
    intake.outake(); // get them unstuck
    intake_1.move_voltage(0); // dont move the first stage as to not loose balls
    wait(120); // give the balls time to get unstuck
    intake.stop();
    chassis.waitUntilDone();
    chassis.turnToPoint(0, 0, 300, {.forwards = false}, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // hold so robot doesn't drift
    intake.mid_goal(); // score mid goal
        unloader.firePiston(true);
    moveStraight(2, 200, {}, false);
    wait(400); // give time to score
    intake.outake(); // finished
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-46, 45.5, 1600, {.maxSpeed = 100}, true); // go towards matchloaders
    intake.mid_goal();
    wait(150);
    intake.outake(); // outtake to push balls into matchloader
    wait(200);
    intake.mid_goal_strong();
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    rb(100);
    chassis.turnToHeading(-90, 600, {}, false);
    intake.intake();
    robot.ram(98, 390); // get matchloader balls
    robot.ram(50, 1400); // push hard to tilt matchloader
    moveStraight(-8, 300, {.minSpeed=1, .earlyExitRange = 1.5}, true);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 250, {}, false);
    chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_right_side() + 0.1, chassis.getPose().theta); //
    chassis.turnToHeading(0, 600); unloader.firePiston(false); // Matchloader up 
    moveStraight(7, 440, {},
    true); // move so dont hit the goals 
    chassis.swingToHeading(90, DriveSide::RIGHT, 650); // turn so can goforwards
    real_brake(); 
    chassis.turnToHeading(88, 230); // make sure we not off 
    intake.stop();
    chassis.moveToPoint(50, 57, 2200, {}, true); // go to other side of field
    error = 25 - chassis.getPose().x;
    while (error > 0.5) { // once past x=30 slow down basically
        error = 30 - chassis.getPose().x;
        wait(20);
    }
    chassis.cancelMotion();
    chassis.moveToPoint(50, 57, 800, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 1},
                        false); // go to other side of field
    chassis.turnToHeading(90, 250, {}, false);
    chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_left_side() - 0.5, chassis.getPose().theta); //
    rb(50); 
                                                                                                     // goal
    chassis.moveToPose(26, 47.2, 90, 1100, {.forwards = false, .lead = 0.3, .minSpeed = 1, .earlyExitRange = 1},
                       false); // get into the goal
    chassis.turnToHeading(90, 300, {.minSpeed = 50, .earlyExitRange = 1}, false);
    robot.ram(-100, 300); // get into goal alligner
    intake.intake();
    score_toggle.firePiston(true); // open up scoring hood
    unloader.firePiston(true); // prefire this early as to not jostle around when deploying before matchloader
    robot.ram(-90, 2100); // move back to get into alligner/goal and score
    intake.stop();
    rb(120); // brake for a bit
    angError = fabs(chassis.getPose().theta - 90);
    if (angError < 3.5)
        chassis.setPose(28, 46.8, chassis.getPose().theta); // if robot isn't within 3.5 degrees(aka its not straight
        //on) don't reset angle
    wait(50);
    chassis.moveToPose(52, 45.5, 90, 1000, {.lead=0.2}, true); // go to matchloader
    wait(300);
    score_toggle.firePiston(false); // close scoring hood
    intake.intake();
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 150, {.minSpeed = 1, .earlyExitRange = 1}, false); // align with matchloader
    robot.ram(95, 350); // get matchloader balls
    robot.ram(50, 1300);
    chassis.moveToPose(29.5, 47.5, 90, 950, {.forwards = false, .lead = 0.3, .minSpeed = 70, .earlyExitRange = 1.5},
                       true); // go back into goal

    // UNJAM INTAKE INCASE SOMETHING STUCK
    wait(300);
    intake.outake();
    intake_3.move_voltage(0);
    wait(150);
    intake.stop();
    wait(100);
    score_toggle.firePiston(true); // open up scoring hood
    chassis.waitUntilDone();
    unloader.firePiston(false); // Matchloader up
    robot.ram(-105, 200); // get into goal
    intake.intake();
    robot.ram(-80, 1650); // move back to get into alligner/goal and score
    angError = fabs(chassis.getPose().theta - 90);
    if (angError < 3.5) { chassis.setPose(28, 46.8, chassis.getPose().theta); }
    wait(15); // let that register

    // TODO REMOVE THIS
    //chassis.setPose(28, 46.8, 90); // if robot isn't within 3.5 degrees(aka its not straight on) don't reset angle
    // chassis.setPose(70 - robot.get_distance_left_side(), -17, 180); // used to be -17.5
    // descore.firePiston(true);
    //   chassis.setPose(-28, -46.8,
    //                  270); //if robot isn't within 3.5 degrees(aka its not straight on) don't reset angle
    //  unloader.firePiston(true);
    //  score_toggle.firePiston(true);
    // wait(1000);

    // #2
    moveStraight(6, 400, {}, false); // move so dont hit the goals
    goal_x = 59.57;
    goal_y = 21;
    chassis.moveToPoint(goal_x, goal_y, 1300, {.maxSpeed = 80}, true);
    error = sqrt(pow(goal_x - chassis.getPose().x, 2) + pow(goal_y - chassis.getPose().y, 2));
    while (error > 15) {
        error = sqrt(pow(goal_x - chassis.getPose().x, 2) + pow(goal_y - chassis.getPose().y, 2));
        wait(10);
    }
    chassis.cancelMotion();
    chassis.moveToPoint(61, goal_y - 1.5, 800, {.maxSpeed = 35}, false);
    rb(60);
    chassis.swingToHeading(175, DriveSide::RIGHT, 600, {}, false);
    intake.intake();
    score_toggle.firePiston(false);
    float timeCross = 2600;
    chassis.arcade(110, 0); // cross barrier
    wait(250);
    chassis.arcade(87, 0);
    wait((timeCross / 2) - 250);
    intake.outake();
    wait(60);
    intake.intake();
    wait((timeCross / 2) - 250);
    float roll = imu.get_roll();
    float prevRoll = -100;
    float dRoll = 2; // change in roll(up and down motion)
    while (dRoll > 0.25) {
        dRoll = fabs(prevRoll - roll);
        prevRoll = roll;
        roll = imu.get_roll();
        wait(200);
    }
    wait(120); // go a little more to be safe
    chassis.arcade(0, 0); // stop
    robot.ram(-60, 1700); // allign with parking zone
    wait(200); // wait for robot to settle
    chassis.setPose(70 - robot.get_distance_left_side(), -17.0f, chassis.getPose().theta); // used to be -17.5
    wait(20); // let register

    // #3
    chassis.moveToPoint(61, -28, 700, {.maxSpeed = 60, .minSpeed = 1, .earlyExitRange = 1}, false);
    chassis.turnToPoint(22.3, -17.37, 700, {.maxSpeed = 60}, false);
    intake.intake();
    chassis.moveToPoint(22.3, -18.1, 900, {.maxSpeed = 100}, false);
    intake.stop();
    chassis.moveToPoint(8.5, -8.5, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 2},
                        false);
    intake.outake(); // get them unstuck
    intake_1.move_voltage(0); // dont move the first stage as to not loose balls
    wait(320); // give the balls time to get unstuck
    intake.stop();
    chassis.waitUntilDone();
    moveStraight(1.95, 200, {}, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // hold so robot doesn't drift
    intake.mid_goal(); // score mid goal
    wait(1400); // give time to score
    moveStraight(1.4, 200, {}, false); //get ot of goal
    intake.mid_goal_weak();
    robot.optical->set_led_pwm(100); // turn on optical sensor led for auton
    lemlib::Timer scoreTime(1900);
    bool shouldExit = false;
    while (scoreTime.getTimePassed() < 1900 && !shouldExit) {
        if (intake.detected) {
            wait(200);
            shouldExit = true;
            intake.stop(); // moment spotted start outaking
        }
        if (scoreTime.getTimePassed() > 1800) { // once time over stop
            wait(100);
            intake.stop();
        }
        intake.color_check(true); // check for any blue
        wait(20); // give a little delay
    }
    intake.intake();
    score_toggle.firePiston(true); // get balls out so dont accidentally store
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // Let go
    chassis.moveToPoint(51, -48, 1800, {.maxSpeed = 85}, true); // go towards matchloaders
    wait(230);
    intake.mid_goal_strong();
    wait(350);
    unloader.firePiston(true); // Piston down to unload matchloads
    wait(300);
    chassis.waitUntilDone();
    score_toggle.firePiston(false); // close scoring hood
    chassis.turnToHeading(90, 500, {.minSpeed = 1, .earlyExitRange = 1}, false); // align with matchloader
    intake.intake();
    robot.ram(95, 470); // get matchloader balls
    robot.ram(50, 1300); // push hard to tilt matchloader
    moveStraight(-9.5, 300, {}, false); // get out of way
    unloader.firePiston(false); // Matchloader up
    chassis.turnToHeading(90, 300, {}, false);
    chassis.setPose(chassis.getPose().x, robot.get_distance_right_side() + 0.1 - 70, chassis.getPose().theta); // reset
    wait(15); // let reset register
    chassis.turnToHeading(180, 600, {}, false);
    moveStraight(7.5, 450, {},
                 true); // move so dont hit the goals wait(100); unloader.firePiston(false); // Matchloader up
    chassis.waitUntilDone();
    chassis.moveToPoint(-50, -57.6, 2200, {}, true); // go to other side of field
    error = chassis.getPose().x + 30;
    while (error > 0.5) { // once past x=-30 slow down basically
        error = chassis.getPose().x + 30;
        wait(20);
    }
    chassis.cancelMotion();
    chassis.moveToPoint(-50, -57.6, 950, {.maxSpeed = 40},
                        false); // go to other side of field
    rb(40);
    chassis.turnToHeading(270, 300, {}, false);
    chassis.setPose(chassis.getPose().x, robot.get_distance_left_side() + 0.5 - 70,
                    chassis.getPose().theta); //                                                  // goal
    chassis.moveToPose(-24, -46.4, -90, 1200, {.forwards = false, .lead = 0.4, .maxSpeed = 95},
                       false); // get into the goal
    chassis.turnToHeading(-90, 300, {.minSpeed = 50, .earlyExitRange = 1}, false);
    robot.ram(-100, 300); // get into goal alligner
    intake.intake();
    score_toggle.firePiston(true); // open up scoring hood
    unloader.firePiston(true); // prefire this early as to not jostle around when deploying before matchloader
    robot.ram(-90, 2650); // move back to get into alligner/goal and score
    intake.stop();
    rb(70); // brake for a bit
    angError = fabs(chassis.getPose().theta - 270);
    if (angError < 3.5)
        chassis.setPose(-28, -46.8,
                        270); // if robot isn't within 3.5 degrees(aka its not straight on) don't reset

    wait(15); // let that register

    // // //TODO REMOVE THIS
    // // chassis.setPose(
    // //     -28, -46.8,
    // //     270); // if robot isn't within 3.5 degrees(aka its not straight on) don't reset angle
    // // wait(15); // let that register
    // // unloader.firePiston(true);
    // // wait(600);

    // #4
    chassis.moveToPoint(-48, -46, 700, {.minSpeed = 1, .earlyExitRange = 0.5}, true); // go to matchloader
    wait(300);
    score_toggle.firePiston(false); // close scoring hood
    intake.intake();
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 150, {.minSpeed = 1, .earlyExitRange = 1}, false); // align with matchloader
    robot.ram(85, 450); // get matchloader balls
    robot.ram(80, 1700); // push hard to tilt matchloader
    chassis.moveToPose(-29.5, -47, 270, 800, {.forwards = false, .lead = 0.3, .minSpeed = 70, .earlyExitRange = 2},
                       true); // go back into goal
    wait(300);
    intake.outake();
    intake_3.move_voltage(0);
    wait(150);
    intake.stop();
    wait(300);
    score_toggle.firePiston(true); // open up scoring hood
    chassis.waitUntilDone();
    unloader.firePiston(false); // Matchloader up
    robot.ram(-105, 200); // get into goal
    intake.intake();
    robot.ram(-80, 2000); // move back to get into alligner/goal and score
    angError = fabs(chassis.getPose().theta - 270);
    chassis.setPose(
        -28, -46.8,
        angError < 3.5
            ? 270
            : chassis.getPose().theta); // if robot isn't within 3.5 degrees(aka its not straight on) don't reset
    moveStraight(6, 400, {}, false); // move so dont hit the goals
    goal_x = -63;
    goal_y = -18.8;
    chassis.moveToPoint(goal_x, goal_y, 1300, {.maxSpeed = 80}, true);
    error = sqrt(pow(goal_x - chassis.getPose().x, 2) + pow(goal_y - chassis.getPose().y, 2));
    while (error > 15) {
        error = sqrt(pow(goal_x - chassis.getPose().x, 2) + pow(goal_y - chassis.getPose().y, 2));
        wait(10);
    }
    chassis.cancelMotion();
    chassis.moveToPoint(-62, goal_y - 1.5, 800, {.maxSpeed = 35}, false);
    moveStraight(-1, 200, {.minSpeed = 30}, false);
    rb(100);
    moveStraight(-2, 400, {}, false); // back up so we don't get caught on the wall
    chassis.swingToHeading(350, DriveSide::RIGHT, 600, {}, false);
    intake.intake();
    score_toggle.firePiston(false);
    intake.outake();
    float timeCrossEnd = 1250; // least amount of time
    chassis.arcade(110, 0); // cross barrier
    wait(200);
    chassis.arcade(98, 0);
    wait(timeCrossEnd - 200);
    chassis.arcade(0, 0); // stop
    //                       //  DONE :) 97 potential points

    // BUILD ISSUES:
    //  1.) Balls in jaws at least like 6 times
    //  2.) intake jam last ball prevent mid goal scoring
    //  3.) couple times matchloader get stuck at lip of matchlodaer
}

void elimRight() { // low goal 4 high 3 plus push
    pros::Task debug(print_pos); // debug tasks + reset tasks
    // #1
    chassis.setPose(70 - 25.5, 70 - robot.get_distance_right_side(), 270);
    intake_1.move_voltage(12000);
    chassis.moveToPose(20.2, 21.9, 303, 1000, {.lead = 0.28, .minSpeed = 50, .earlyExitRange = 0.1}, true);
    wait(650);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    robot.ram(60, 270);
    real_brake();
    unloader.firePiston(false);
    wait(200);
    chassis.turnToHeading(225.5, 600, {}, false); // face towards lowgoal
    chassis.moveToPoint(10.5, 9.2, 800, {}, true); // gotwards low goal
    chassis.waitUntilDone();
    chassis.turnToHeading(230, 180, {.minSpeed = 50, .earlyExitRange = 0.1}, false);
    robot.ram(-75, 140);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    intake_1.move_voltage(-6000);
    intake_2.move_voltage(9000);
    intake_3.move_voltage(-6000);
    wait(2500); // give time to score
    intake.stop();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // hold so robot doesn't drift
    //-46.9
    moveStraight(-49, 1650, {.maxSpeed = 85, .minSpeed = 1, .earlyExitRange = 1}, true);
    wait(200);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    real_brake();
    chassis.turnToHeading(90, 800, {.maxSpeed = 60}, false); // face towards matchloaders
    real_brake();
    intake.intake();
    robot.ram(75, 1800);
    moveStraight(-9, 350, {.forwards = false, .minSpeed = 50, .earlyExitRange = 1});
    chassis.moveToPose(26, 45, 90, 750, {.forwards = false, .lead = 0.3, .minSpeed = 60, .earlyExitRange = 1}, false);
    robot.ram(-115, 450); // get into goal
    score_toggle.firePiston(true);
    unloader.firePiston(false);
    robot.ram(-85, 700);
    intake.stop();
    moveStraight(3, 170, {.minSpeed = 50, .earlyExitRange = 1}, false);
    chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 800, {}, false);
    moveStraight(7.6, 400, {}, false);
    score_toggle.firePiston(false);
    chassis.turnToHeading(90, 550, {}, false);
    intake.outake();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.ram(-90, 1100);
    wait(900);
    intake.stop();
}

void norcalRight() {
    pros::Task debug(print_pos);

    // grab first three balls
    chassis.setPose(70 - 25.5, 70 - robot.get_distance_right_side(), 270);
    // intake_1.move_voltage(12000);
    intake.intake();
    chassis.moveToPose(20.2, 21.9, 303, 1150, {.lead = 0.28, .minSpeed = 50, .earlyExitRange = 0.1}, true);
    wait(500);
    unloader.firePiston(true);
    chassis.waitUntilDone();

    // go to matchloader
    chassis.turnToHeading(413, 540, {.maxSpeed = 110}, false);
    chassis.waitUntilDone();
    chassis.moveToPose(47.4, 48, 408, 1230, {}, false);
    chassis.turnToHeading(450, 350, {.maxSpeed = 1000}, false);
    robot.ram(85, 1000);
    // moveStraight(-9, 300, {}, false);
    /*chassis.moveToPose(33.288, 46.8, 450, 1000,
                       {.forwards = false, .lead = 0.3, .minSpeed = 55, .earlyExitRange = 1},
                       false); // go back into goal*/

    moveStraight(-4, 250, {.minSpeed = 1, .earlyExitRange = 1}, false);
    chassis.moveToPoint(30, 47.5, 600, {.forwards = false}, false);
    robot.ram(-105, 300); // get into goal
    score_toggle.firePiston(true); // open up scoring hood
    intake.intake(); // start scoring
    robot.ram(-85, 1300);
    unloader.firePiston(false); // Matchloader up
    brake();

    //
    // descore.firePiston(false);
    moveStraight(3, 170, {.minSpeed = 50, .earlyExitRange = 1}, false);
    chassis.swingToHeading(360, lemlib::DriveSide::LEFT, 800, {}, false);
    moveStraight(7.75, 400, {}, false);
    score_toggle.firePiston(false);
    chassis.turnToHeading(446, 550, {}, false);
    intake.outake();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.ram(-90, 1100);
    wait(900);
    intake.stop();
}

void elimRightFast() { // low goal 4 high 3 plus push
    // debug tasks + reset tasks
    pros::Task debug(print_pos);

    // #1
    chassis.setPose(70 - 25.5, 70 - robot.get_distance_right_side(), 270);
    intake_1.move_voltage(12000);
    chassis.moveToPose(20.2, 21.9, 303, 1000, {.lead = 0.28, .minSpeed = 50, .earlyExitRange = 0.1}, true);
    wait(650);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    robot.ram(60, 270);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    wait(100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    unloader.firePiston(false);
    chassis.turnToHeading(225.5, 600, {}, false); // face towards lowgoal
    chassis.moveToPoint(10.5, 9.2, 800, {}, true); // gotwards low goal
    chassis.waitUntilDone();
    chassis.turnToHeading(230, 180, {.minSpeed = 50, .earlyExitRange = 0.1}, false);
    intake_1.move_voltage(-6000);
    intake_2.move_voltage(9000);
    intake_3.move_voltage(-6000);
    robot.ram(-75, 90);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    wait(1900); // give time to score
    intake.stop();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // hold so robot doesn't drift
    moveStraight(-44.5, 1650, {.minSpeed = 60, .earlyExitRange = 1}, true); // go back towards matchloaders
    wait(200);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    wait(100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToHeading(90, 660, {}, false); // face towards matchloaders
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    wait(100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    intake.intake();
    moveStraight(3, 300, {.minSpeed = 80, .earlyExitRange = 1}, false);
    robot.ram(75, 1700);
    moveStraight(-9, 350, {.forwards = false, .minSpeed = 50, .earlyExitRange = 1});
    chassis.moveToPose(26, 45.5, 90, 750, {.forwards = false, .lead = 0.3, .minSpeed = 70, .earlyExitRange = 1},
                       false); // go back into matchloader
    score_toggle.firePiston(true);
    robot.ram(-115, 100); // get into goal
    unloader.firePiston(false);
    robot.ram(-85, 600);
    intake.stop();
    moveStraight(3, 170, {.minSpeed = 50, .earlyExitRange = 1}, false);
    chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 800, {}, false);
    moveStraight(7.6, 400, {}, false);
    score_toggle.firePiston(false);
    chassis.turnToHeading(90, 550, {}, false);
    intake.outake();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.ram(-105, 650);
    wait(900);
    intake.stop();
}

void highlandQuals() {
    // Starting Position
    lemlib::Pose start_pose(robot.get_distance_right_side() - 70, -18, 180);

    // #1
    chassis.setPose(start_pose);
    descore.firePiston(true);
    chassis.moveToPoint(-37.267, -46, 800, {.maxSpeed = 70, .minSpeed = 1, .earlyExitRange = 0.3}, false); //
    chassis.turnToHeading(270, 750, {}, false);
    unloader.firePiston(true);
    intake.intake();
    chassis.waitUntilDone();
    moveStraight(15, 800, {.minSpeed = 1, .earlyExitRange = 1}, false); // get close
    robot.ram(85, 700);
    moveStraight(-9, 300, {}, false);
    chassis.moveToPose(-30.535, -47.335, 270, 900,
                       {.forwards = false, .lead = 0.3, .minSpeed = 55, .earlyExitRange = 1},
                       false); // go back into goal
    robot.ram(-105, 300); // get into goal
    score_toggle.firePiston(true); // open up scoring hood
    intake.intake(); // start scoring
    robot.ram(-85, 660);
    unloader.firePiston(false); // Matchloader up
    brake();

    // // #2
    // moveStraight(4, 250, {}, false); //move so dont hit the goals
    // chassis.swingToHeading(20, DriveSide::RIGHT, 750, {}, false);
    // score_toggle.firePiston(false); // close scoring hood
    // added stuff
    float angError = fabs(chassis.getPose().theta - 270);
    chassis.setPose(angError < 3.5 ? -29.5 : chassis.getPose().x, robot.get_distance_left_side() - 70,
                    angError < 3.5 ? 270 : chassis.getPose().theta);

    // #2
    // moveStraight(4, 250, {}, false); //move so dont hit the goals
    chassis.swingToHeading(405, DriveSide::RIGHT, 900, {}, false);
    score_toggle.firePiston(false); // close scoring hood
    moveStraight(26.5, 1200, {.maxSpeed = 60}, false);
    wait(1000);
    moveStraight(-1.5, 300, {}, false);
    intake.outake();
    intake_1.move_voltage(-6000);
}

void highlandElims() {
    // Starting Position
    lemlib::Pose start_pose(robot.get_distance_right_side() - 70, -18, 180);

    // #1
    chassis.setPose(start_pose);
    // descore.firePiston(true);
    chassis.moveToPoint(-37.267, -46, 800, {.maxSpeed = 70, .minSpeed = 1, .earlyExitRange = 0.3}, false); //
    chassis.turnToHeading(270, 750, {}, false);
    unloader.firePiston(true);
    intake.intake();
    chassis.waitUntilDone();
    moveStraight(15, 800, {.minSpeed = 1, .earlyExitRange = 1}, false); // get close
    robot.ram(85, 700);
    moveStraight(-9, 300, {}, false);
    chassis.moveToPose(-30.535, -47.335, 270, 900,
                       {.forwards = false, .lead = 0.3, .minSpeed = 55, .earlyExitRange = 1},
                       false); // go back into goal
    robot.ram(-105, 300); // get into goal
    score_toggle.firePiston(true); // open up scoring hood
    intake.intake(); // start scoring
    robot.ram(-85, 660);
    unloader.firePiston(false); // Matchloader up
    brake();

    //
    // descore.firePiston(false);
    moveStraight(3, 170, {.minSpeed = 50, .earlyExitRange = 1}, false);
    chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 800, {}, false);
    moveStraight(6, 400, {}, false);
    score_toggle.firePiston(false);
    chassis.turnToHeading(270, 550, {}, false);
    intake.outake();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.ram(-90, 1100);
    wait(900);
    intake.stop();

    //

    // // #
}

void awp() {
    // Starting Position
    lemlib::Pose start_pose(robot.get_distance_right_side() - 70, -20, 180);

    // #1
    chassis.setPose(start_pose);
    // descore.firePiston(true);
    chassis.moveToPoint(-37.267, -44.8, 920, {.maxSpeed = 70, .minSpeed = 1, .earlyExitRange = 0.3}, false); //
    rb(50);
    unloader.firePiston(true);
    chassis.turnToHeading(270, 750, {.minSpeed = 1, .earlyExitRange = 1}, false);
    intake.intake();
    chassis.moveToPose(-55, -49, 270, 900, {.lead = 0.3, .minSpeed = 10, .earlyExitRange = 1}, false);
    robot.ram(80, 740); //get matchloads
    moveStraight(-9, 300, {.minSpeed = 1, .earlyExitRange = 0.5}, false);
    chassis.moveToPose(-30.535, -48.6, 270, 900, {.forwards = false, .lead = 0.2, .minSpeed = 55, .earlyExitRange = 1},
                       true); // go back into goal
    chassis.waitUntilDone();
    score_toggle.firePiston(true); // open up scoring hood
    robot.ram(-105, 300); // get into goal
    intake.intake(); // start scoring
    robot.ram(-85, 750);
    unloader.firePiston(false); // Matchloader up
    float angError = fabs(chassis.getPose().theta - 270);
    chassis.setPose(
        angError < 3.5 ? -29.5 : chassis.getPose().x, angError < 3.5 ? -46.8 : chassis.getPose().y,
        chassis.getPose()
            .theta); // reset y and x, dont reset angle because not too far in auton, angle shouldn't have drifted far

    // #2
    moveStraight(6, 280, {}, true); // move so dont hit the goals
    intake.outake();
    intake_1.move_voltage(0);
    chassis.waitUntilDone();
    intake.stop();
    chassis.swingToHeading(20, DriveSide::RIGHT, 650, {.minSpeed = 1, .earlyExitRange = 1}, false);
    score_toggle.firePiston(false); // close scoring hood
    intake.intake();
    chassis.moveToPose(-19.7, -13.1, 360, 1000, {.lead = 0.4, .minSpeed = 50, .earlyExitRange = 1},
                       true); // go towards 1st trio
    wait(400);
    intake.outake();
    wait(80);
    intake.intake();
    chassis.waitUntilDone();
    // descore.firePiston(false);
    chassis.moveToPoint(-21.5, 20, 900, {.maxSpeed = 100},
                        true); // go towards 2nd trio
    wait(780);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-47, 49.5, 550, {.minSpeed = 1, .earlyExitRange = 2}, false);
    chassis.moveToPoint(-47, 45.5, 750, {.minSpeed = 20, .earlyExitRange = 0.5}, false); // go near long goal
    chassis.turnToHeading(270, 420, {}, false); // align with long goal
    chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_right_side(), chassis.getPose().theta);
    chassis.moveToPose(-27.5, 47, 270, 950, {.forwards = false, .lead = 0.25, .minSpeed = 1, .earlyExitRange = 0.5},
                       true);
    chassis.waitUntilDone();
    score_toggle.firePiston(true); // open up scoring hood before ramming in so that the robot alligns better
    robot.ram(-105, 200);

    intake.intake(); // start scoring
    robot.ram(-85, 790); //score like 4-5 balls
    angError = fabs(chassis.getPose().theta - 270);
    if (angError < 3.5) { chassis.setPose(-28, 46.8, chassis.getPose().theta); } // reset position only if alligned
    intake.outake();
    intake_1.move_voltage(0); // dont let balls out
    chassis.moveToPose(-55.5, 45.7, 270, 950, {.forwards = true, .lead = 0.3, .minSpeed = 10, .earlyExitRange = 1.5},
                       true); //go to matchloader
    wait(100);
    intake.stop();
    score_toggle.firePiston(false);
    intake.intake();
    chassis.waitUntilDone();
    robot.ram(87, 400); // get balls
    robot.ram(80,350); //go a bit slower
    moveStraight(-6, 200, {.forwards = false, .minSpeed = 1, .earlyExitRange = 0.5}, false);
    chassis.moveToPose(-11.6, 11, 315, 1400, {.forwards=false, .minSpeed=1, .earlyExitRange=0.5}, true); //head over midgoal
    wait(350); //wait until a little closer
    intake.outake();
    intake_1.move_voltage(0);
    wait(200);
    intake.stop();
    chassis.cancelMotion();
    chassis.moveToPoint(-14.1, 13.5, 900, {.forwards=false, .minSpeed=1, .earlyExitRange=1.3}, true); //switch over to moveToPoint to speed up the motion
    wait(550);
    intake.mid_goal();
    intake_3.move_voltage(-5500);
    chassis.waitUntilDone();
    intake.mid_goal(); // score mid goal
    intake_3.move_voltage(-5500);
    robot.ram(-80, 1000);
}

void left_4_plus_3_elims_state_wa_region_team_98040C_long_goal_first(){
    chassis.setPose(-54,18.5, 180); 

    //1
    chassis.moveToPoint(-45.6, 47.4, 800, {.forwards=false, .minSpeed=1, .earlyExitRange=1}, true);
    lemlib::Timer lower_unload(240);
    float error = 34.5 - chassis.getPose().y; 
    while(error > 1){
        error = 34.5 - chassis.getPose().y; 
        if(lower_unload.isDone()){
            unloader.firePiston(true);
        }
        wait(15);
    }
    chassis.cancelMotion();
    chassis.turnToHeading(270, 600, {.minSpeed=1, .earlyExitRange=1}, false); // face towards matchloader
    intake.intake();
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPoint(pose.x + 20 * sin(lemlib::degToRad(pose.theta)),
                        pose.y + 20 * cos(lemlib::degToRad(pose.theta)), 950, {.maxSpeed=70, .minSpeed=69}, false); //add 2 for drift in y direction

    moveStraight(-6, 150, {.minSpeed=119, .earlyExitRange=2}, true); //back up from matchloader
    wait(90);
    chassis.setPose(chassis.getPose().x, 70-robot.get_distance_right_side(), chassis.getPose().theta); // reset again
    chassis.cancelMotion();

    chassis.moveToPose(-28, 46.9, 270, 900, {.forwards=false, .lead=0.3, .minSpeed=80, .earlyExitRange=1}, false); // go back into goal
    robot.ram(-105, 250); //get into goal
        score_toggle.firePiston(true); // open up scoring hood so we can allign properly
        unloader.firePiston(false);
    robot.ram(-80, 610); //give time to score the 4 balls
      float angError = fabs(chassis.getPose().theta - 270);
    if(angError < 3.5){
    chassis.setPose(
        -29.5,  46.8,
        chassis.getPose()
            .theta); // reset y and x, dont reset angle because not too far in auton, angle shouldn't have drifted fa
    }else if(angError < 16){ //else if its a little mis alligned use wall sensor
        chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_right_side(), chassis.getPose().theta);
    }
    

    
    //2
    chassis.swingToHeading(160, DriveSide::LEFT, 720, {.maxSpeed=126, .minSpeed=125, .earlyExitRange=2}, true);
    angError = chassis.getPose().theta  - 180; 
    lemlib::Timer maxTime(680);
    while(angError > 2 && !maxTime.isDone()){ //check if within time and close to 180 to reset pos
        angError = chassis.getPose().theta  - 180; 
        wait(13);
    }
    chassis.setPose(robot.get_distance_right_side() - 70, chassis.getPose().y, chassis.getPose().theta); //quick reset when robot is relatviely straight with the wall
    chassis.waitUntilDone();
    score_toggle.firePiston(false); //close hood
    intake.intake();
    chassis.moveToPoint(-23, 24, 900, {.minSpeed=110, .earlyExitRange=1}, true); //go get trio of balls
    lemlib::Timer ballCapture(250);
    error = chassis.getPose().y - 30; 
    while(error > 1){
        error = chassis.getPose().y - 30; 
        wait(20); //dont hog CPU
        if(ballCapture.isDone()){
            unloader.firePiston(true);
        }
    }
    chassis.cancelMotion(); //once we get the balls stop
    chassis.moveToPoint(-8, 9, 1200, {.forwards=false, .minSpeed=80, .earlyExitRange=1}, true); //head over to midgoal
    lemlib::Timer middle_intake(450);
    lemlib::Timer max_time(1200);
    error = -13 - chassis.getPose().x;
    while(error > 1 && !max_time.isDone()){
        error = -13 - chassis.getPose().x;
         if(middle_intake.isDone()){
        intake.mid_goal(); //score mid goal
        intake_1.move_voltage(12000);
        intake_3.move_voltage(-9000);
         }
         wait(18); //dont hog cpu
    }
    chassis.cancelMotion(); 
    chassis.turnToPoint(0,-3, 400, {.forwards=false}, false);
    // chassis.turnToPoint(0,2, 300, {}, false); //TODO maybe add this as turning is ah
    robot.ram(-80, 600); //score for a bit
      unloader.firePiston(false);
          robot.ram(-80, 600); //score for a bit
    moveStraight(5, 170, {}, true);
    wait(100);
    chassis.cancelMotion();


    chassis.moveToPoint(-34, 41, 1300, {.minSpeed=100, .earlyExitRange=1}, true);
    error = 29.5 - chassis.getPose().y; //cancel motion once past y=30
    lemlib::Timer wingTime(250);
     while(error > 1){
        error = 29.5 - chassis.getPose().y; 
        wait(15);
        if(wingTime.isDone()){
            descore.firePiston(true);
        }
    }
    chassis.cancelMotion();
    chassis.turnToHeading(270, 450, {.minSpeed=95, .earlyExitRange=1}, false);
    descore.firePiston(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.ram(-90, 900); //go back

}
void right_4_3() {
    // debug tasks + reset tasks
    pros::Task debug(print_pos);

    // Starting Position
    lemlib::Pose start_pose(robot.get_distance_right_side() - 70, -18, 180);

    // #1
    chassis.setPose(start_pose);
    descore.firePiston(true);
    chassis.moveToPose(-37.267, -47.737, 151.82, 1000, {.minSpeed = 50, .earlyExitRange = 0.5},
                       false); // go towards quad balls
    chassis.turnToHeading(270, 680, {.minSpeed = 50, .earlyExitRange = 1}, false);
    chassis.moveToPose(-50, -48.007, 270, 1600, {.lead = 0.25, .minSpeed = 50, .earlyExitRange = 3},
                       true); // go towards quad balls
    unloader.firePiston(true);
    intake.intake();
    chassis.waitUntilDone();
    robot.ram(93, 750);
    moveStraight(-9, 300, {}, false);
    chassis.moveToPose(-30.535, -48.335, 270, 900,
                       {.forwards = false, .lead = 0.3, .minSpeed = 55, .earlyExitRange = 1},
                       false); // go back into goal
    robot.ram(-105, 300); // get into goal
    score_toggle.firePiston(true); // open up scoring hood
    intake.intake(); // start scoring
    robot.ram(-85, 1200);
    unloader.firePiston(false); // Matchloader up
    brake();
    float angError = fabs(chassis.getPose().theta - 270);
    chassis.setPose(angError < 3.5 ? -29.5 : chassis.getPose().x, robot.get_distance_left_side() - 70,
                    angError < 3.5 ? 270 : chassis.getPose().theta);

    // #2
    // moveStraight(4, 250, {}, false); //move so dont hit the goals
    chassis.swingToHeading(405, DriveSide::RIGHT, 900, {}, false);
    score_toggle.firePiston(false); // close scoring hood
    moveStraight(26.5, 1200, {.maxSpeed = 60}, false);
    wait(1000);
    moveStraight(-1.5, 300, {}, false);
    intake.outake();
    intake_1.move_voltage(-6000);
}

void skillsold() {
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;
    color_sort_auton = false;
    // Starting Position
    lemlib::Pose start_pose(-48, -13.5, 180);

    // Other Poses
    lemlib::Pose top_left_matchloading(-51.497, -49.995, 270);
    lemlib::Pose bottom_right_goal(40.148, -65.064, 90);
    lemlib::Pose bottom_right_goal_alligner(33.244, -46.3, 90);
    lemlib::Pose bottom_right_matchload(50.61, -46.032, 90);
    lemlib::Pose last_goal(-24.509, 43, -90);
    lemlib::Pose first_goal(24.509, -47, -90);

    // SEQUENCE #1
    chassis.setPose(start_pose);
    unloader.firePiston(true); // Piston down to unload matchloads
    intake.intake(); // Start intake
    mP(top_left_matchloading, 1800, {.lead = 0.4}, false); // Move to top left matchloading zone
    robot.ram(95, 500); // first ram quickly into it so we can dislodge balls
    robot.ram(80, 1800); // slow down as not to accidentally track
    moveStraight(-6, 500, {.forwards = false}, false); // Back away from matchloaders
    unloader.firePiston(false); // Matchloader up
    chassis.turnToHeading(128, 650, {}, false); // turn to get ready to traverse field
    chassis.moveToPose(-24.18, -64.617, 90, 1600, {}, false); // Traverse to other side of field
    intake.stop(); // stop intake to prevent burnout
    chassis.follow(get_to_bottom_right_goal_txt, 15, 3500, true, true); // Follow path to bottom right goal
    wait(1000);
    wall_reset_enabled = false;
    chassis.waitUntilDone();
    wall_reset_enabled = true;
    chassis.turnToHeading(75, 360, {}, false); // Turn to face goal
    // chassis.turnToPoint(first_goal.x, first_goal.y, 150, {.forwards = false}, false);
    chassis.moveToPose(28.6, -47.8, 90, 350, {.forwards = false, .lead = 0.3}, false); // Move to goal
    chassis.turnToHeading(90, 450, {}, false);
    robot.ram(-65, 1100); // move back to get into alligner
    descore.firePiston(true); // Raise hood to enable scoring
    intake.outake(); // score balls
    intake_1.move_voltage(0); // dont let any balls out
    wait(250);
    intake.intake();
    robot.ram(-100, 2300); // move back to get into alligner
    intake.stop();

    // SEQUENCE #2
    intake.intake(); // Start intake
    chassis.setPose(27.3, -47, chassis.getPose().theta);
    unloader.firePiston(true); // Piston down to unload matchloads
    mP(bottom_right_matchload, 850, {.forwards = true, .lead = 0.3, .earlyExitRange = 1},
       false); // Move to bottom right matchloaders
    descore.firePiston(false); // disable scoring hood
    robot.ram(95, 500); // first ram quickly into it so we can dislodge balls
    robot.ram(80, 1800); // slow down as not to accidentally track
    moveStraight(-6, 300, {.forwards = false, .earlyExitRange = 2}, false); // Back away from matchloaders
    mP(bottom_right_goal_alligner, 830, {.forwards = false, .lead = 0.3}, false); // Move to bottom right goal alligner
    robot.ram(-65, 750); // move back to get into alligner
    descore.firePiston(true); // Raise hood to enable scoring
    intake.intake(); // score balls
    robot.ram(-90, 2300); // move back to get into alligner
    intake.stop();
    moveStraight(7, 330, {}, false); // get out to push the thing back in
    descore.firePiston(false); // disable scoring hood
    unloader.firePiston(false); // Matchloader up
    robot.ram(-85, 600); // push balls deeper into goal also alligning robot via alligner
    descore.firePiston(true); // open back up so any jammed balls can exit
    intake.intake();

    // SEQUENCE #3
    chassis.setPose(27.3, -46.5, chassis.getPose().theta); // set position in goal alligner
    chassis.moveToPose(63, -18, 24.4, 1950, {.maxSpeed = 85}, false); // go towards parking zone area
    intake.stop();
    chassis.waitUntilDone();
    robot.ram(65, 900); // make sure head on with parking zone
    unloader.firePiston(true); // smack balls in parking zone out
    wait(400); // give balls time to move out
    unloader.firePiston(false); // get back up
    wait(300);
    intake.outake(); // push balls away to ensure clearing
    chassis.turnToHeading(10, 200, {}, false);
    robot.ram(100, 400);
    robot.ram(95, 2100);
    chassis.turnToHeading(0, 200, {}, false);

    // SEQUENCE #4
    wall_reset_enabled = true;
    chassis.setPose((70 - robot.get_distance_left_side()), (70 - robot.get_distance_front()), chassis.getPose().theta);
    chassis.moveToPose(60.8, 15.5, 0, 1000, {.forwards = false, .lead = 0.2}, false); // go back
    descore.firePiston(false); // close hood
    chassis.moveToPose(43.3, 47.5, -76, 2000, {}, false); // go in front of matchloader
    chassis.turnToHeading(90, 700, {}, false); // turn toward matchloader
    intake.intake();
    unloader.firePiston(true);
    wait(100);
    robot.ram(100, 700); // first ram quickly into it so we can dislodge balls
    robot.ram(80, 2100); // slow down as not to accidentally track
    moveStraight(-10, 500, {}, false);
    chassis.turnToHeading(330, 600, {}, false);
    unloader.firePiston(false); // matchload up
    intake.stop();
    chassis.follow(last_goal_skills_txt, 16, 3600, true, false);
    chassis.turnToHeading(-90, 550, {}, false);
    // chassis.turnToPoint(last_goal.x, last_goal.y, 200, {.forwards = false}, false);
    robot.ram(-85, 1100); // go back into goal
    chassis.setPose(-30.7, 47.51, chassis.getPose().theta); // now we're alligned reset position
    descore.firePiston(true); // open up hood to enable scoring
    intake.intake();
    robot.ram(-90, 2300);
    intake.stop();
    unloader.firePiston(true); // matchloader out to start getting balls
    intake.intake();
    chassis.moveToPose(-54.435, 46.1, -90, 1300, {.lead = 0.3}, false);
    descore.firePiston(false);
    robot.ram(85, 2500); // ram into matchload
    moveStraight(-24, 1100, {}, false); // get back out of matchload area
    chassis.turnToPoint(last_goal.x, last_goal.y + 3, 700, {.forwards = false}, false);
    robot.ram(-85, 500); // get into alligner
    descore.firePiston(true);
    robot.ram(-90, 3000);
    unloader.firePiston(false);
    intake.stop();

    // SEQUENCE #5
    chassis.setPose(-27.5, 47.5, chassis.getPose().theta);
    chassis.moveToPose(-61.5, 21.5, 180 + 24.4, 1800, {.maxSpeed = 85}, false); // go towards parking zone area
    chassis.turnToHeading(183, 200, {}, false);
    robot.ram(58, 1200); // make sure head on with parking zone
    unloader.firePiston(true); // smack balls in parking zone out
    wait(400); // give balls time to move out
    unloader.firePiston(false); // get back up
    wait(300);
    intake.outake(); // push balls away to ensure clearing
    robot.ram(120, 300); // strong inital push
    robot.ram(80, 900); // keep going
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void SSAWP() {
    lemlib::Timer auton_length(15000);
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;
    color_sort_auton = false;
    lemlib::Pose start(-47, -15, 180); // need to fix
    // lemlib::TurnToHeadingParams turnParams;
    // turnParams.earlyExitRange = 5.0;
    // turnParams.minSpeed = 25;
    // turnParams.maxSpeed = 90;

    chassis.setPose(start);
    intake.intake();
    moveStraight(32, 900, {}, false); // go to right side matchloader
    unloader.firePiston(true);
    chassis.turnToHeading(270, 500, {}, false);
    intake.intake();
    // moveStraight(50, 600, {}, false);
    robot.ram(90, 1250); // get matchloader blocks
    // moveStraight(-6, 300, {}, false);
    chassis.moveToPose(-32.558, -53, 270, 1100, {.forwards = false, .lead = 0.3, .minSpeed = 70, .earlyExitRange = 3},
                       false); // go to long goal
    descore.firePiston(true); // open hood to score
    robot.ram(-95, 600); // score on goal
    robot.ram(-85, 400); // score on goal
    unloader.firePiston(false); // unloader up
    chassis.setPose(-27.3, -46, chassis.getPose().theta); // reset position
    moveStraight(2, 180, {.minSpeed = 90}, false);
    chassis.swingToHeading(50, DriveSide::RIGHT, 600, {}, false); // swing towards 3 balls
    descore.firePiston(false); // clase hood to
    moveStraight(19, 870, {.maxSpeed = 50}, false); // get first 3 balls
    wait(200); // wait to get balls
    chassis.turnToHeading(-2, 400, {}, false);
    moveStraight(49.5, 1100, {}, true);
    wait(650); // wait until close to balls
    // unloader.firePiston(true);
    chassis.waitUntilDone();
    unloader.firePiston(false);
    chassis.turnToHeading(-44, 400, {}, false);
    chassis.moveToPose(-9.8, 9.83, 315, 900, {.forwards = false}, true); // go to mid goal
    intake.outake();
    intake_1.move_voltage(0);
    wait(750);
    intake.stop();
    chassis.waitUntilDone();
    intake.mid_goal();
    wait(900); // give time to score on mid goal
    intake.intake();
    chassis.moveToPose(-44.98, 45.7, 270, 1400, {.lead = 0.17}, true); // go to matchloader
    chassis.waitUntilDone();
    intake.intake();
    unloader.firePiston(true);
    chassis.turnToHeading(270, 100, {}, false);
    robot.ram(100, 350); // get matchloader blocks
    robot.ram(85, 700);
    chassis.moveToPose(-33.5, 45.5, 270, 800, {.forwards = false, .minSpeed = 80}, false); // go to last long goal
    descore.firePiston(true); // open hood to score
    intake.intake();
    robot.ram(-100, 1200); // score on goal
}

void elimLeftButOnRight() {
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;
    color_sort_auton = false;
    lemlib::Pose start(-50, -16, 90);

    chassis.setPose(start);
    intake.intake();
    chassis.follow(elimLeftButOnRight_txt, 16, 3000, true, true);
    wait(500);
    unloader.firePiston(true); // capture balls
    wait(350); // keep it down
    unloader.firePiston(false);
    chassis.waitUntilDone();
    wait(250);
    chassis.moveToPose(-20.1, -18.84, 138, 1150, {.forwards = false, .minSpeed = 55}); // go back to the mid goal area
    chassis.turnToHeading(238, 500, {}, true); // turm toward midgoal
    intake.outake();
    intake_1.move_voltage(0);
    wait(450);
    intake.stop();
    chassis.waitUntilDone();
    chassis.moveToPose(-10.9, -10.6, 225, 800, {.forwards = false, .lead = 0.3}, false); // get into middle goal
    intake.mid_goal();
    wait(1000);
    intake.stop();
    chassis.moveToPose(-52.7, -49, -90, 1600, {.lead = 0.3}, true); // get to matchload
    wait(500);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    intake.intake();
    robot.ram(89, 1000); // get preload balls
    chassis.moveToPose(-34.3, -49.1, -92, 1300, {.forwards = false, .minSpeed = 65},
                       true); // line up with the scoring thing
    wait(200);
    unloader.firePiston(false); // unloader up
    chassis.waitUntilDone();
    robot.ram(-90, 500); // get back into it
    descore.firePiston(true);
    intake.intake();
    robot.ram(-85, 1600); // give time to score
    intake.stop();
    moveStraight(6, 300, {}, false);
    descore.firePiston(false);
    robot.ram(-89, 340); // push balls deeper
    moveStraight(15, 600, {.earlyExitRange = 1.3}, false);
    chassis.turnToHeading(-133, 400, {}, false);
    chassis.moveToPose(-24.95, -58.58, -90, 1500, {.forwards = false, .lead = 0.5}, true); // get next to the goal
    score_toggle.firePiston(true);
    chassis.waitUntilDone();
    chassis.setPose(robot.get_distance_front() - 70, 70 - robot.get_distance_left_side(),
                    chassis.getPose().theta); // reset position
    float front_distance = robot.get_distance_front();
    float error = 60 - front_distance;
    float prevError = error;
    float theta = chassis.getPose().theta;
    score_toggle.firePiston(false);
    while (true) {
        theta = chassis.getPose().theta;
        if (fabs(theta + 90) > 9) { // kind of off course
            chassis.turnToHeading(-90, 300, {}, false); // get back on course
        }
        if (error > 1.5 && error < 8) {
            chassis.arcade(-35, 0); // already really close
        } else if (error > 8 && error < 12) {
            chassis.arcade(-80, 0);
        } else if (error > 12) {
            chassis.arcade(-90, 0);
        } else {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        }
        prevError = error;
        error = 60 - robot.get_distance_front();
        wait(10);
    }
}

void sevenBlockRight() {
    lemlib::Timer auton_length(15000);
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;

    intake.intake();
    chassis.setPose(-46, 0, 90);
    chassis.follow(sevenBallRight_txt, 16, 2500, true, true);
    wait(910);
    unloader.firePiston(true); // capture balls
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 150, {}, false);
    robot.ram(90, 1200); // get matchloader blocks
    moveStraight(-6, 300, {}, false);
    chassis.moveToPose(-32.07, -46.61, 270, 900, {.forwards = false, .lead = 0.3, .minSpeed = 55, .earlyExitRange = 3},
                       false); // go to goals
    intake.intake();
    descore.firePiston(true); // open hood to score
    robot.ram(-95, 1750); // score on goal
    unloader.firePiston(false); // unloader up
    // chassis.setPose(-27.3, -46, chassis.getPose().theta); // reset position
    moveStraight(5, 200, {.minSpeed = 90}, false);
    descore.firePiston(false); // close hood
    robot.ram(-100, 1000); // back away

    // moveStraight(5, 180, {.minSpeed = 90}, false);
    // descore.firePiston(false); // close hood
    // chassis.turnToHeading(-45, 400, {.earlyExitRange = 4}, false);
    // chassis.moveToPose(-40, -33.75, 345, 1000, {.forwards = true}, false); // get to next blocks
    // chassis.turnToHeading(-90, 300, {}, false);
    // float front_distance = robot.get_distance_front();
    // float error = 65 - front_distance;
    // float prevError = error;
    // float theta = chassis.getPose().theta;
    // intake.intake();
    // score_toggle.firePiston(false);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // chassis.turnToHeading(-90, 300, {}, false);
    // while (true) {
    //     theta = chassis.getPose().theta;
    //     if (fabs(theta - 270) > 9) { // kind of off course
    //         chassis.turnToHeading(-90, 300, {}, false); // get back on course
    //     }
    //     if (error > 1.5 && error < 8) {
    //         chassis.arcade(-50, 0); // already really close
    //     } else if (error > 8 && error < 12) {
    //         chassis.arcade(-90, 0);
    //     } else if (error > 12) {
    //         chassis.arcade(-95, 0);
    //     } else {
    //         chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //         chassis.arcade(0, 0);
    //     }
    //     prevError = error;
    //     error = 60 - robot.get_distance_front();
    //     wait(10);
    //     if (auton_length.getTimePassed() == 14850) { // lift not to touch the ball
    //         robot.ram(80, 150);
    //     }
    // }
}

void elimRightButOnLeft() { // the elim that we made on the right side of red but now is on left
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;
    color_sort_auton = false;
    lemlib::Pose start(-50, 16, 90);

    chassis.setPose(start);
    chassis.follow(elimRightButLeft_txt, 16, 2400, true, true); // get to two balls out there
    intake.intake(); // start intake
    wait(460);
    unloader.firePiston(true); // capture balls
    wait(350); // keep it down
    unloader.firePiston(false);
    chassis.waitUntilDone();
    wait(250);
    chassis.moveToPose(-21, 20.3, 43, 1200, {.forwards = false, .minSpeed = 60}); // go back to the mid goal area
    chassis.turnToHeading(315, 300);
    chassis.moveToPose(-45.5, 52.8, 270, 1900, {.lead = 0.3, .minSpeed = 55}, true); // get to matchloader
    wait(200);
    unloader.firePiston(true); // get matchloader
    chassis.waitUntilDone();
    robot.ram(89, 1400); // matchload balls
    chassis.moveToPose(-23.13, 53.4, -90, 1300, {.forwards = false, .minSpeed = 65},
                       true); // line up with the scoring thing
    wait(100);
    chassis.waitUntilDone();
    robot.ram(-85, 400);
    descore.firePiston(true);
    robot.ram(-85, 2300); // get back into it
    intake.intake();
    descore.firePiston(false); // bring down hood to ram
    moveStraight(6, 250, {}, false);
    robot.ram(-89, 10000); // push deeper
}

void driveOneINch() { moveStraight(5, 250, {}, false); }

void elimLeft() {
    lemlib::Timer auton_length(15000);
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;
    color_sort_auton = false;
    lemlib::Pose start(-50, 16, 90);

    chassis.setPose(start);
    intake.intake();
    chassis.follow(elimsLeft_txt, 16, 3000, true, true);
    wait(470);
    unloader.firePiston(true); // capture balls
    wait(350); // keep it down
    unloader.firePiston(false);
    chassis.waitUntilDone();
    wait(250);
    chassis.moveToPose(-20.1, 18.84, 48, 1150, {.forwards = false, .minSpeed = 55}); // go back to the mid goal area
    chassis.turnToHeading(-32, 500, {}, true); // turm toward midgoal
    intake.outake();
    intake_1.move_voltage(0);
    wait(700);
    intake.stop();
    chassis.waitUntilDone();
    chassis.moveToPose(-10.9, 10.6, -45, 800, {.forwards = false, .lead = 0.3}, false); // get into middle goal
    intake.mid_goal();
    wait(1000);
    intake.intake();
    chassis.moveToPose(-52.7, 48.5, -90, 1600, {.lead = 0.3}, true); // get to matchload
    wait(500);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    intake.intake();
    robot.ram(89, 1000); // get preload balls
    chassis.moveToPose(-34.3, 49.1, -88, 1300, {.forwards = false, .minSpeed = 65},
                       true); // line up with the scoring thing
    wait(200);
    chassis.waitUntilDone();
    robot.ram(-90, 500); // get back into it
    descore.firePiston(true);
    intake.intake();
    robot.ram(-85, 2100); // give time to score
    intake.stop();
    moveStraight(6, 300, {}, false);
    descore.firePiston(false);
    robot.ram(-89, 340); // push balls deeper
    unloader.firePiston(false); // unloader up
    moveStraight(15, 600, {.earlyExitRange = 1.3}, false);
    chassis.turnToHeading(-133, 400, {}, false);
    chassis.moveToPose(-24.95, 57.75, -90, 1200, {.forwards = false, .lead = 0.5}, true); // get next to the goal
    chassis.waitUntilDone();
    chassis.setPose(robot.get_distance_front() - 70, 70 - robot.get_distance_left_side(),
                    chassis.getPose().theta); // reset position
    float front_distance = robot.get_distance_front();
    float error = 61.5 - front_distance;
    float prevError = error;
    float theta = chassis.getPose().theta;
    while (true) {
        theta = chassis.getPose().theta;
        if (fabs(theta + 90) > 9) { // kind of off course
            chassis.turnToHeading(-90, 300, {}, false); // get back on course
        }
        if (error > 1.5 && error < 8) {
            chassis.arcade(-50, 0); // already really close
        } else if (error > 8 && error < 12) {
            chassis.arcade(-90, 0);
        } else if (error > 12) {
            chassis.arcade(-95, 0);
        } else {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            chassis.arcade(0, 0);
        }
        prevError = error;
        error = 60 - robot.get_distance_front();
        wait(10);
        if (auton_length.getTimePassed() == 14850) { // lift not to touch the ball
            robot.ram(80, 150);
        }
    }
}

void elimLeftSafe() {
    lemlib::Timer auton_length(15000);
    // debug tasks + reset tasks
    pros::Task debug(print_pos);
    pros::Task reset_pos(reset_robot);
    wall_reset_enabled = false;
    color_sort_auton = false;
    lemlib::Pose start(-50, 16, 90);

    chassis.setPose(start);
    intake.intake();
    chassis.follow(elimsLeft_txt, 16, 3000, true, true);
    wait(470);
    unloader.firePiston(true); // capture balls
    wait(350); // keep it down
    unloader.firePiston(false);
    chassis.waitUntilDone();
    wait(250);
    chassis.moveToPose(-20.1, 18.84, 48, 1150, {.forwards = false, .minSpeed = 55}); // go back to the mid goal area
    chassis.turnToHeading(-32, 500, {}, true); // turm toward midgoal
    intake.outake();
    intake_1.move_voltage(0);
    wait(750);
    intake.stop();
    chassis.waitUntilDone();
    chassis.moveToPose(-10.9, 10.6, -45, 800, {.forwards = false, .lead = 0.3}, false); // get into middle goal
    intake.mid_goal();
    wait(600);
    intake.outake();
    intake_1.move_voltage(0);
    wait(320);
    intake.mid_goal();
    wait(700);
    intake.intake();
    chassis.moveToPose(-52.7, 48.5, -90, 1600, {.lead = 0.3}, true); // get to matchload
    wait(500);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    intake.intake();
    robot.ram(89, 1000); // get preload balls
    chassis.moveToPose(-34.3, 49.1, -88, 1300, {.forwards = false, .minSpeed = 65},
                       true); // line up with the scoring thing
    wait(200);
    chassis.waitUntilDone();
    robot.ram(-90, 500); // get back into it
    descore.firePiston(true);
    intake.intake();
    robot.ram(-85, 3600); // give time to score
}

/**
 * Runs during auto
 *
 * This is an example
 *  routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    pros::Task log(logData); // log data

    left_4_plus_3_elims_state_wa_region_team_98040C_long_goal_first();

    //  intake.outake(); // get them unstuck
    // intake_1.move_voltage(0); // dont move the first stage as to not loose balls
    // wait(320); // give the balls time to get unstuck
    // intake.stop();
    // moveStraight(1.5, 200, {}, true);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // hold so robot doesn't drift
    // intake.mid_goal(); // score mid goal
    // wait(1400); // give time to score
    // intake.mid_goal_weak();
    // lemlib::Timer scoreTime(1500);
    // robot.optical->set_led_pwm(100); // turn on optical sensor led for auton
    // while(scoreTime.getTimePassed() < 1500){
    //     if(intake.detected){
    //         intake.outake(); //moment spotted start outaking
    //         wait(20);
    //     }
    //     if(scoreTime.getTimePassed() > 1400){ //once time over stop
    //         wait(100);
    //         intake.stop();
    //     } else if(!intake.detected){
    //        intake.color_check(true); //check for any blue
    //        wait(20); //give a little delay
    //     }
    // }
    // intake.stop();
}

void dummy_func() {}

void skills_crossing() {
    float timeCross = 2500;
    chassis.arcade(110, 0); // cross barrier
    wait(250);
    chassis.arcade(87, 0);
    wait((timeCross / 2) - 250);
    intake.outake();
    wait(60);
    intake.intake();
    wait((timeCross / 2) - 250);
    float roll = imu.get_roll();
    float prevRoll = -100;
    float dRoll = 2; // change in roll(up and down motion)
    while (dRoll > 0.25) {
        dRoll = fabs(prevRoll - roll);
        prevRoll = roll;
        roll = imu.get_roll();
        wait(200);
    }
    wait(120); // go a little more to be safe
    chassis.arcade(0, 0); // stop
    robot.ram(-60, 1700); // allign with parking zone
    wait(200); // wait for robot to settle
    chassis.setPose(70 - robot.get_distance_left_side(), -17.0f, chassis.getPose().theta); // used to be -17.5
    wait(20); // let register
}

/**
 * Runs in driver control
 */
bool driver_skills = true;
bool currentUpClicked = false;
bool upLastClicked = false;
bool upClicked = false;

void opcontrol() {
    // // robot.color_sort = true; // enable color sorting for now
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // make sure not braked from previous auton(could be braked holding
    //                                                  // control zone in long goal)
    robot.color_sort = false; // enable color sorting for driver control
    pros::Task debug(print_pos); // TODO uncomment this
    // pros::Task debug(checkColorGaps);
    robot.optical->set_led_pwm(100); // turn on optical sensor led for driver control
    pros::Task cross(dummy_func);
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        intake.runIntake();
        unloader.toggleFire();
        descore.toggleFire();
        score_toggle.toggleFire();
        // // delay to save resources
        intake.color_sort();
        pros::delay(10);
    }
}
