#include "main.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "lemlib/timer.hpp"
#include "pushback/api.hpp"
#include "pushback/Wall_Sen.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain motors
pros::MotorGroup leftMotors({-11, -12, 13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-18, 19, 20},
                             pros::MotorGearset::blue); // right motor group - ports 6 (reversed), 7, 9

// These are all the intake motors
pros::Motor intake_1(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); // first stage
pros::Motor intake_2(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); // mid stage
pros::Motor intake_3(121, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees); // high stage
pros::Optical optical(120);
pros::Imu imu(2);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(100);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-14);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.0, 0);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2.0, -0.4);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              9.8, // 10 inch track width
                              3.25, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(7.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            22, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            100 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angularController(5.8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             44, // derivative gain (kD)
                                             0, // anti windup
                                             5, // small error range, in degrees
                                             260, // small error range timeout, in milliseconds
                                             10, // large error range, in degrees
                                             550, // large error range timeout, in milliseconds
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

// Pistons
//1 is middle goal intake
//2 is hood
pushback::Piston unloader(4, pros::E_CONTROLLER_DIGITAL_DOWN);
pushback::Piston score_toggle(2, pros::E_CONTROLLER_DIGITAL_B); //midle goal
pushback::Piston descore(3, pros::E_CONTROLLER_DIGITAL_Y);
pushback::Piston middle_goal(1, pros::E_CONTROLLER_DIGITAL_A); 

//distance sensors
std::vector<pushback::Wall_Sen> distance_sensors = {
    pushback::Wall_Sen(1,1,1, pushback::Wall_Sen::FRONT),
    pushback::Wall_Sen(2,1,1, pushback::Wall_Sen::BACK),
    pushback::Wall_Sen(3,1,1, pushback::Wall_Sen::LEFT),
    pushback::Wall_Sen(4,1,1, pushback::Wall_Sen::RIGHT)
};
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
pushback::Robot robot(chassis, &intake_1, &intake_2, &intake_3, &descore, &unloader, &descore, &middle_goal,
                      distance_sensors, &imu, controller, &optical, 0);

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
    middle_goal.register_controller(&robot);
    score_toggle.register_controller(&robot);

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
    while (true) {
        const auto pos = chassis.getPose();
        intake.color_sort();

        char buf1[48], buf2[48], buf3[48], buf7[48];
        std::snprintf(buf1, sizeof(buf1), "X: %.3f", pos.x);
        std::snprintf(buf2, sizeof(buf2), "Y: %.3f", pos.y);
        std::snprintf(buf3, sizeof(buf3), "Theta: %.3f", pos.theta);

        pros::screen::print(pros::E_TEXT_MEDIUM, 1, buf1);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, buf2);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, buf3);


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

//LT = left motor temps
//RT = right motor temps
void logData() {
    //mark the start of data stream
    printf("Data Start;\n");
    std::vector<double> leftTemps = leftMotors.get_temperature_all();
    std::vector<double> rightTemps = rightMotors.get_temperature_all();
    //print motor temps
    printf("LT; %.2f,%.2f,%.2f\n", leftTemps[0], leftTemps[1], leftTemps[2]);
    printf("RT; %.2f,%.2f,%.2f\n", rightTemps[0], rightTemps[1], rightTemps[2]);
    printf("IT1; %.2f\n", intake_1.get_temperature());
    printf("IT2; %.2f\n", intake_2.get_temperature());
    while (true) {
        lemlib::Pose pos = chassis.getPose();

        printf("Pos; %.2f,%.2f,%.2f\n", pos.x, pos.y, pos.theta);
        pros::delay(50);
    }
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
    intake.mid_goal_strong(); // score mid goal
        unloader.firePiston(true);
    moveStraight(2, 200, {}, false);
    wait(600); // give time to score
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
    //chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_right_side() + 0.1, chassis.getPose().theta); //
    chassis.turnToHeading(0, 600); unloader.firePiston(false); // Matchloader up 
    moveStraight(7, 440, {},
    true); // move so dont hit the goals 
    chassis.swingToHeading(90, DriveSide::RIGHT, 650); // turn so can goforwards
    real_brake(); 
    chassis.turnToHeading(88, 230); // make sure we not off 
    intake.stop();
    chassis.moveToPoint(50, 57, 2200, {}, true); // go to other side of field
    intake.outake();
    wait(100);
    intake.intake();
    error = 25 - chassis.getPose().x;
    while (error > 0.5) { // once past x=30 slow down basically
        error = 30 - chassis.getPose().x;
        wait(20);
    }
    chassis.cancelMotion();
    chassis.moveToPoint(50, 57, 800, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 1},
                        false); // go to other side of field
    chassis.turnToHeading(90, 250, {}, false);
    //chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_left_side() - 0.5, chassis.getPose().theta); //
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
    //chassis.setPose(70 - robot.get_distance_left_side(), -17, 180); // used to be -17.5
    //descore.firePiston(true);
    //   chassis.setPose(-28, -46.8,
    //                  270); //if robot isn't within 3.5 degrees(aka its not straight on) don't reset angle
    //  unloader.firePiston(true);
    //  score_toggle.firePiston(true);
    //wait(1000);

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
    chassis.turnToHeading(180, 500, {}, false);
    robot.ram(-60, 1700);
    wait(200); // wait for robot to settle
    //chassis.setPose(70 - robot.get_distance_left_side(), -17.0f, chassis.getPose().theta); // used to be -17.5
    wait(20); // let register

    // #3
    chassis.moveToPoint(61, -28, 700, {.maxSpeed = 60, .minSpeed = 1, .earlyExitRange = 1}, false);
    chassis.turnToPoint(23.4, -18.59, 700, {.maxSpeed = 60}, false);
    intake.intake();
    chassis.moveToPoint(22.7, -18.67, 900, {.maxSpeed = 90}, false);
    intake.stop();
    chassis.moveToPoint(9.89, -8.89, 1300, {.forwards = false, .maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 2},
                        true);
    intake.outake(); // get them unstuck
    intake_1.move_voltage(0); // dont move the first stage as to not loose balls
    wait(500); // give the balls time to get unstuck
    intake.stop();
    chassis.waitUntilDone();
    moveStraight(1.95, 200, {}, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // hold so robot doesn't drift
    intake.mid_goal(); // score mid goal
    wait(800); // give time to score
    moveStraight(1.6, 240, {}, false); //get ot of goal
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
    //TODO chassis.setPose(chassis.getPose().x, robot.get_distance_right_side() + 0.1 - 70, chassis.getPose().theta); // reset
    wait(15); // let reset register
    chassis.turnToHeading(180, 600, {}, false);
    moveStraight(7.5, 450, {},
                 true); // move so dont hit the goals wait(100); unloader.firePiston(false); // Matchloader up
    chassis.waitUntilDone();
    intake.intake();
    chassis.moveToPoint(-50, -57.6, 2200, {}, true); // go to other side of field
    intake.outake();
    wait(100);
    intake.intake();
    error = chassis.getPose().x + 30;
    lemlib::Timer reset_point(800); //0.4 second in reset position
    bool reset = false;
    while (error > 0.5) { // once past x=-30 slow down basically
        error = chassis.getPose().x + 30;
        if(reset_point.isDone() && !reset){
            //TODO chassis.setPose(chassis.getPose().x, robot.get_distance_left_side() - 70, chassis.getPose().theta);
            reset= true;
        }
        wait(20);
    }
    chassis.cancelMotion();
    chassis.moveToPoint(-50, -57.6, 950, {.maxSpeed = 40},
                        false); // go to other side of field
    rb(40);
    chassis.turnToHeading(270, 300, {}, false);
    //TODOchassis.setPose(chassis.getPose().x, robot.get_distance_left_side() + 0.5 - 70,
     //TODO               chassis.getPose().theta);                                                  // goal
    chassis.moveToPose(-24, -46.9, -90, 1200, {.forwards = false, .lead = 0.4, .maxSpeed = 95},
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


void awp() {
    // Starting Position
    //TODO lemlib::Pose start_pose(robot.get_distance_right_side() - 70, -20, 180);

    // #1
    //TODO chassis.setPose(start_pose);
    chassis.moveToPoint(-48.24, -43.23, 620, {.maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 0.3}, true); //
    wait(100);
    unloader.firePiston(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 650, {.minSpeed = 1, .earlyExitRange = 1}, false);
    intake.intake();
        lemlib::Pose pose = chassis.getPose();
        chassis.moveToPoint(pose.x + 20 * sin(lemlib::degToRad(pose.theta)),
                        pose.y + 20 * cos(lemlib::degToRad(pose.theta)), 850, {.maxSpeed=70, .minSpeed=69}, false); //add 2 for drift in y direction
    moveStraight(-9, 300, {.minSpeed = 1, .earlyExitRange = 0.5}, true);
    wait(190);
     //TODO chassis.setPose(chassis.getPose().x, robot.get_distance_left_side()-70, chassis.getPose().theta); // reset again
     chassis.waitUntilDone();
    chassis.moveToPose(-30.535, -46.8, 270, 900, {.forwards = false, .lead = 0.2, .minSpeed = 55, .earlyExitRange = 1},
                       true); // go back into goal
    chassis.waitUntilDone();
    score_toggle.firePiston(true); // open up scoring hood
    robot.ram(-105, 300); // get into goal
    intake.intake(); // start scoring
    robot.ram(-85, 950); //score
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
    chassis.swingToHeading(20, DriveSide::RIGHT, 600, {.minSpeed = 1, .earlyExitRange = 2}, false);
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
    //TODO chassis.setPose(chassis.getPose().x, 70 - robot.get_distance_right_side(), chassis.getPose().theta);
    chassis.moveToPose(-27.5, 47, 270, 950, {.forwards = false, .lead = 0.25, .minSpeed = 1, .earlyExitRange = 0.5},
                       true);
    chassis.waitUntilDone();
    score_toggle.firePiston(true); // open up scoring hood before ramming in so that the robot alligns better
    robot.ram(-105, 200);

    intake.intake(); // start scoring
    robot.ram(-85, 1030); //score like 4-5 balls
    angError = fabs(chassis.getPose().theta - 270);
    if (angError < 3.5) { chassis.setPose(-28, 46.8, chassis.getPose().theta); } // reset position only if alligned
    intake.outake();
    intake_1.move_voltage(0); // dont let balls out
    chassis.moveToPose(-56.2, 45.7, 270, 950, {.forwards = true, .lead = 0.3, .minSpeed = 10, .earlyExitRange = 1.5},
                       true); //go to matchloader
    wait(100);
    intake.stop();
    score_toggle.firePiston(false);
    intake.intake();
    chassis.waitUntilDone();
    robot.ram(60, 400); // get balls
    robot.ram(60,440); //go a bit slower
    moveStraight(-6, 200, {.forwards = false, .minSpeed = 1, .earlyExitRange = 0.5}, false);
    chassis.moveToPose(-11.6, 11, 315, 1400, {.forwards=false, .minSpeed=1, .earlyExitRange=0.5}, true); //head over midgoal
    wait(800); //wait until a little closer
    intake.outake();
    intake_1.move_voltage(0);
    wait(200);
    intake.stop();
    chassis.cancelMotion();
    chassis.moveToPoint(-14.1, 13.5, 900, {.forwards=false, .minSpeed=1, .earlyExitRange=1.3}, true); //switch over to moveToPoint to speed up the motion
    wait(550);
    intake.mid_goal();
    intake_3.move_voltage(-3500);
    chassis.waitUntilDone();
    intake.mid_goal(); // score mid goal
    intake_3.move_voltage(-4500);
    robot.ram(-80, 1000);
}


void driveOneINch() { moveStraight(5, 250, {}, false); }




/**
 * Runs during auto
 *
 * This is an example
 *  routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    pros::Task log(logData); // log data
    

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
    pros::Task log(logData); // log data
    // pros::Task debug(checkColorGaps);
    robot.optical->set_led_pwm(100); // turn on optical sensor led for driver control
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        intake.runIntake();
        unloader.toggleFire();
        descore.toggleFire();
        middle_goal.toggleFire();
        score_toggle.toggleFire();
        // // delay to save resources
        intake.color_sort();
        pros::delay(10);
    }
}
