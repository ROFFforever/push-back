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
pros::Optical optical(5);
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
                                            400, // large error range timeout, in milliseconds
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
pushback::Wall_Sen* front = new pushback::Wall_Sen(6, -4.375, 4.1, pushback::Wall_Sen::FRONT);
pushback::Wall_Sen* back = new pushback::Wall_Sen(7,2.5,2.9, pushback::Wall_Sen::BACK);
pushback::Wall_Sen* left = new pushback::Wall_Sen(3,0,4.7, pushback::Wall_Sen::LEFT);
pushback::Wall_Sen* right = new pushback::Wall_Sen(8,-3,5, pushback::Wall_Sen::RIGHT);
pushback::Wall_Sen* back2 = new pushback::Wall_Sen(4, -5.5, 3.7, pushback::Wall_Sen::FRONT);

std::vector<pushback::Wall_Sen> distance_sensors = {
   *front,
   *back,
   *left,
   *right
};
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
pushback::Robot robot(chassis, &intake_1, &intake_2, &intake_3, &descore, &unloader, &descore, &middle_goal,
                      distance_sensors, &imu, controller, &optical, pushback::Robot::BLUE);

// Intake mechanism
pushback::Intake intake(robot);


void reset_x(){
    if(robot.reset_x()){ //check if we actually reset
        printf("X reset success\n");
    }else{
        printf("Failed to reset\n");
    }
}
void reset_y(){
    if(robot.reset_y()){ //check if we actually reset
        printf("Y reset success\n");
    }else{
         printf("Failed to reset\n");
    }
}
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
void clear_intake(){
    intake.outake();
    wait(2000);
    intake.stop();
}

void af(){
    clear_intake();
    wait(1000000);
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
ASSET(skills_other_side_txt);
ASSET(skills_other_side_2_txt);
ASSET(rightFourFive_txt);

// debug task to print position to screen
void print_pos() {
    float distance = robot.get_dist_from_wall(&distance_sensors[3]); //just test right for now
    while (true) {
        const auto pos = chassis.getPose();
        distance = robot.get_dist_from_wall(&distance_sensors[3]); //just test right for now
        char buf1[48], buf2[48], buf3[48], buf7[48];
        std::snprintf(buf1, sizeof(buf1), "X: %.3f", pos.x);
        std::snprintf(buf2, sizeof(buf2), "Y: %.3f", pos.y);
        std::snprintf(buf3, sizeof(buf3), "Theta: %.3f", pos.theta);
        std::snprintf(buf7, sizeof(buf7), "Center to wall: %.3f", distance);


        pros::screen::print(pros::E_TEXT_MEDIUM, 1, buf1);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, buf2);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, buf3);
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, buf7);


        pros::delay(50);
    }
}

void waitUntilScore(int maxTime){
    optical.set_led_pwm(100);
    lemlib::Timer timeMax(maxTime);
    while((!robot.detectOpposingColor()) && (!timeMax.isDone())){ //until we see the wrong color just keep waiting/scoring
        wait(30);
    }
    //dont burn it out
    optical.set_led_pwm(0); 
}

void scoreAllRed(int maxTime, int color){

    //start scoring middle
    middle_goal.firePiston(false);
    intake.intake_weak();

    lemlib::Timer timeMax(maxTime);
    bool detected = robot.detectedColor(color);
    while((!detected) && (!timeMax.isDone())){ //until we see the wrong color just keep waiting/scoring
        detected = robot.detectedColor(color);
        wait(20);
    }

    if(detected){

    //spit out that one bad ball
    intake.stop();
    score_toggle.firePiston(true); //get this out of the way first
    middle_goal.firePiston(true);
    wait(200); //activation time
    

    //wait until that one bad block leaves intake
    int timeForBlockToFallOut = 50;
    intake.intake();
    while(!robot.detectedColor(robot.oppositeColor(color))) wait(15); 
    wait(timeForBlockToFallOut);

    //keep all the blocks and lower back down
    intake.outake();
    wait(150);
    intake.stop();
    middle_goal.firePiston(false);
    score_toggle.firePiston(false); //lower hood back down to orig position
    
    //dont burn it out
    optical.set_led_pwm(0); 

    //wait until timer runs out
    intake.intake_weak();
    while((!timeMax.isDone())){ //until we see the wrong color just keep waiting/scoring
        wait(30);
    }
    middle_goal.firePiston(false);
    intake.intake_weak();
    }
    middle_goal.firePiston(true);
    intake.intake_weak();
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
    //float distance = robot.get_dist_from_wall(&distance_sensors[3]); //just test right for now

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
        // distance = robot.get_dist_from_wall(&distance_sensors[3]); //just test right for now    
        printf("Pos; %.2f,%.2f,%.2f\n", pos.x, pos.y, pos.theta);
        // printf("ACCEL Z: %.2f\n", imu.get_accel().z);
        pros::delay(50);
    }
}


float brakeSmart() {
    lemlib::Pose speed = lemlib::getLocalSpeed(true);
    float current_speed = sqrt(speed.x * speed.x + speed.y * speed.y);
    return current_speed;
}
void brake(){
       //brake motors
        leftMotors.move_voltage(-1500);
    rightMotors.move_voltage(-1500);
    wait(150);
leftMotors.move_voltage(-100);
    rightMotors.move_voltage(-100);
}

void old_skills() {
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

void waitUntilOnGround(){
    int times_0 = 0;
    while(times_0 < 12){
        if(imu.get_accel().z > 0){
            times_0++;
        }
        wait(7);
    }
    printf("On ground\n");
}
void pickupballs(){
    chassis.setPose(52,0,90);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    intake.intake();
    robot.ram(106,400);
    robot.ram(90,110);
    leftMotors.move_voltage(2800);
    rightMotors.move_voltage(2800);
    wait(250);
    rightMotors.move_voltage(-1100);
    leftMotors.move_voltage(-1100);
    wait(800);
    leftMotors.move_voltage(5300);
    rightMotors.move_voltage(5300);
    wait(650);
    rightMotors.move_voltage(-900);
    leftMotors.move_voltage(-900);
    wait(700);
    robot.ram(-90, 1150);
    robot.ram(70, 900);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}
void skillspt2(){
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    robot.reset_x(false, false, true);
    chassis.setPose(chassis.getPose().x, 70-robot.get_dist_from_wall(left), 90);
    moveStraight(-18, 1000, {}, false); //go past parking lot a bit
    chassis.turnToHeading(0, 600, {}, false);
    intake.intake();
    chassis.moveToPoint(25, 22, 800, {}, true);
    wait(200);
    reset_y();
    reset_x();
    chassis.waitUntilDone();
    chassis.turnToPoint(8, 8, 700, {.forwards=true}, false);
    chassis.moveToPoint(9, 9, 800, {.forwards=true}, true);
    wait(500);
    intake.outake();
    wait(100);
}
void skillsoutake(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
     intake.outake();
            wait(150);
    middle_goal.firePiston(false); //lift the lower intake
    intake.outake_weak();
    wait(1000);
    intake.outake_super_weak();
    wait(1800);
    intake.stop();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}
void skillspt3(){
    chassis.moveToPoint(16, 16, 500, {.forwards=false}, false);
     middle_goal.firePiston(true);
    chassis.swingToPoint(22, -19.5, DriveSide::RIGHT, 700, {.minSpeed=1, .earlyExitRange=1}, false);
    reset_x();
    intake.intake();
    chassis.moveToPoint(22, -19, 1000, {.minSpeed=1, .earlyExitRange=1.2}, true);//go towards block
    reset_y();
    wait(700);
    descore.firePiston(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(46, -47.6, 1200, {}, true);
    reset_x();
    wait(200);
    reset_y();
    chassis.waitUntilDone();
    unloader.firePiston(true);
    chassis.turnToHeading(90, 500, {});
    reset_y();
    chassis.waitUntilDone();
    intake.intake();
    chassis.moveToPoint(70, -46.8, 1500, {.maxSpeed=65},true);
    lemlib::Timer maxT(1000);
    while((chassis.getPose().x < 52.7) && (!maxT.isDone())) wait(15);
    chassis.cancelMotion();
    robot.ram(45, 1650); //get matchloaders
    descore.firePiston(true);
}
void skillspt4(){
    //go to other side of the field
    chassis.follow(skills_other_side_txt, 18, 1200, false, false);
    printf("Done with path\n");
    reset_y();
    reset_x();
    unloader.firePiston(false);
    intake.outake();
    chassis.moveToPoint(-24, -57.5, 1400, {.forwards=false, .minSpeed=10, .earlyExitRange=1.5}, true);
    wait(150);
    intake.stop();
    wait(150);
    reset_x();
    wait(600);
    reset_y();
    chassis.waitUntilDone();
   chassis.moveToPoint(-40, -48, 650, {.forwards=false}, false);
    chassis.turnToHeading(-90, 600, {}, false);
    reset_y();
    descore.firePiston(false);
    chassis.moveToPoint(-27, -46.8, 1000, {.forwards=false, .minSpeed=1, .earlyExitRange=1}, false);
    robot.ram(-115, 300); //push into goal
    score_toggle.firePiston(true); // open up scoring hood
    intake.intake();
    wait(1500);

}
void skillspt5(){
    score_toggle.firePiston(false);
    intake.intake();
    reset_y();
    chassis.moveToPoint(-50, -46.8, 800, {.minSpeed=1, .earlyExitRange=1}, true);
    unloader.firePiston(true);
    wait(200);
    reset_y();
    chassis.waitUntilDone();
    chassis.moveToPose(-70, -46.3, 270, 1800, {.lead=0.28}, true); //go to matchloader
    lemlib::Timer maxx(900);
    while((chassis.getPose().x > -52.7) && (!maxx.isDone())) wait(15);
    chassis.cancelMotion();
    robot.ram(45, 1500); //get matchloaders
    chassis.moveToPoint(-27, -46.8, 700, {.forwards=false, .minSpeed=1, .earlyExitRange=1},
                       false); // go back into goal

    score_toggle.firePiston(true);
    robot.ram(-115, 200); //get into goal
        intake.intake();
        unloader.firePiston(false);
        robot.ram(-80, 1800);
        intake.stop();
}
void skillspt6(){
    reset_y();
    reset_x();
    intake.intake();
    leftMotors.move_voltage(7500);
    rightMotors.move_voltage(-3500);
    float theta = chassis.getPose().theta;
    lemlib::Timer max(1000);
    while((theta < -20) && (!max.isDone())){
    theta = chassis.getPose().theta;
    wait(10);
    }
    printf("DONE TURNING \n");
    score_toggle.firePiston(false);
    leftMotors.move_voltage(-1800);
    rightMotors.move_voltage(1800);
    chassis.moveToPoint(-38.5, -20, 900, {.maxSpeed=80}, true);
    wait(100);
    reset_x();
    chassis.setPose(chassis.getPose().x, robot.get_dist_from_wall(back2) - 70, chassis.getPose().theta);
    while(chassis.getPose().y < -30){
        wait(15);
    }
    chassis.cancelMotion();
    reset_x();
    chassis.moveToPoint(-44.5, 0.5, 1200, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.move_voltage(6000);
    leftMotors.move_voltage(-4000);
    intake.intake();
    while(chassis.getPose().theta > -80){
        wait(20);
    }
    rightMotors.move_voltage(-900);
    leftMotors.move_voltage(900);
    chassis.turnToHeading(270, 400, {}, false);
}
void pickupballs2(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    robot.reset_y(false, true, true);
    intake.intake();
    robot.ram(112,400);
    robot.ram(90,110);
    leftMotors.move_voltage(2800);
    rightMotors.move_voltage(2800);
    wait(250);
    rightMotors.move_voltage(-1100);
    leftMotors.move_voltage(-1100);
    wait(800);
    leftMotors.move_voltage(5300);
    rightMotors.move_voltage(5300);
    wait(700);
    rightMotors.move_voltage(100);
    leftMotors.move_voltage(100);
    wait(300);
    robot.ram(-80, 400);
    robot.ram(70, 400);
    robot.ram(-90, 1150);
    robot.ram(70, 900);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}
void skillspt7(){
    wait(100); //wait for to settle
    intake.intake();
    robot.reset_x(false, true, false);
    chassis.setPose(chassis.getPose().x, robot.get_dist_from_wall(left)-70, chassis.getPose().theta);
    chassis.moveToPoint(-15.1,9.1,1000, {.forwards=false}, false);
    rightMotors.move_voltage(-9000);
    leftMotors.move_voltage(1000);
    float theta = chassis.getPose().theta;
    lemlib::Timer maxTime(1000);
    while((theta < -45) && (!maxTime.isDone())){
    theta = chassis.getPose().theta;
    wait(10);
    }
    rightMotors.move_voltage(0);
    leftMotors.move_voltage(0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    score_toggle.firePiston(true); //helps mid goal scoring
    scoreAllRed(2500, pushback::Robot::BLUE);
    middle_goal.firePiston(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    intake.intake();
    chassis.moveToPoint(-38.6, 43.5, 1200, {.minSpeed=1, .earlyExitRange=1}, true); //go to matchloader
    wait(200);
    score_toggle.firePiston(false); //close scoring hood again
    wait(100);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 600, {}, false);
    reset_y();
    chassis.moveToPose(-70, 46, 270, 2500, {.lead=0.28, .maxSpeed=70}, true);
    lemlib::Timer maxT(1200);
    unloader.firePiston(true);
    while((chassis.getPose().x > -52.7) && (!maxT.isDone())) wait(15);
    chassis.cancelMotion();
    robot.ram(45, 1650); //get matchloaders

}
void skillspt8(){
    chassis.follow(skills_other_side_2_txt, 18, 1300, false, false);
    printf("Done with path\n");
    reset_y();
    reset_x();
    intake.outake();
        unloader.firePiston(false); //give time to funnel balls
    chassis.moveToPoint(24, 57.5, 1800, {.forwards=false, .minSpeed=1, .earlyExitRange=1}, true); //go to other side of the field
    wait(150);
    intake.stop();
    wait(150);
      reset_x();
      wait(600);
    reset_y();
    chassis.waitUntilDone();
    descore.firePiston(false);
    reset_y();
    chassis.moveToPoint(39, 45, 650, {.forwards=false}, false);
    chassis.turnToHeading(90, 600, {}, false);
    descore.firePiston(false);
    chassis.moveToPoint(24, 46.3, 600, {.forwards=false, .minSpeed=1, .earlyExitRange=1}, false); //go into goal
    robot.ram(-115, 300); //push into goal
    middle_goal.firePiston(true);
    score_toggle.firePiston(true); // open up scoring hood
    intake.intake();
    wait(1500); //score on long goal
}
void skillspt9(){
    intake.intake();
    score_toggle.firePiston(false);
    chassis.setPose(25, chassis.getPose().y, chassis.getPose().theta);
    reset_y();
    chassis.moveToPose(70, 46.2, 90, 1800, {.lead=0.28}, true);
    unloader.firePiston(true);
    wait(50);
    reset_y();
    lemlib::Timer maxT(900);
    while((chassis.getPose().x < 52.7) && (!maxT.isDone())){ 
        wait(15);
        if(maxT.getTimePassed() > 200){
            reset_y();
        }
    }
    chassis.cancelMotion();
    robot.ram(45, 1500); //get matchloaders
    chassis.moveToPoint(27, 46.8, 700, {.forwards=false, .minSpeed=1, .earlyExitRange=1},
                       false); // go back into goal
    chassis.waitUntilDone();
    score_toggle.firePiston(true);
    robot.ram(-115, 200); //get into goal
        intake.intake();
        wait(1500);
        intake.stop();
}
void park(){
    reset_x();
    reset_y();
    intake.intake();
    unloader.firePiston(false);
    chassis.moveToPoint(60, 22, 1000, {}, false);
    intake.outake();
    chassis.swingToHeading(170, DriveSide::RIGHT, 450, {}, false);
    robot.ram(110, 1200); //finish parking
}
void skills_119(){
    // Starting Position



    //chassis.setPose(70-robot.get_dist_from_wall(front), 70-robot.get_dist_from_wall(left), 90);
    // score_toggle.firePiston(true);
    //      wait(500);
    middle_goal.firePiston(true);
    pickupballs(); //gets 6 balls in parking zone
    skillspt2(); //get to low goal
    skillsoutake(); //score 7 on low goal
    skillspt3(); //get first matchloaders and get 3 extra balls
    skillspt4(); // cross field and score in long goal
    skillspt5(); //go to second matchloader and go back to goal and score
    skillspt6(); //get one red block and line up with parking zone
    pickupballs2(); //get blocks inside parking zone for middle goal
    skillspt7(); //score mid goal and go to matchloader
    skillspt8(); //go to other side of field and score in long goal
    skillspt9(); // go to matchloader and score in goal
    park(); // park in parking zone

    //ROUTE TOTAL: 119 points(maximum possible)

    // //TODO swing faster, dont worry about drift movetopoint will autocorrect
    // wait(3000);
    // af();

    


}

void awp() {
    // Starting Position
    lemlib::Pose start_pose(robot.get_dist_from_wall(left) - 70, robot.get_dist_from_wall(back) - 70, 0);
    robot.color = pushback::Robot::RED;

    // #1
    lemlib::Timer timer(20000);
    chassis.setPose(start_pose);
    middle_goal.firePiston(true);
    intake.intake();
    robot.ram(95, 400); //move partner and get ball
    chassis.moveToPoint(-48, -43, 1300, {.forwards=false}, true);
    wait(500);
    unloader.firePiston(true);
    reset_x();
    chassis.waitUntilDone();
      printf("finished motion\n");
    chassis.turnToHeading(270, 500, {}, false);
    reset_y();
    robot.ram(75, 950); //get matchloaders
    reset_y();
    chassis.moveToPose(-30, -46.8, 270, 1200, {.forwards = false, .lead = 0.2, .minSpeed = 80, .earlyExitRange = 1},
                       true); // go back into goal
                       wait(300);
                        reset_y();
                       wait(430);
                       intake.outake();
                       score_toggle.firePiston(true); //open up for scoring
                       wait(60);
                       intake.stop();
                       chassis.waitUntilDone();
    leftMotors.move_voltage(-10000);
     rightMotors.move_voltage(-10000);
     unloader.firePiston(false); // Matchloader up
    intake.intake();
    robot.ram(-80, 900);
    reset_y();

    // #2
    leftMotors.move_voltage(7500);
    rightMotors.move_voltage(-3500);
    float theta = chassis.getPose().theta;
    lemlib::Timer maxTime(1000);
    while((theta < 10) && (!maxTime.isDone())){
    theta = chassis.getPose().theta;
    wait(10);
    }
    score_toggle.firePiston(false); // close scoring hood
    printf("DONE TURNING \n");
    reset_x();
    reset_y();
        leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    chassis.moveToPoint(-24, 18, 1200, {.maxSpeed = 100},
                        true); // go towards balls
    wait(270);
    unloader.firePiston(true);
    wait(380);
    unloader.firePiston(false);
    wait(450);
    descore.firePiston(true); // avoid hititng long goal
    unloader.firePiston(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-39, 46.6, 1300, {.minSpeed = 20, .earlyExitRange = 0.5}, false); // go near long goal
    chassis.turnToHeading(270, 450, {}, true); // align with long goal
    descore.firePiston(false);
    chassis.waitUntilDone();
    reset_y();
    chassis.moveToPose(-27, 47, 270, 950, {.forwards = false, .lead = 0.25, .minSpeed = 60, .earlyExitRange = 0.5},
                       true);
                       wait(300);
                       intake.outake();
                       score_toggle.firePiston(true); // open up scoring hood before ramming in so that the robot alligns better
                       wait(120);
                       intake.stop();
    chassis.waitUntilDone();
      leftMotors.move_voltage(-11000);
     rightMotors.move_voltage(-11000);
    intake.intake();
    wait(890);
    leftMotors.move_voltage(0);
     rightMotors.move_voltage(0);
     chassis.moveToPose(-70, 46.7, 270, 1200, {.minSpeed=50, .earlyExitRange=0.4}, true); //go to matchloader
        wait(200);
     intake.outake();
     wait(100);
     intake.stop();
        score_toggle.firePiston(false); // close scoring hood
      lemlib::Timer maxT(1000);
           intake.intake();
    while((chassis.getPose().x > -52.7) && (!maxT.isDone())) wait(15);
    chassis.cancelMotion();
    robot.ram(45, 1150); //get matchloaders
        
    moveStraight(-8, 600, {.minSpeed=1, .earlyExitRange=1}, false);
    chassis.turnToHeading(285, 300, {}, false);
    chassis.moveToPoint(-13,13, 1500, {.forwards=false, .minSpeed=60, .earlyExitRange=1}, true); 
    reset_x();
    reset_y();
    wait(300);
    unloader.firePiston(false);
    chassis.waitUntilDone();
    middle_goal.firePiston(false);
    intake.weak_kinda();
    waitUntilScore(1300);
    intake.outake();
    middle_goal.firePiston(true);
    printf("Done with Auton: %.f \n", (float) timer.getTimePassed());
   
}
void firstTrioArcRight(){
    chassis.setPose(-47, -13.5, 90);
    robot.reset_x(false, false, true);
    robot.reset_y(false, false, true);
    intake.intake();
    leftMotors.move_voltage(11000);
    rightMotors.move_voltage(6500);
    wait(430);
    unloader.firePiston(true);

}

void firstTrioArcLeft(){
    chassis.setPose(-47, -13.5, 90);
    robot.reset_x(false, false, true);
    robot.reset_y(false, false, true);
    intake.intake();
    leftMotors.move_voltage(6500);
    rightMotors.move_voltage(11000);
    wait(430);
    unloader.firePiston(true);

}
void firstTrioArcLeftM(){
    chassis.setPose(-47, 13.5, 90);
    robot.reset_x(false, false, true);
    robot.reset_y(false, false, true);
    intake.intake();
    leftMotors.move_voltage(4700);
    rightMotors.move_voltage(11000);
    wait(430);
    unloader.firePiston(true);
    descore.firePiston(true);
}
void firstTrioArcRightM(){
    chassis.setPose(-47, -13.5, 90);
    robot.reset_x(false, false, true);
    robot.reset_y(false, false, true);
    intake.intake();
    leftMotors.move_voltage(11000);
    rightMotors.move_voltage(4700);
    wait(430);
    unloader.firePiston(true);
    

}

void getMatchLoadsSevenBallRight(){
        leftMotors.move_voltage(11000);
    rightMotors.move_voltage(-2000);
    while(chassis.getPose().theta < 185){
        wait(15);
    }
    int wait_time = 555;
     leftMotors.move_voltage(7500);
    rightMotors.move_voltage(11500);
    while(chassis.getPose().y>-33) wait(15);
    chassis.swingToHeading(270, DriveSide::RIGHT, 500, {.maxSpeed=110, .minSpeed=1, .earlyExitRange=2}, false);
    





    chassis.moveToPose(-70, -46.8, 270, 1400, {.lead=0.28, .maxSpeed=80, .minSpeed=50, .earlyExitRange=1}, true);
    lemlib::Timer maxT(1000);
           intake.intake();
    while((chassis.getPose().x > -52.7) && (!maxT.isDone())) wait(15);
    chassis.cancelMotion();
    robot.ram(50, 600); //get matchloaders
}
void threePlusFourGoIntoGoal(){
        leftMotors.move_voltage(11000);
    rightMotors.move_voltage(-2000);
    while(chassis.getPose().theta < 185){
        wait(15);
    }
    int wait_time = 520;
     leftMotors.move_voltage(6500);
    rightMotors.move_voltage(11500);
    wait(wait_time);

    chassis.swingToHeading(273, DriveSide::RIGHT, 500, {.maxSpeed=110, .minSpeed=1, .earlyExitRange=2}, false);
    while(chassis.getPose().theta < 270) wait(15);
    reset_y();
    reset_x();


    //brake motors
        leftMotors.move_voltage(-1500);
    rightMotors.move_voltage(1500);
    wait(150);
leftMotors.move_voltage(-100);
    rightMotors.move_voltage(-100);

    chassis.moveToPose(-23, -46.8, 270, 1300, {.forwards=false, .lead=0.3}, false);
    score_toggle.firePiston(true);
    intake.intake();
    wait(1300);

}
void getMatchLoadsSevenBallLeft(){
           leftMotors.move_voltage(-2000);
    rightMotors.move_voltage(11000);
    while(chassis.getPose().theta < -5){
        wait(15);
    }
    int wait_time = 555;
     leftMotors.move_voltage(11500);
    rightMotors.move_voltage(7500);
    wait(wait_time);

    chassis.swingToHeading(267, DriveSide::RIGHT, 500, {.maxSpeed=110, .minSpeed=1, .earlyExitRange=2}, false);
    while(chassis.getPose().theta < 270) wait(15);
    reset_y();
    reset_x();


    //brake motors
        leftMotors.move_voltage(-1500);
    rightMotors.move_voltage(-1500);
    wait(150);
leftMotors.move_voltage(-100);
    rightMotors.move_voltage(-100);



    chassis.moveToPose(-70, 46.8, 270, 1400, {.lead=0.28, .maxSpeed=80, .minSpeed=50, .earlyExitRange=1}, true);
    lemlib::Timer count(150000);
    while((chassis.getPose().x > -48)){
        if(count.getTimePassed() > 150) reset_y();
        wait(10);
    }

    chassis.cancelMotion();
    printf("CANCELLED MOVETOPOSE\n");
    robot.ram(55, 400);
    robot.ram(70, 350);
}
void scoreBallsSevenBallRight(){
    chassis.moveToPose(-32, -47, 270, 1500, {.forwards=false, .lead=0.3, .minSpeed=10, .earlyExitRange=1}, true);
    reset_y();
    chassis.waitUntilDone();

    robot.ram(-115, 150); //allign with long goal
    score_toggle.firePiston(true);
    unloader.firePiston(false);
    robot.ram(-70, 900); //give time score balls
}
void scoreBallsSevenBallLeft(){
    chassis.moveToPose(-32, 47, 270, 1500, {.forwards=false, .lead=0.3, .minSpeed=10, .earlyExitRange=1}, true);
    reset_y();
    chassis.waitUntilDone();

    robot.ram(-115, 150); //allign with long goal
    score_toggle.firePiston(true);
    middle_goal.firePiston(true);
    unloader.firePiston(false);
    robot.ram(-70, 900); //give time score balls
}
void sevenBallSwingRight(){
    //swing out of goal and push
    leftMotors.move_voltage(-3500);
    rightMotors.move_voltage(8000);
    lemlib::Timer maxTime(1000);
    while((chassis.getPose().theta > 220) && (!maxTime.isDone())){
    wait(10);
    }
    score_toggle.firePiston(false); // close scoring hood
    printf("DONE TURNING \n");
    rightMotors.move_voltage(0);
    leftMotors.move_voltage(0);
    chassis.turnToHeading(270, 600, {.minSpeed=10, .earlyExitRange=1}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    lemlib::Timer waitForReset(100000);
    while(chassis.getPose().x < -16.3){
        leftMotors.move_voltage(-6000);
        rightMotors.move_voltage(-6000);
        if(waitForReset.getTimePassed() > 300){ reset_x(); waitForReset.reset(); };
        wait(15);
    }
    reset_x();
    
}
void sevenBallBlueRight(){
    middle_goal.firePiston(true);
    firstTrioArcRight();
    getMatchLoadsSevenBallRight();
    scoreBallsSevenBallRight();
    sevenBallSwingRight();
}

void sevenBallBlueLeft(){
    firstTrioArcLeft();
    getMatchLoadsSevenBallLeft();
    scoreBallsSevenBallLeft();
    sevenBallSwingRight();
}

void grabSecondBallLeft(){
    chassis.moveToPoint(-9.3, 34, 790, {.forwards=true, .minSpeed=1, .earlyExitRange=0.5}, true);
    wait(175);
    unloader.firePiston(false);
    wait(100);
    reset_y();
    chassis.waitUntilDone();
    reset_y();
    chassis.swingToPoint(-4, 57, DriveSide::LEFT, 250, {.maxSpeed=110}, true);
    chassis.waitUntilDone();
   
    robot.ram(80, 50); //go a bit forward
     unloader.firePiston(true);
     robot.ram(80, 250);
     wait(400); //wait to grab balls
    reset_y();
}
void grabSecondBallRight(){
    chassis.moveToPoint(-9.3, -34, 790, {.forwards=true, .minSpeed=1, .earlyExitRange=0.5}, true);
    wait(175);
    unloader.firePiston(false);
    wait(100);
    reset_y();
    chassis.waitUntilDone();
    reset_y();
    chassis.swingToPoint(-4, -57, DriveSide::LEFT, 250, {.maxSpeed=110}, true);
    chassis.waitUntilDone();
   
    robot.ram(80, 50); //go a bit forward
     unloader.firePiston(true);
     robot.ram(80, 250);
     wait(400); //wait to grab balls
    reset_y();
}
void goBackToGoalLeft4_5(){
    unloader.firePiston(false);
    wait(200);

    //arc backwards
    leftMotors.move_voltage(-400);
    rightMotors.move_voltage(-7500);
    while(chassis.getPose().theta < 35) wait(15);
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);

    chassis.moveToPoint(-31, 38.5, 900, {.forwards=false, .minSpeed=1,.earlyExitRange=1}, true);
    while(chassis.getPose().x > -29) wait(15);
    reset_y();
    chassis.waitUntilDone();

    //swing into goal
    chassis.swingToHeading(268, DriveSide::LEFT, 650, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed=50, .earlyExitRange=1}, true);
    while(chassis.getPose().theta < 260) wait(15);
    chassis.cancelMotion();

     //brake motors: prevent drift
    leftMotors.move_voltage(-1000);
    rightMotors.move_voltage(3000);
    wait(180);

    wait(150); //wait a little more could be more drift
    reset_y();

    //go into goal
    chassis.moveToPoint(-19, 47, 800, {.forwards=false, .minSpeed=80}, false);
    score_toggle.firePiston(true);
    wait(1500); //score balls
}
void goBackToGoalRight4_5(){
    unloader.firePiston(false);
    wait(200);

    //arc backwards
    leftMotors.move_voltage(-7500);
    rightMotors.move_voltage(-400);
    while(chassis.getPose().theta < 145) wait(15);
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);

    chassis.moveToPoint(-31, -38.5, 900, {.forwards=false, .minSpeed=1,.earlyExitRange=1}, true);
    while(chassis.getPose().x > -29) wait(15);
    reset_y();
    chassis.waitUntilDone();

    //swing into goal
    chassis.swingToHeading(272, DriveSide::LEFT, 650, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed=50, .earlyExitRange=1}, true);
    while(chassis.getPose().theta < 280) wait(15);
    chassis.cancelMotion();

     //brake motors: prevent drift
    leftMotors.move_voltage(3000);
    rightMotors.move_voltage(-1000);
    wait(180);

    wait(150); //wait a little more could be more drift
    reset_y();

    //go into goal
    chassis.moveToPoint(-19, -47, 800, {.forwards=false, .minSpeed=80}, false);
    score_toggle.firePiston(true);
    wait(1500); //score balls
}
void getMatchLoadAndGoToMiddleLeft(){
    intake.intake();
    robot.reset_y();
    chassis.setPose(-30, chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-65, 46.4, 270, 1400, {}, true); //go for matchloaders
    wait(100);
    unloader.firePiston(true);
    score_toggle.firePiston(false);
    while(chassis.getPose().x > -40) wait(15);
    chassis.cancelMotion();
    chassis.moveToPoint(-67, 46.4, 450, {.maxSpeed=60}, true); //go for matchloaders
    reset_y();
    chassis.waitUntilDone();

    robot.ram(80, 400); //get three matchloads
    moveStraight(-8, 350, {}, false);
    chassis.turnToHeading(270, 150, {}, false);
    robot.reset_x(false, true, true);
    robot.reset_y(false, true, true);
    chassis.setPose(chassis.getPose().x, chassis.getPose().y + 0.5, chassis.getPose().theta); //we are off by around half an inch
    printf("NEW POS: X: %.2f, Y: %.2f\n", chassis.getPose().x, chassis.getPose().y);
    intake.stop();
    chassis.turnToPoint(-10, 10, 400, {.forwards=false}, true);
    chassis.moveToPoint(-10, 10, 1600, {.forwards=false}, true);
    while(chassis.getPose().x < -13.5) wait(15); //once past certain point prefire midgoal
    middle_goal.firePiston(true);
    intake.intake();
}
void getMatchLoadAndGoToMiddleRight(){
    intake.intake();
    robot.reset_y();
    chassis.setPose(-30, chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-65, -46.4, 270, 1400, {}, true); //go for matchloaders
    wait(100);
    unloader.firePiston(true);
    score_toggle.firePiston(false);
    while(chassis.getPose().x > -40) wait(15);
    chassis.cancelMotion();
    chassis.moveToPoint(-67, -46.4, 450, {.maxSpeed=60}, true); //go for matchloaders
    reset_y();
    chassis.waitUntilDone();

    robot.ram(80, 400); //get three matchloads
    moveStraight(-8, 350, {}, false);
    chassis.turnToHeading(270, 150, {}, false);
    robot.reset_x(false, true, true);
    robot.reset_y(false, true, true);
    chassis.setPose(chassis.getPose().x, chassis.getPose().y + 0.5, chassis.getPose().theta); //we are off by around half an inch
    printf("NEW POS: X: %.2f, Y: %.2f\n", chassis.getPose().x, chassis.getPose().y);
    intake.stop();
    chassis.turnToPoint(-10, -10, 400, {.forwards=false}, true);
    chassis.moveToPoint(-10, -10, 1600, {.forwards=false}, true);
    while(chassis.getPose().x < -13.5) wait(15); //once past certain point prefire midgoal
    middle_goal.firePiston(true);
    intake.intake();
}
void matchloadLowGoal(){
    intake.intake();
    robot.reset_y();
    chassis.setPose(-30, chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-65, -46.4, 270, 1400, {}, true); //go for matchloaders
    wait(100);
    unloader.firePiston(true);
    score_toggle.firePiston(false);
    while(chassis.getPose().x > -40) wait(15);
    chassis.cancelMotion();
    chassis.moveToPoint(-67, -46.4, 450, {.maxSpeed=60}, true); //go for matchloaders
    reset_y();
    chassis.waitUntilDone();

    robot.ram(80, 400); //get three matchloads
    moveStraight(-8, 350, {}, false);
    chassis.turnToHeading(270, 150, {}, false);
    robot.reset_x(false, true, true);
    robot.reset_y(false, true, true);
    chassis.setPose(chassis.getPose().x, chassis.getPose().y + 0.5, chassis.getPose().theta); //we are off by around half an inch
    printf("NEW POS: X: %.2f, Y: %.2f\n", chassis.getPose().x, chassis.getPose().y);
    intake.stop();
    unloader.firePiston(false);
    chassis.turnToPoint(-10, -10, 600, {.forwards=true}, false);
    chassis.moveToPoint(-10, -10, 1600, {.forwards=true}, false);
    intake.outake();
}
void fivePlusFourBlueLeft(){
    firstTrioArcLeftM();
    grabSecondBallLeft();
    goBackToGoalLeft4_5();
    getMatchLoadAndGoToMiddleLeft();
}

void fivePlusFourBlueRight(){
    firstTrioArcRightM();
    grabSecondBallRight();
    goBackToGoalRight4_5();
    getMatchLoadAndGoToMiddleRight();
}

void threePlusFourBlueRight(){
    middle_goal.firePiston(true);
    firstTrioArcRight();
    threePlusFourGoIntoGoal();
    matchloadLowGoal();


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
    // awp();
    // sevenBallBlueLeft();
    // fivePlusFourBlueLeft();
    sevenBallBlueRight();
    // fivePlusFourBlueRight();
    // threePlusFourBlueRight();
    // fivePlusFourBlueLeft();
    // af();
    //skills_119();

    
    //scoreAllRed(3000, pushback::Robot::BLUE);

   
}













/**
 * Runs in driver control
 */
bool driver_skills = true;
bool currentUpClicked = false;
bool upLastClicked = false;
bool upClicked = false;

void opcontrol() {

    chassis.setPose(40, 0, 270);
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
