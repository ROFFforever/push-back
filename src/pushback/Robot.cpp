#include "pushback/Robot.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace pushback {

float mmToInches(float mm) { return mm / 25.4; }

// Returns the deviation from the closest cardinal angle (0, 90, 180, 270)
double deviationFromCardinal(double angle) {
    // Normalize angle to [0, 360)
    angle = fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;

    // Find remainder when divided by 90
    double remainder = fmod(angle, 90.0);

    // The deviation is the minimum of:
    // - distance to the next lower cardinal (remainder)
    // - distance to the next higher cardinal (90 - remainder)
    return std::min(remainder, 90.0 - remainder);
}

float Robot::get_distance_left_side() {
    // Get distance in millimeters from the distance sensor
    float mmDistance = distance_sensor_left_side->get();

    // Check for sensor error or out of range
    if (mmDistance == PROS_ERR || mmDistance >= 9999) { return PROS_ERR_F; }

    // Convert to inches
    float distance = mmToInches(mmDistance);

    // Get robot's angle and ensure it's in degrees
    float theta = chassis.getPose(false).theta; // false for degrees

    // find the closest cardinal angle, this is important as we need to determine
    // which wall the distance sensor is pointing at
    float closest_cardinal_angle = round(fabs(theta) / 90.0f) * 90.0f;

    // Wrap result into [0, 360)
    closest_cardinal_angle = fmodf(closest_cardinal_angle, 360.0f);

    // Normalize theta to be between 0 and 90 degrees
    theta = fmod(fabs(theta), 90.0);

    // Get relative angle to wall, this wall is perpendicular so we have to offset by 90 degrees
    if ((closest_cardinal_angle == 90 || closest_cardinal_angle == 270 || closest_cardinal_angle == 180) &&
        theta != 0 && theta > 45) {
        // Adjust distance based on robot's orientation
        float adjustedDistance = distance * fabs(sin(theta * M_PI / 180.0));
        adjustedDistance +=
            fabs(sin(theta * M_PI / 180.0)) * wall_distance_offset; // Apply offset with angle consideration
        return adjustedDistance;
    } else {
        // Adjust distance based on robot's orientation
        float adjustedDistance = distance * fabs(cos(theta * M_PI / 180.0));
        adjustedDistance +=
            fabs(cos(theta * M_PI / 180.0)) * wall_distance_offset; // Apply offset with angle consideration
        return adjustedDistance;
    }
}

float Robot::get_distance_right_side() {
    // Get distance in millimeters from the distance sensor
    float mmDistance = distance_sensor_right_side->get();

    // Check for sensor error or out of range
    if (mmDistance == PROS_ERR || mmDistance >= 9999) { return PROS_ERR_F; }

    // Convert to inches
    float distance = mmToInches(mmDistance);

    // Get robot's angle and ensure it's in degrees
    float theta = chassis.getPose(false).theta; // false for degrees

    // find the closest cardinal angle, this is important as we need to determine
    // which wall the distance sensor is pointing at
    float closest_cardinal_angle = round(fabs(theta) / 90.0f) * 90.0f;

    // Wrap result into [0, 360)
    closest_cardinal_angle = fmodf(closest_cardinal_angle, 360.0f);

    // Normalize theta to be between 0 and 90 degrees
    theta = fmod(fabs(theta), 90.0);

    // Get relative angle to wall, this wall is perpendicular so we have to offset by 90 degrees
    if ((closest_cardinal_angle == 90 || closest_cardinal_angle == 270 || closest_cardinal_angle == 180) &&
        theta != 0 && theta > 45) {
        // Adjust distance based on robot's orientation
        float adjustedDistance = distance * fabs(sin(theta * M_PI / 180.0));
        adjustedDistance +=
            fabs(sin(theta * M_PI / 180.0)) * wall_distance_offset; // Apply offset with angle consideration
        return adjustedDistance;
    } else {
        // Adjust distance based on robot's orientation
        float adjustedDistance = distance * fabs(cos(theta * M_PI / 180.0));
        adjustedDistance +=
            fabs(cos(theta * M_PI / 180.0)) * wall_distance_offset; // Apply offset with angle consideration
        return adjustedDistance;
    }
}
float Robot::get_distance_front() {
    // Get distance in millimeters from the distance sensor
    float mmDistance = distance_sensor_right_side->get();

    // Check for sensor error or out of range
    if (mmDistance == PROS_ERR || mmDistance >= 9999) { return PROS_ERR_F; }

    // Convert to inches
    float distance = mmToInches(mmDistance);

    // Get robot's angle and ensure it's in degrees
    float theta = chassis.getPose(false).theta; // false for degrees

    // find the closest cardinal angle, this is important as we need to determine
    // which wall the distance sensor is pointing at
    float closest_cardinal_angle = round(fabs(theta) / 90.0f) * 90.0f;

    // Wrap result into [0, 360)
    closest_cardinal_angle = fmodf(closest_cardinal_angle, 360.0f);

    // Normalize theta to be between 0 and 90 degrees
    theta = fmod(fabs(theta), 90.0);

    // Get relative angle to wall, this wall is perpendicular so we have to offset by 90 degrees
    if ((closest_cardinal_angle == 90 || closest_cardinal_angle == 270 || closest_cardinal_angle == 180) &&
        theta != 0 && theta > 45) {
        // Adjust distance based on robot's orientation
        float adjustedDistance = distance * fabs(sin(theta * M_PI / 180.0));
        adjustedDistance += fabs(sin(theta * M_PI / 180.0)) * 6.5; // Apply offset with angle consideration
        return adjustedDistance;
    } else {
        // Adjust distance based on robot's orientation
        float adjustedDistance = distance * fabs(cos(theta * M_PI / 180.0));
        adjustedDistance += fabs(cos(theta * M_PI / 180.0)) * 6.5; // Apply offset with angle consideration
        return adjustedDistance;
    }

    // other wall is straight so we dont have to offset
}

void Robot::go_until_front(float distance, int speed, int timeout, int resistance, float desired_theta, bool initBurst, int burst_time) {
    float closest_cardinal_angle = round(fabs(chassis.getPose().theta) / 90.0f) * 90.0f;
    float initDistance = get_distance_front();
    float last_distance = initDistance;
    float current_distance = initDistance;
    float theta = chassis.getPose().theta;
    lemlib::Timer timer(timeout);
    float error = 80;
    if (initDistance == PROS_ERR || initDistance > 80) {
        chassis.arcade(speed, 0);
        error = 80; // give it some higher error for now as readings above 80 inches are iffy
    } else error = get_distance_front() - distance; //else error is valid

    while (!timer.isDone() && error > 1) { // as long timer isnt done and is within one inch of target
        theta = chassis.getPose().theta;
        float dev = desired_theta - theta;
        float reading = get_distance_front();
        if ((reading != PROS_ERR) && ((fabs(reading - last_distance)) < 9)) {
            last_distance = reading;
            error = reading - distance;
            if(error < 10){
                chassis.arcade(speed * fabs(error/10), dev * resistance); // go at speed with specified turn
            }
            if(timer.getTimePassed() < burst_time && initBurst){
                 chassis.arcade(100, dev * resistance); // go at speed with specified turn
            }else{
                chassis.arcade(speed, dev * resistance); // go at speed with specified turn
            }
        }
        pros::delay(5);
    }
    chassis.arcade(0, 0);
}

void Robot::set_distance_offset(float offset) { wall_distance_offset = offset; }

void Robot::turnToPointSide(lemlib::Pose target, int timeout) {
    float initTheta = chassis.getPose().theta;
    float y_dist = fabs(70 - get_distance_left_side());
    float triangle_y = (target.y - y_dist); // get the y of the triangle
    float triangle_x = ((target.x) - (chassis.getPose().x)); // get the x of the triangle
    float theta = atan(triangle_x / triangle_y) * (180 / M_PI); // get angle and change to degrees
    chassis.turnToHeading(initTheta + theta, timeout); // changed angle
}

bool Robot::filter_x() {
    float robot_x = chassis.getPose().x;
    if (fabs((70 - get_distance_left_side()) - fabs(robot_x)) > 5)
        return false; // if the difference between wall and robot_x is more than 8 inches than no good
    return true;
}

bool Robot::filter_y() {
    float robot_y = chassis.getPose().y;
    if (fabs((70 - get_distance_left_side()) - fabs(robot_y)) > 5)
        return false; // if the difference between wall and robot_y is more than 8 inches than no good
    return true;
}

void Robot::reset_x() {
    float distance = get_distance_left_side(); // Get distance from the wall using distance sensor
    float robot_x = chassis.getPose().x; // Get x position of robot
    float robot_y = chassis.getPose().y;
    float theta = chassis.getPose().theta;
    float reset_x = 70 - distance; // Calculate what the x position should be based on distance from wall

    // ERROR HANDLING
    if (distance == PROS_ERR || distance >= 9999) return; // getting distance failed

    if (fabs(robot_x) < 35) // if 45 inches away from wall, distance resetting becomes risky
        return;
    // If the reading is not good, do not reset; It's inconsistent with what drivetrain is reporting
    if (!filter_x()) return;

    if (robot_x > 0) { // on the red loader side of field, x is positive(using vex coords)
        chassis.setPose(reset_x, robot_y, theta);
    } else { // on the blue loader side of the field, x is negative
        chassis.setPose(-reset_x, robot_y, theta);
    }
}

void Robot::reset_y() {
    float distance = get_distance_left_side(); // Get distance from the wall using distance sensor
    float robot_x = chassis.getPose().x; // Get x position of robot
    float robot_y = chassis.getPose().y;
    float theta = chassis.getPose().theta;
    float reset_y = 70 - distance; // Calculate what the x position should be based on distance from wall

    // ERROR HANDLING
    if (distance == PROS_ERR || distance >= 9999) return; // getting distance failed

    if (fabs(robot_y) < 35) // if 45 inches away from wall, distance resetting becomes risky
        return;
    // If the reading is not good, do not reset; It's inconsistent with what drivetrain is reporting
    if (!filter_y()) return;

    if (robot_y > 0) { // on the red loader side of field, x is positive(using vex coords)
        chassis.setPose(robot_x, reset_y, theta);
    } else { // on the blue loader side of the field, x is negative
        chassis.setPose(robot_x, -reset_y, theta);
    }
}

bool Robot::safe_to_reset_x() {
    float robot_x = chassis.getPose().x;
    float theta = chassis.getPose().theta;

    // find the closest cardinal angle, this is important as we need to determine
    // which wall the distance sensor is pointing at
    float closest_cardinal_angle = round(theta / 90.0f) * 90.0f;

    // Wrap result into [0, 360)
    closest_cardinal_angle = fmodf(closest_cardinal_angle, 360.0f);

    // check if facing the wrong wall
    if (closest_cardinal_angle == 90 || closest_cardinal_angle == 270) { return false; }

    // Normalize theta to be between 0 and 90 degrees
    theta = fmod(fabs(theta), 90.0);

    // if 35 inches away from wall, distance resetting becomes risky
    if (fabs(robot_x) < 45) return false;

    // if robot is not relatively straight to the wall, don't reset
    if (theta > 10) return false;

    // make sure not to reset pos in loader areas as the tubes stick out, also not near the far ends of field as walls
    // can change direction
    float abs_y = fabs(chassis.getPose().y);
    float abs_x = fabs(robot_x);
    if ((abs_y >= 41.5 && abs_y <= 52) && (abs_y > 46 && abs_x > 46)) { return false; }

    // if passed everything, safe to reset
    return true;
}

bool Robot::safe_to_reset_y() {
    float robot_y = chassis.getPose().y;
    float theta = chassis.getPose().theta;

    // find the closest cardinal angle, this is important as we need to determine
    // which wall the distance sensor is pointing at
    float closest_cardinal_angle = round(theta / 90.0f) * 90.0f;

    // Wrap result into [0, 360)
    closest_cardinal_angle = fmodf(closest_cardinal_angle, 360.0f);

    // check if facing the wrong wall
    if (closest_cardinal_angle == 0 || closest_cardinal_angle == 360 || closest_cardinal_angle == 180) { return false; }

    // Normalize theta to be between 0 and 90 degrees
    theta = fmod(fabs(theta), 90.0);

    // if 25 inches away from wall, distance resetting becomes risky
    if (fabs(robot_y) < 45) return false;

    // if robot is not relatively straight to the wall, don't reset
    if (theta > 10) return false;

    // Dont reset near middle goals and scoring pillars
    float abs_x = fabs(chassis.getPose().x);
    float abs_y = fabs(robot_y);
    if ((abs_x > 27) && (abs_x > 46 && abs_y > 46)) { return false; }

    // if passed everything, safe to reset
    return true;
}

void Robot::jiggle(float magnitude, int cycle_time, int time) {
    int elapsed_time = 0;
    while (elapsed_time < time) {
        // Move forward
        chassis.arcade(magnitude, 0);
        pros::delay(cycle_time / 2);
        elapsed_time += cycle_time / 2;

        // Move backward
        chassis.arcade(-magnitude, 0);
        pros::delay(cycle_time / 2);
        elapsed_time += cycle_time / 2;
    }

    // Stop movement
    chassis.arcade(0, 0);
}

void Robot::ram(int magnitude, int time) {
    chassis.arcade(magnitude, 0);
    pros::delay(time);
    chassis.arcade(0, 0);
}

// Constructor: now takes chassis instead of separate motor groups
Robot::Robot(lemlib::Chassis& chassis, pros::Motor* intake1, pros::Motor* intake2, pros::Motor* intake3,
             pushback::Piston* piston1, pushback::Piston* piston2, pushback::Piston* piston3, pushback::Piston* piston4,
             pros::Distance* distance_sensor_left_side, pros::Distance* distance_sensor_right_side, pros::Imu* imu,
             pros::Controller& controller, pros::Optical* optical, int color)
    : chassis(chassis),
      intake_1(intake1),
      intake_2(intake2),
      intake_3(intake3),
      piston_1(piston1),
      piston_2(piston2),
      piston_3(piston3),
      piston_4(piston4),
      distance_sensor_left_side(distance_sensor_left_side),
      distance_sensor_right_side(distance_sensor_right_side),
      inertial(imu),
      controller(controller),
      optical(optical),
      color(color) {
    // Nothing else needed — all members are set to the provided objects
}

} // namespace pushback
