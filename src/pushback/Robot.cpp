#include "pushback/Robot.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace pushback {

pushback::Wall_Sen* Robot::find_sensor(int const TYPE) {
    for (pushback::Wall_Sen& s : distance_sensors) {
        if (s.TYPE == TYPE) { return &s; }
    }
    return nullptr;
}

// We are using "VEX Gaming Positioning System" for these calculations
bool Robot::reset_y(bool async, bool check_relative, bool poll) {
    //run async if possible
    if (async) {
        pros::Task([this]() {
            this->reset_y(false);
        });
        return false;
    }

    // wall sensor that will be used to reset
    pushback::Wall_Sen* sen = nullptr;

    // current robot position
    float x = chassis.getPose().x;
    float y = chassis.getPose().y;
    float globalTheta = chassis.getPose().theta; // In degrees
    float new_y = y;

    // Which "SIDE" robot facing where 0 degrees corresponds to FRONT(using VGP system)
    const int side = get_side(globalTheta);
    bool negativeY = false;

    // find the correct sensor to use to reset
    if ((side == pushback::Wall_Sen::FRONT)) { // if it's facing forward
        if (y < 0) { // and y is negative, use the back sensor
            negativeY = true;
            sen = find_sensor(Wall_Sen::BACK);
        } else { // if y>0 make sense to use front sensor
            negativeY = false;
            sen = find_sensor(Wall_Sen::FRONT);
        }
    }
    else if ((side == pushback::Wall_Sen::RIGHT)) { // if it's facing right
        if (y < 0) { // and y is negative, use the back sensor
            negativeY = true;
            sen = find_sensor(Wall_Sen::RIGHT);
        } else { // if y>0 make sense to use left sensor
            negativeY = false;
            sen = find_sensor(Wall_Sen::LEFT);
        }
    }
    else if ((side == pushback::Wall_Sen::BACK)) { // if it's facing right
        if (y < 0) { // and y is negative, use the front sensor
            negativeY = true;
            sen = find_sensor(Wall_Sen::FRONT);
        } else { // if y>0 make sense to use back sensor
            negativeY = false;
            sen = find_sensor(Wall_Sen::BACK);
        }
    } else if ((side == pushback::Wall_Sen::LEFT)) { // if it's facing left
        if (y < 0) { // and y is negative, use the left sensor
            negativeY = true;
            sen = find_sensor(Wall_Sen::LEFT);
        } else { // if y>0 make sense to use right sensor
            negativeY = false;
            sen = find_sensor(Wall_Sen::RIGHT);
        }
    }

    // if can't find any sensor to use, return ERROR pose
    if (sen == nullptr) { return false; } //no sensor found

    float dist = 0;

    if(poll){
    //find median val
    float vals[3];

    for (int i = 0; i < 3; i++) {
        float d = get_dist_from_wall(sen);
        if (d == -1) return false;
        vals[i] = d;
        pros::delay(10);
    }
    //sort lowest to highest
    std::sort(vals, vals + 3);
    dist = vals[1]; // median
    }else{
        dist = get_dist_from_wall(sen);
    }

    // if in negative quadrant subtract 70 to make negative
    if (negativeY) {
        new_y = dist - 70;
    } else {
        new_y = 70 - dist;
    }

    //check if too far from current position
    if(check_relative){ //check if setting enabled
        float error = fabs(new_y - y);
        if(error > 8){ //pretty off
            return false;
        }
    }
    //set position
    chassis.setPose(chassis.getPose().x, new_y, chassis.getPose().theta);
    
    //position sucessfully reset
    return true; 
}

// We are using "VEX Gaming Positioning System" for these calculations
bool Robot::reset_x(bool async, bool check_relative, bool poll) {
    //run async if possible
    if (async) {
        pros::Task([this]() {
            this->reset_x(false);
        });
        return false;
    }
    // wall sensor that will be used to reset
    pushback::Wall_Sen* sen = nullptr;

    // current robot position
    float x = chassis.getPose().x;
    float y = chassis.getPose().y;
    float globalTheta = chassis.getPose().theta; // In degrees
    float new_x = x;

    // Which "SIDE" robot facing where 0 degrees corresponds to FRONT(using VGP system)
    const int side = get_side(globalTheta);
    bool negativeX = false;

    // find the correct sensor to use to reset
    if ((side == pushback::Wall_Sen::FRONT)) { // if it's facing forward
        if (x < 0) { // and x is negative, use the left sensor
            negativeX = true;
            sen = find_sensor(Wall_Sen::LEFT);
        } else { // if x>0 make sense to use right sensor
            negativeX = false;
            sen = find_sensor(Wall_Sen::RIGHT);
        }
    }
    else if ((side == pushback::Wall_Sen::RIGHT)) { // if it's facing right
        if (x < 0) { // and x is negative, use the back sensor
            negativeX = true;
            sen = find_sensor(Wall_Sen::BACK);
        } else { // if x>0 make sense to use left sensor
            negativeX = false;
            sen = find_sensor(Wall_Sen::FRONT);
        }
    }
    else if ((side == pushback::Wall_Sen::BACK)) { // if it's facing back
        if (x < 0) { // and x is negative, use the right sensor
            negativeX = true;
            sen = find_sensor(Wall_Sen::RIGHT);
        } else { // if x>0 make sense to use left sensor
            negativeX = false;
            sen = find_sensor(Wall_Sen::LEFT);
        }
    } else if ((side == pushback::Wall_Sen::LEFT)) { // if it's facing left
        if (x < 0) { // and x is negative, use the front sensor
            negativeX = true;
            sen = find_sensor(Wall_Sen::FRONT);
        } else { // if x>0 make sense to use back sensor
            negativeX = false;
            sen = find_sensor(Wall_Sen::BACK);
        }
    }

    // if can't find any sensor to use, return ERROR pose
    if (sen == nullptr) { return false; } // couldn't find sensor just dont reset

    float dist = 0;

    if(poll){
    //find median val
    float vals[3];

    for (int i = 0; i < 3; i++) {
        float d = get_dist_from_wall(sen);
        if (d == -1) return false;
        vals[i] = d;
        pros::delay(10);
    }
    //sort lowest to highest
    std::sort(vals, vals + 3);
    dist = vals[1]; // median
    }else{
        dist = get_dist_from_wall(sen);
    }

    // if in negative quadrant subtract 70 to make negative
    if (negativeX) {
        new_x = dist-70;
    } else {
        new_x = 70-dist;
    }

    //check if too far from current position
    if(check_relative){ //check if setting enabled
        float error = fabs(new_x - x);
        if(error > 10){ //pretty off
            return false;
        }
    }

    //set position
    chassis.setPose(new_x, chassis.getPose().y , chassis.getPose().theta);

    //passed all tests
    return true;
}

int Robot::get_side(float angle) {
    // normalize angle first into 0 to 360(could be like 745 or -25)
    float normalized = std::fmod(std::fmod(angle, 360.0f) + 360.0f, 360.0f);

    // 2. Shift by 45 and divide by 90 to get an index (0, 1, 2, or 3)
    // Adding 45 makes 315-45 become the "0" bucket.
    int index = static_cast<int>((normalized + 45.0f)) / 90;

    // 3. Wrap index 4 back to 0 (for the 315-360 range)
    index %= 4;

    if (index == 0) {
        return pushback::Wall_Sen::FRONT; //if within (-45,45) basically facing 0 degrees
    } else if (index == 1) {
        return pushback::Wall_Sen::RIGHT; //if within (45, 135) basically facing 90 degrees
    } else if (index == 2) {
        return pushback::Wall_Sen::BACK; //if within (135, 225) basically facing 180 degrees
    } else if (index == 3) {
        return pushback::Wall_Sen::LEFT; //if within (225, 315) basically facing 270 degrees
    } else {
        return -1; // shouldn't be possible
    }
}

float Robot::get_dist_from_wall(pushback::Wall_Sen* sen) {
    float globalTheta = chassis.getPose().theta; // In degrees
    float dist = sen->get_dist();
    if(dist == PROS_ERR){ //ERROR VAL
        return -1; //error val
    }
    float localTheta = std::fmod(std::fmod(globalTheta + 45.0f, 90.0f) + 90.0f, 90.0f) -
                       45.0f; // add 45 in the beginning to account for negative values
    localTheta = localTheta * (std::numbers::pi_v<float> / 180.0f);
    return (dist + sen->vert_offset) * std::cos(localTheta) +
           (sen->horiz_offset * std::sin(localTheta)); // get dist from center or robot to wall
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
             std::vector<pushback::Wall_Sen> distance_sensors, pros::Imu* imu, pros::Controller& controller,
             pros::Optical* optical, int color)
    : chassis(chassis),
      intake_1(intake1),
      intake_2(intake2),
      intake_3(intake3),
      piston_1(piston1),
      piston_2(piston2),
      piston_3(piston3),
      piston_4(piston4),
      distance_sensors(distance_sensors),
      inertial(imu),
      controller(controller),
      optical(optical),
      color(color) {
    // Nothing else needed — all members are set to the provided objects
}

} // namespace pushback
