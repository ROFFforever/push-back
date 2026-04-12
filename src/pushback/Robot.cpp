#include "pushback/Robot.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace pushback {
// We are using "VEX Gaming Positioning System" for these calculations
lemlib::Pose Robot::reset_y() {
    pushback::Wall_Sen* sen = nullptr;
    float x = chassis.getPose().x;
    float y = chassis.getPose().y;
    float globalTheta = chassis.getPose().theta; // I think it's in radians
    const int side = get_side(globalTheta);
    bool negativeY = false;

    // find the correct sensor to use to reset
    if ((side == pushback::Wall_Sen::FRONT)) { // if it's facing forward
        if (y < 0) { //and y is negative, use the back sensor 
            negativeY = true;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::BACK) {
                    sen = &s;
                    break;
                }
            }
        }
        else{ //if y>0 make sense to use front sensor
            negativeY = false;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::FRONT) {
                    sen = &s;
                    break;
                }
            }
        }
    } 

    else if ((side == pushback::Wall_Sen::RIGHT)) { // if it's facing right
        if (y < 0) { //and y is negative, use the back sensor 
            negativeY = true;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::RIGHT) {
                    sen = &s;
                    break;
                }
            }
        }
        else{ //if y>0 make sense to use left sensor
            negativeY = false;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::LEFT) {
                    sen = &s;
                    break;
                }
            }
        }
    }

    else if ((side == pushback::Wall_Sen::BACK)) { // if it's facing right
        if (y < 0) { //and y is negative, use the front sensor 
            negativeY = true;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::FRONT) {
                    sen = &s;
                    break;
                }
            }
        }
        else{ //if y>0 make sense to use back sensor
            negativeY = false;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::BACK) {
                    sen = &s;
                    break;
                }
            }
        }
    }
    else if ((side == pushback::Wall_Sen::LEFT)) { // if it's facing right
        if (y < 0) { //and y is negative, use the left sensor 
            negativeY = true;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::LEFT) {
                    sen = &s;
                    break;
                }
            }
        }
        else{ //if y>0 make sense to use right sensor
            negativeY = false;
            for (pushback::Wall_Sen& s : distance_sensors) {
                if (s.TYPE == pushback::Wall_Sen::RIGHT) {
                    sen = &s;
                    break;
                }
            }
        }
    }

    if(sen == nullptr){
        return lemlib::Pose(-1,-1,-1); // if no sensor was found, return ERROR pose
    }

    float dist_from_wall = get_dist_from_wall(sen);

    if(negativeY){
        return lemlib::Pose(x, dist_from_wall - 70, globalTheta);
    }else{
        return lemlib::Pose(x, 70 - dist_from_wall, globalTheta);
    }
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
        return pushback::Wall_Sen::FRONT;
    } else if (index == 1) {
        return pushback::Wall_Sen::RIGHT;
    } else if (index == 2) {
        return pushback::Wall_Sen::BACK;
    } else if (index == 3) {
        return pushback::Wall_Sen::LEFT;
    } else {
        return -1; // shouldn't be possible
    }
}

float Robot::get_dist_from_wall(pushback::Wall_Sen* sen) {
    float globalTheta = chassis.getPose().theta; // I think it's in radians
    float dist = sen->get_dist();
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
