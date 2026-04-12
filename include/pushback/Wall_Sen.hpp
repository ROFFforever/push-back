#pragma once
#include "pros/distance.hpp"

namespace pushback{
    class Wall_Sen{
        public:
        /**
         * @brief Get distance from wall to sensor in in
         * 
         * @return float 
         */
        float get_dist();

        Wall_Sen(int port, float horiz_offset, float vert_offset, const int type);
        static const int BACK = 0;
        static const int LEFT = 1;
        static const int FRONT = 2;
        static const int RIGHT = 3;
        float horiz_offset; //use images to help know, but basically, if wall sensor to the right of the center of robot, offset is positive, to the left, offset is negative
        float vert_offset; //always positive
        const int TYPE; //wheter its back, front, left, or right relative to the intake of the robot(front distance sensor is intake)
        pros::Distance* sensor;
    };
}