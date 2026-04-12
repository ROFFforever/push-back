#include "pushback/Wall_Sen.hpp"

namespace pushback {

    Wall_Sen::Wall_Sen(int port, float horiz_offset, float vert_offset, const int type) 
        : horiz_offset(horiz_offset), 
          vert_offset(vert_offset), 
          sensor(new pros::Distance(port)),
          TYPE(type)
    {
    }

    float Wall_Sen::get_dist() {
        return (sensor->get_distance() / 25.4); // Convert mm to inches
    }

}
