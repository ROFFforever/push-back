#include "main.h"
#include <iostream>
#include <string>

void opcontrol() {
    std::string input_data;
    // Optional: Clear the screen once at the start
    pros::screen::erase();

    while (true) {
        // Listen for data from the PC
        if (std::cin >> input_data) {
            // 1. Print response back to PC
            std::cout << "Brain received: " << input_data << std::endl;
            
            // 2. Clear previous line and print new data to Brain Screen
            // Added .c_str() to fix the "char" error
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Data: %s", input_data.c_str());
        }
        pros::delay(20);
    }
}
