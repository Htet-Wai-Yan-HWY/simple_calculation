#include <iostream>

int main() {
    double initialPosition, initialVelocity, acceleration, time;

    // Input initial position, initial velocity, acceleration, and time
    std::cout << "Enter initial position: ";
    std::cin >> initialPosition;

    std::cout << "Enter initial velocity: ";
    std::cin >> initialVelocity;

    std::cout << "Enter acceleration: ";
    std::cin >> acceleration;

    std::cout << "Enter time: ";
    std::cin >> time;

    // Calculate final position using kinematic equation: s = ut + 0.5 * a * t^2
    double finalPosition = initialPosition + initialVelocity * time + 0.5 * acceleration * time * time;

    // Output the final position
    std::cout << "Final position: " << finalPosition << std::endl;

    return 0;
}